/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "class/cdc/cdc_device.h"
#include "cmsis_gcc.h"
#include "stm32g474xx.h"
#include "stm32g4xx_hal_def.h"
#include "stm32g4xx_hal_gpio.h"
#include "stm32g4xx_hal_uart.h"
#include "stm32g4xx_hal_uart_ex.h"
#include "tinygps.h"
#include "rosjam.h"
#include "tusb_config.h"
#include "tusb.h"
#include "cobs.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define WATCHDOG_ENABLE
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
IWDG_HandleTypeDef hiwdg;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_FS;

/* USER CODE BEGIN PV */
// Define ms between reporting of diagnostic data (valid/error GPS packets received)
#define DIAG_REPORT_PERIOD_MS 500

#define GPS_TYPE GPS_UBX

char satellites[50];
char latitude[50];
char longitude[50];
char heading[50];

gps_t gps;

#define GPS_BUFFER_SIZE (UBX_MAX_PAYLOAD)
static uint8_t gps_buffers[2][GPS_BUFFER_SIZE];
static int buffer_sizes[2];
static volatile int gps_index = 0;
static int gps_ready[2] ={0, 0};

UART_HandleTypeDef *pantilt_uart = &huart1;

// Pantilt
#define PANTILT_BUFFER_SIZE 100
static uint8_t pantilt_buffers[2][PANTILT_BUFFER_SIZE];
static volatile int pantilt_index = 0;
volatile int pantilt_ready = 0;

UART_HandleTypeDef *term_uart = &huart4;

// Terminal
#define TERM_BUFFER_SIZE 100
static uint8_t term_buffers[2][TERM_BUFFER_SIZE];
static volatile int term_index = 0;
static volatile int term_ready = 0;
static volatile uint16_t term_size = 0;

// UART board commands
#define CMD_BUFFER_SIZE 256
static uint8_t cmd_buf[CMD_BUFFER_SIZE];
static int cmd_len = 0;
static bool cmd_overflow = false;
static cobs_reader_t usb_cobs_reader;
static uint8_t usb_chunk[64];
static uint32_t usb_chunk_len = 0;

// Diagnostic drop/error counters
static volatile uint32_t usb_tx_dropped = 0;
static volatile uint32_t uart_errors = 0;

// TX ring for a UART relay
#define TXQ_SIZE 512
#define TXQ_MASK (TXQ_SIZE - 1)

typedef struct {
  UART_HandleTypeDef *huart;
  uint8_t  buf[TXQ_SIZE];
  volatile uint16_t head;
  volatile uint16_t tail;
  volatile uint16_t inflight;
  volatile uint8_t  busy;
  volatile uint32_t dropped;
} txq_t;

static txq_t pantilt_txq;
static txq_t term_txq;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USB_PCD_Init(void);
static void MX_UART4_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_IWDG_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Enqueues n bytes for later transmission. Returns 0 and counts a drop if they don't fit.
static int txq_push(txq_t *q, const uint8_t *d, int n) {
  uint16_t head = q->head;
  uint16_t used = (uint16_t)(head - q->tail) & TXQ_MASK;

  if (n < 0 || (uint16_t)n > (TXQ_MASK - used)) {
    q->dropped++;
    return 0;
  }

  for (int i = 0; i < n; i++) q->buf[(head + i) & TXQ_MASK] = d[i];
  q->head = (uint16_t)(head + n) & TXQ_MASK;
  return 1;
}

// Starts the next transmit chunk if the UART is idle and data is queued. Main loop only.
static void txq_kick(txq_t *q) {
  if (q->busy) return;
  uint16_t head = q->head;
  uint16_t tail = q->tail;
  if (head == tail) return;
  uint16_t len = (head > tail) ? (uint16_t)(head - tail) : (uint16_t)(TXQ_SIZE - tail);

  q->busy = 1;
  q->inflight = len;
  if (HAL_UART_Transmit_IT(q->huart, &q->buf[tail], len) != HAL_OK) q->busy = 0;
}

// Sends encoded [type][payload] COBS frame
static void send_frame(uint8_t type, const uint8_t *payload, int payload_len) {
  tud_cdc_n_write_flush(USB_CDC_ITF);
  uint8_t raw[CMD_BUFFER_SIZE];
  uint8_t encoded[cobs_estimate_encoded_size(CMD_BUFFER_SIZE)]; // Worst case COBS overhead
  if (payload_len < 0 || payload_len > (int)sizeof(raw) - 1) return;
  raw[0] = type;
  if (payload_len > 0) memcpy(raw + 1, payload, payload_len);
  int n = cobs_encode(raw, payload_len + 1, encoded, sizeof(encoded), 0);
  if (n <= 0) return;
  if (tud_cdc_n_write_available(USB_CDC_ITF) < (uint32_t)n) { usb_tx_dropped++; return; }
  send_msg_raw((char*)encoded, n);
}

// Dispatches one decoded [type][payload] USB frame.
static void process_usb_frame(uint8_t *frame, int len) {
  if (len < 1) return;
  uint8_t type = frame[0];
  uint8_t *payload = frame + 1;
  int payload_len = len - 1;

  switch (type) {
    case 'p':
      if (payload_len > 0 && payload_len < 100) {
        uint8_t line[100];
        memcpy(line, payload, payload_len);
        line[payload_len] = '\n';
        txq_push(&pantilt_txq, line, payload_len + 1);
      }
      break;
    case 't':
      txq_push(&term_txq, payload, payload_len);
      break;
  }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USB_PCD_Init();
  MX_UART4_Init();
  MX_USART3_UART_Init();
  MX_USART1_UART_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */
  setup_simple();
  cobs_setup_stream_reader(&usb_cobs_reader);
  pantilt_txq.huart = pantilt_uart;
  term_txq.huart = term_uart;

  gps_init(&gps, GPS_TYPE, &huart3, true);
  HAL_UARTEx_ReceiveToIdle_IT(gps.huart, gps_buffers[gps_index], GPS_BUFFER_SIZE);

  HAL_UARTEx_ReceiveToIdle_IT(pantilt_uart, pantilt_buffers[pantilt_index], PANTILT_BUFFER_SIZE);
  HAL_UARTEx_ReceiveToIdle_IT(term_uart, term_buffers[term_index], TERM_BUFFER_SIZE);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (!tud_cdc_ready()){
    process_simple(); // Keep USB connection running until ready
#ifdef WATCHDOG_ENABLE
    HAL_IWDG_Refresh(&hiwdg);
#endif
  }

  char* startup_message = "GPS Ready";
  send_frame('s', (uint8_t*) startup_message, strlen(startup_message));

  while (1) {
    if (gps_ready[0] || gps_ready[1]) {
      int gps_filled_index = (gps_index == 0) ? 1 : 0;
      if (gps_ready[gps_filled_index] != 0){
        uint8_t *buf = gps_buffers[gps_filled_index];
        for (int i = 0; i < buffer_sizes[gps_filled_index]; i++) gps_process(&gps, buf[i]);
        gps_ready[gps_filled_index] = 0;
      }
    }

    gps_data_t data;
    bool got_fix = gps_read_snapshot(&gps, &data);
    if (got_fix) {
      HAL_GPIO_TogglePin(USER_LED_GPIO_Port, USER_LED_Pin); // Blink LED for GPS processing

      int_to_string(data.numSV, satellites, 50);
      float_to_string(data.lat, 8, latitude, 50);
      float_to_string(data.lon, 8, longitude, 50);
      float_to_string(data.headMot, 8, heading, 50);

      char gps_payload[200];
      int gps_payload_len = snprintf(gps_payload, sizeof(gps_payload), "%s,%s,%s,%s", satellites, latitude, longitude, heading);
      send_frame('g', (uint8_t*)gps_payload, gps_payload_len);
    }

    // Diagnostic data to check whether GPS is working as expected
    static uint32_t last_status_tick = 0;
    uint32_t now = HAL_GetTick();
    if (now - last_status_tick >= DIAG_REPORT_PERIOD_MS) {
      last_status_tick = now;
      char status_payload[64];
      int status_len = snprintf(status_payload, sizeof(status_payload), "%lu,%lu,%lu,%lu,%lu,%lu",
        (unsigned long)gps_get_valid_frames(&gps), (unsigned long)gps_get_error_frames(&gps),
        (unsigned long)pantilt_txq.dropped, (unsigned long)term_txq.dropped,
        (unsigned long)usb_tx_dropped, (unsigned long)uart_errors);
      send_frame('d', (uint8_t*)status_payload, status_len);
    }

    // Send pantilt angle reporting to Jetson
    if (pantilt_ready == 1) {
      int pantilt_filled_index = (pantilt_index == 0) ? 1 : 0;
      char *parsed = (char*)pantilt_buffers[pantilt_filled_index];
      int comma_count = 0;
      int newline_pos = -1;

      for (int i = 0; i < PANTILT_BUFFER_SIZE - 1; i++) {
        if (parsed[i] == ',') comma_count++;
        if (parsed[i] == '\n') {
          newline_pos = i;
          break;
        }
      }

      if (comma_count == 1 && newline_pos >= 0) {
        send_frame('p', (uint8_t*)parsed, newline_pos);
      } else {
        send_frame('s', (uint8_t*)parsed, newline_pos);
      }

      pantilt_ready = 0;
    }

    // Send terminal output to Jetson
    if (term_ready) {
      int term_filled_index = (term_index == 0) ? 1 : 0;
      send_frame('t', term_buffers[term_filled_index], term_size);
      term_ready = 0;
    }

    if (usb_chunk_len < sizeof(usb_chunk)) {
      usb_chunk_len += tud_cdc_n_read(USB_CDC_ITF, usb_chunk + usb_chunk_len, sizeof(usb_chunk) - usb_chunk_len);
    }

    if (usb_chunk_len > 0) {
      cobs_result_t r = cobs_stream_decode(&usb_cobs_reader, usb_chunk, usb_chunk_len,
                                            cmd_buf + cmd_len, CMD_BUFFER_SIZE - cmd_len, 0);
      cmd_len += usb_cobs_reader.last_written_bytes;

      if (r == COBS_INCOMPLETE_FRAME) {
        usb_chunk_len = 0;
      } else {
        uint32_t consumed = usb_cobs_reader.last_read_bytes;

        if (r == COBS_DONE) {
          if (!cmd_overflow) process_usb_frame(cmd_buf, cmd_len);
          cmd_len = 0;
          cmd_overflow = false;
        } else if (r == COBS_OUTPUT_FULL) {
          cmd_len = 0;
          cmd_overflow = true;
        } else if (r == COBS_RESET) {
          cmd_len = 0;
          cmd_overflow = false;
        }

        uint32_t leftover = usb_chunk_len - consumed;
        if (leftover > 0 && consumed > 0) memmove(usb_chunk, usb_chunk + consumed, leftover);
        usb_chunk_len = leftover;
      }
    }

    txq_kick(&pantilt_txq);
    txq_kick(&term_txq);

    process_simple();

#ifdef WATCHDOG_ENABLE
    HAL_IWDG_Refresh(&hiwdg);
#endif
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 12;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV4;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */
#ifndef WATCHDOG_ENABLE
  return;
#endif
  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_32;
  hiwdg.Init.Window = 999;
  hiwdg.Init.Reload = 999;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart4, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart4, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_PCD_Init(void)
{

  /* USER CODE BEGIN USB_Init 0 */

  /* USER CODE END USB_Init 0 */

  /* USER CODE BEGIN USB_Init 1 */

  /* USER CODE END USB_Init 1 */
  hpcd_USB_FS.Instance = USB;
  hpcd_USB_FS.Init.dev_endpoints = 8;
  hpcd_USB_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_FS.Init.Sof_enable = DISABLE;
  hpcd_USB_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_FS.Init.battery_charging_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_Init 2 */

  /* USER CODE END USB_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USER_LED_GPIO_Port, USER_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_LED_Pin */
  GPIO_InitStruct.Pin = USER_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USER_LED_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

int __io_putchar(int ch)
{
 ITM_SendChar(ch);
 return(ch);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart){
  uart_errors++;
  
  if (huart == gps.huart) {
    HAL_UARTEx_ReceiveToIdle_IT(gps.huart, gps_buffers[gps_index], GPS_BUFFER_SIZE);
  }
  if (huart == pantilt_uart) {
    pantilt_txq.busy = 0;
    HAL_UARTEx_ReceiveToIdle_IT(pantilt_uart, pantilt_buffers[pantilt_index], PANTILT_BUFFER_SIZE);
  }
  if (huart == term_uart) {
    term_ready = 0;
    term_txq.busy = 0;
    HAL_UARTEx_ReceiveToIdle_IT(term_uart, term_buffers[term_index], TERM_BUFFER_SIZE);
  }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
  if (huart == pantilt_uart) {
    pantilt_index = (pantilt_index == 0) ? 1 : 0;
    pantilt_ready = 1;
    HAL_UARTEx_ReceiveToIdle_IT(pantilt_uart, pantilt_buffers[pantilt_index], PANTILT_BUFFER_SIZE);
  } else if (huart == term_uart) {
    term_size = Size;
    term_index = (term_index == 0) ? 1 : 0;
    term_ready = 1;
    HAL_UARTEx_ReceiveToIdle_IT(term_uart, term_buffers[term_index], TERM_BUFFER_SIZE);
  } else if (huart == gps.huart){
    buffer_sizes[gps_index] = Size;
    gps_ready[gps_index] = 1;
    gps_index = (gps_index == 0) ? 1 : 0;
    HAL_UARTEx_ReceiveToIdle_IT(gps.huart, gps_buffers[gps_index], GPS_BUFFER_SIZE);
  }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
  txq_t *q;
  if (huart == pantilt_uart) q = &pantilt_txq;
  else if (huart == term_uart) q = &term_txq;
  else return;

  q->tail = (uint16_t)(q->tail + q->inflight) & TXQ_MASK;
  q->busy = 0;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1)
  {
    // Blink LED fast if something went wrong
    HAL_GPIO_TogglePin(USER_LED_GPIO_Port, USER_LED_Pin);
    for (volatile int i = 0; i < 500000; i++);
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
