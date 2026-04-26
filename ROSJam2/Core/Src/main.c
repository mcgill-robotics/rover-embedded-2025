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
#include "device/usbd.h"
#include "rosjam.h"
#include "json_serde/serialization.h"
#include "stdio.h"
#include "cobs.h"
#include "stm32g4xx_hal_def.h"
#include <stdint.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
PCD_HandleTypeDef hpcd_USB_FS;

/* USER CODE BEGIN PV */
uint8_t rx_buff[1000];
int led_state = 1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USB_PCD_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
RosjamEndpoint endpoint0;
RosjamEndpoint endpoint1;
RosjamEndpoint endpoint2;
RosjamEndpoint endpoint3;
RosjamEndpoint endpoint4;
RosjamEndpoint endpoint5;
RosjamEndpoint endpoint6;


uint8_t uart0buf[ENDPOINT_BUF_LEN];
uint8_t uart1buf[ENDPOINT_BUF_LEN];
uint8_t uart2buf[ENDPOINT_BUF_LEN];
uint8_t uart3buf[ENDPOINT_BUF_LEN];
uint8_t uart4buf[ENDPOINT_BUF_LEN];
uint8_t uart5buf[ENDPOINT_BUF_LEN];
uint8_t diag0buf[ENDPOINT_BUF_LEN];
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
  /* USER CODE BEGIN 2 */
  // setup_simple();
  
  init_interface(&endpoint0, "diag0", diag0buf, ENDPOINT_BUF_LEN);
  init_interface(&endpoint1, "uart0", uart0buf, ENDPOINT_BUF_LEN);
  init_interface(&endpoint2, "uart1", uart1buf, ENDPOINT_BUF_LEN);
  init_interface(&endpoint3, "uart2", uart2buf, ENDPOINT_BUF_LEN);
  init_interface(&endpoint4, "uart3", uart3buf, ENDPOINT_BUF_LEN);
  init_interface(&endpoint5, "uart4", uart4buf, ENDPOINT_BUF_LEN);
  init_interface(&endpoint6, "uart5", uart5buf, ENDPOINT_BUF_LEN);
  // must call init before registering interfaces
  init_rosjam_usb();
  register_interface(&endpoint0);
  register_interface(&endpoint1);
  register_interface(&endpoint2);
  register_interface(&endpoint3);
  register_interface(&endpoint4);
  register_interface(&endpoint5);
  register_interface(&endpoint6);
  
  set_diag_endpoint(&endpoint0);
  
  // HAL_UART_Receive_IT(&huart3, rx_buff, 1000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  int counter = 0;
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // if (tud_cdc_n_ready(0)){
    //   char* str = "Hello World! diag: This is a longer message\n";
    //   int written = encode( (uint8_t*)str, strlen(str), (uint8_t*)data, 1000, 0);
    //   tud_cdc_n_write(0, data, written);
    //   tud_cdc_n_write_flush(0);
      // serialize(&test ,"test", "Hello World! diag: This is a longer message\n");
      // serialize_simple("test", "Hello World! diag: This is a longer message\n");
    // }
    
    if (led_state==1){
        HAL_GPIO_WritePin(USER_LED_GPIO_Port, USER_LED_Pin, 1);
        // HAL_GPIO_TogglePin (USER_LED_GPIO_Port, USER_LED_Pin);
    } else {
        HAL_GPIO_WritePin(USER_LED_GPIO_Port, USER_LED_Pin, 0);
    }
    // if(HAL_UART_Receive(&huart4, rx_buff, 1000, 1000)==HAL_OK) //if transfer is successful
    // { 
    //   printf("Received: %s\n", rx_buff);
    //   // char* hello = "Hello World!\n";
    //   // HAL_UART_Transmit(&hlpuart1, rx_buff, 1000, 1000);
      
    // } else {
    //   __NOP();
    // }
    //look into this for converting float to string https://stackoverflow.com/questions/41394063/how-to-simply-convert-a-float-to-a-string-in-order-to-write-it-to-a-text-file-in
    // int sat = 5;
    // double latitude = 12.1234345345345345345;
    // double longitude = 1.23456234245345345;
    // char convert_buf[100];
    // int_to_string(sat, convert_buf, 100);
    // print_to_usb(convert_buf);
    // print_to_usb(",");
    // float_to_string(latitude, 8, convert_buf, 100);
    // print_to_usb(convert_buf);
    // print_to_usb(",");
    // float_to_string(longitude, 8, convert_buf, 100);
    // print_to_usb(convert_buf);
    // print_to_usb("\n");

    // if (has_data()) {
    //   char incomingChar = read_char();
    //   if (incomingChar == ',') {
    //     commaIndex = index;
    //   }
    //   if (incomingChar == '\n') {
    //     if (commaIndex > 0 ) {
    //       buf[commaIndex] = '\0';
    //       char* panStr = buf;
    //       char* tiltStr = buf+commaIndex+1;
    //       buf[index] = '\0';
    //       double panFloat = string_to_float(panStr);
    //       double tiltFloat = string_to_float(tiltStr);
    //       char buf2[100];
    //       float_to_string(panFloat, 8, buf2, 100);
    //       print_to_usb(buf2);
    //       print_to_usb(" sep ");
    //       float_to_string(tiltFloat, 8, buf2, 100);
    //       print_to_usb(buf2);
    //       print_to_usb("\n");
    //     }
    //     index = 0;
    //     commaIndex = 0;
    //   } 
    //   else {
    //     buf[index] = incomingChar; // Append to buffer
    //     index++;
    //   }
    // }
    for (int i = 0; i<2; i++){
      RosjamEndpoint* endpoint = NULL;
      switch (counter%6) {
        case 0: endpoint = &endpoint1; break;
        case 1: endpoint = &endpoint2; break;
        case 2: endpoint = &endpoint3; break;
        case 3: endpoint = &endpoint4; break;
        case 4: endpoint = &endpoint5; break;
        case 5: endpoint = &endpoint6; break;
      }
      char base[100];
      char* str = "Hello jetson this is some longer data this is some longer data this is some longer data ";
      char* str2 = "from ";
      int strlen2 = strlen(str2);
      memcpy(base, str2, strlen2);
      char* topic = endpoint->topic;
      int topic_len = strlen(topic);
      memcpy(base+strlen2, topic, topic_len);
      *(base+strlen2+topic_len) = ' ';
      memcpy(base+strlen2+topic_len+1, str, strlen(str)+1);
      char convert_buf[100];
      int_to_string(counter, convert_buf, 100);
      strcat(base, convert_buf);
      send_msg(endpoint, base) ;
      
      counter+=1;
    }
    
    // for (int i =0; i<1 ;i++){
    //   send_msg(&endpoint1, "Hello World! uart0: This is a longer message  uart0: This is a longer message Hello World! Hello World! uart0: This is a longer message  uart0: This is a longer message Hello World!");
    //   send_msg(&endpoint2, "Hello World! uart1: This is a longer message  uart1: This is a longer message Hello World! Hello World! uart0: This is a longer message  uart0: This is a longer message Hello World!");
    //   send_msg(&endpoint3, "Hello World! uart2: This is a longer message  uart2: This is a longer message Hello World! Hello World! uart0: This is a longer message  uart0: This is a longer message Hello World!");
    //   send_msg(&endpoint4, "Hello World! uart3: This is a longer message  uart3: This is a longer message Hello World! Hello World! uart0: This is a longer message  uart0: This is a longer message Hello World!");
    //   send_msg(&endpoint5, "Hello World! uart4: This is a longer message  uart4: This is a longer message Hello World! Hello World! uart0: This is a longer message  uart0: This is a longer message Hello World!");
    //   send_msg(&endpoint6, "Hello World! uart5: This is a longer message  uart5: This is a longer message Hello World! Hello World! uart0: This is a longer message  uart0: This is a longer message Hello World!");
    // }
    process();
    // tud_task();
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
  RCC_OscInitStruct.PLL.PLLN = 16;
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USER_LED_GPIO_Port, USER_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA2 PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF12_LPUART1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : USER_LED_Pin */
  GPIO_InitStruct.Pin = USER_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USER_LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC4 PC5 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB11 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PC10 PC11 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_UART4;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
// void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
// {
//   printf("Received %s", rx_buff);
//   // send_msg(&endpoint0,  (char*) rx_buff);
//   HAL_UART_Receive_IT(&huart3, rx_buff, 1000); //You need to toggle a breakpoint on this line!
// }

int __io_putchar(int ch)
{
 // Write character to ITM ch.0
//  HAL_UART_Transmit(&hlpuart1, (uint8_t *)&ch, 1, 0xFFFF);
 ITM_SendChar(ch);
 return(ch);
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
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
