#include "uart_debugging.h"

#define UART_TX_BUFFER_SIZE 256

extern UART_HandleTypeDef huart2;
uint8_t uartTxBuffer[UART_TX_BUFFER_SIZE];
volatile bool uartTxDone = true;

void uart_debug_print(const char *format, ...) {
    if (!uartTxDone) return;  // Wait until last TX finishes

    uartTxDone = false;
    memset(uartTxBuffer, 0, UART_TX_BUFFER_SIZE);

    va_list args;
    va_start(args, format);
    int len = vsnprintf((char *)uartTxBuffer, UART_TX_BUFFER_SIZE, format, args);
    va_end(args);

    if (len > 0 && len < UART_TX_BUFFER_SIZE) {
        HAL_UART_Transmit_DMA(&huart2, uartTxBuffer, len);
    } else {
        uartTxDone = true;  // Fail-safe reset
    }
}



void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        uartTxDone = true;
        HAL_Delau(500)
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6); // DEBUG: LED blinks if TX completes
        HAL_Delau(500)
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6); // DEBUG: LED blinks if TX completes
        HAL_Delau(500)
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6); // DEBUG: LED blinks if TX completes
        HAL_Delau(500)
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6); // DEBUG: LED blinks if TX completes
		  HAL_Delau(500)
		  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6); // DEBUG: LED blinks if TX completes
		  HAL_Delau(500)
		  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6); // DEBUG: LED blinks if TX completes
		  HAL_Delau(500)
		  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6); // DEBUG: LED blinks if TX completes
    }
}


