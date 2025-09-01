#include "uart_debugging.h"
#include <string.h>
#include "main.h"
#define DEBUG_PRINT

// IN ORDER TO DEBUG THINGS, UNCOMMENT THE DEFINE OF MX_USART2_UART_INIT IN MAIN.H, NAD MANUALLY REMOVE STATIC TO THE ONES IN MAIN.C --> UNDO THIS FOR NON-DEBUG


#define UART_TX_BUFFER_SIZE 1024

extern UART_HandleTypeDef huart2;
uint8_t uartTxBuffer[UART_TX_BUFFER_SIZE];
volatile bool uartTxDone = true;
extern void MX_USART2_UART_Init(void);

void uart_debug_print(const char *format, ...) {
#ifdef DEBUG_PRINT
    char buffer[UART_TX_BUFFER_SIZE];

    va_list args;
    va_start(args, format);
    int len = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    if (len > 0 && len < sizeof(buffer)) {
        uint32_t startTick = HAL_GetTick();
        HAL_StatusTypeDef status;

        do {
            status = HAL_UART_Transmit(&huart2, (uint8_t*)buffer, len, 100);  // max 100ms block
        } while (status == HAL_BUSY && (HAL_GetTick() - startTick < 200));

        if (status != HAL_OK) {
            // UART likely locked up → try resetting it
            HAL_UART_DeInit(&huart2);
            MX_USART2_UART_Init();

            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET); // LED ON = error
        } else {
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6); // blink on success
        }
    }
#endif
}




void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        uartTxDone = true;
        HAL_Delay(500);
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6); // DEBUG: LED blinks if TX completes
        HAL_Delay(500);
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6); // DEBUG: LED blinks if TX completes
        HAL_Delay(500);
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6); // DEBUG: LED blinks if TX completes
        HAL_Delay(500);
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6); // DEBUG: LED blinks if TX completes
		  HAL_Delay(500);
		  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6); // DEBUG: LED blinks if TX completes
		  HAL_Delay(500);
		  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6); // DEBUG: LED blinks if TX completes
		  HAL_Delay(500);
		  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6); // DEBUG: LED blinks if TX completes
    }
}


