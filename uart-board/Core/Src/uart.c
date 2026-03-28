#include "uart.h"
#include "main.h"
#include "stm32g4xx.h"
#include "stm32g4xx_hal_uart.h"
#include "stm32g4xx_hal_uart_ex.h"
#include <string.h>

uart uarts[6];

void init_uart(uart* uart, UART_HandleTypeDef* huart, uint8_t *topic) {
    uart->huart = huart;
    uart->topic = topic;
    uart->active_rx = 0;
    uart->data_size = 0;
    HAL_UARTEx_ReceiveToIdle_DMA(huart, uart->rx[uart->active_rx], UART_BUF_LEN);
}

uart* get_uart(UART_HandleTypeDef *huart) {
    for (int i = 0; i < 6; i++) {
        if (uarts[i].huart == huart) return &uarts[i];
    }
    return NULL;
}

void process_uart() {
    for (int i = 0; i < 6; i++) {
        if (uarts[i].data_size) {
            uint8_t buffer = !uarts[i].active_rx;
            for (int j = 0; j < uarts[i].data_size; j++) {
                if (uarts[i].rx[buffer][j] == '\n') {
                    uarts[i].rx[buffer][j] = '\0';
                }
            }
            memcpy(uarts[i].process, uarts[i].rx[buffer], uarts[i].data_size);
            uarts[i].data_size = 0;
        }
    }
}

void transmit_uart(uart* uart, uint16_t size) {
    HAL_UART_Transmit_DMA(uart->huart, uart->tx, size);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    uart* uart = get_uart(huart);
    if (uart == NULL) return;
    uart->data_size = Size;
    uart->active_rx = !uart->active_rx;
    HAL_UARTEx_ReceiveToIdle_DMA(huart, uart->rx[uart->active_rx], UART_BUF_LEN);
}

void HAL_UART_TxHalfCpltCallback(UART_HandleTypeDef *huart) {
    return;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    return;
}
