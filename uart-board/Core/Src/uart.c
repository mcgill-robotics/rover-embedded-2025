#include "uart.h"
#include "main.h"
#include "rosjam.h"
#include "stm32g4xx.h"
#include "stm32g4xx_hal_uart.h"
#include "stm32g4xx_hal_uart_ex.h"
#include <string.h>
#include <stdio.h>

uart uarts[6];
uint8_t test_tx[RECON_BUF_LEN];

void init_uart(uart* uart, UART_HandleTypeDef* huart, uint8_t *topic) {
    uart->huart = huart;
    uart->topic = topic;
    uart->buffer.capacity = RECON_BUF_LEN;
    uart->buffer.size = 0;
    uart->active_rx = 0;
    uart->active_tx = 0;
    uart->rx_size = 0;
    uart->tx_size = 0;
    HAL_UARTEx_ReceiveToIdle_DMA(huart, uart->rx[uart->active_rx], UART_BUF_LEN);
}

uart* get_uart(UART_HandleTypeDef *huart) {
    for (int i = 0; i < 6; i++) {
        if (uarts[i].huart == huart) return &uarts[i];
    }
    return NULL;
}

uint8_t* get_buffer_write_pointer(uart_buffer* buffer, uint32_t size) {
	uint32_t metadata_size = 4; // 4 for metadata about string size
	uint32_t size_with_padding = size+(4-(size%4))%4;
	uint32_t to_reserve = size_with_padding+metadata_size;
	
	if (size > buffer->capacity) {
		return NULL;
	}
	int available = buffer->capacity - buffer->size;
	uint8_t* write_position;
	if (available < to_reserve) {
		// Drop everything currently in buffer
		buffer-> read_offset = 0;
		buffer -> size = to_reserve;
		write_position = buffer->buf;
	} else {
		write_position = buffer->buf + buffer->size;
		buffer -> size += to_reserve;
	}
	((uint32_t*) write_position)[0] = size;
	write_position = write_position+metadata_size;
	return write_position;
}

void process_uart() {
    for (int i = 0; i < 6; i++) {
        if (uarts[i].huart != NULL && uarts[i].rx_size) {
            uint8_t buffer = !uarts[i].active_rx;
            int start = 0;
            // printf("%s\n", (char*) (uarts[i].rx[buffer]));
            for (int end = 0; end < uarts[i].rx_size; end++) {
                if (uarts[i].rx[buffer][end] == '\n') {
                    uint8_t* write_start = get_buffer_write_pointer(&uarts[i].buffer, end - start);
                    memcpy(write_start, &uarts[i].rx[buffer][start], end - start);
                    // Transmit test
                    memcpy(&test_tx[uarts[i].tx_size], &uarts[i].rx[buffer][start], end - start);
                    uarts[i].tx_size += end - start;

                    start = end + 1;
                }
            }
            uarts[i].rx_size = 0;
        }

        // Transmit test
        if (uarts[i].huart != NULL && uarts[i].tx_size && !uarts[i].active_tx) {
            memcpy(&uarts[i].tx, &test_tx, uarts[i].tx_size);
            transmit_uart(&uarts[i]);
            uarts[i].tx_size = 0;
        }
    }
}

void transmit_uart(uart* uart) {
    uart->active_tx = 1;
    HAL_UART_Transmit_DMA(uart->huart, uart->tx, uart->tx_size);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    uart* uart = get_uart(huart);
    // Ignore half complete DMA RX events
    if (huart->RxEventType == HAL_UART_RXEVENT_HT) return;
    if (uart == NULL) return;
    uart->rx_size = Size;
    uart->active_rx = !uart->active_rx;
    HAL_UARTEx_ReceiveToIdle_DMA(huart, uart->rx[uart->active_rx], UART_BUF_LEN);
}

void HAL_UART_TxHalfCpltCallback(UART_HandleTypeDef *huart) {
    return;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    uart* uart = get_uart(huart);
    uart->active_tx = 0;
}
