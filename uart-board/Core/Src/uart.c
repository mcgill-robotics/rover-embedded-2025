#include "uart.h"
#include "main.h"
#include "rosjam.h"
#include "stm32g4xx.h"
#include "stm32g4xx_hal_uart.h"
#include "stm32g4xx_hal_uart_ex.h"
#include <string.h>

uart uarts[6];

void init_uart(uart* uart, UART_HandleTypeDef* huart, uint8_t *topic) {
    uart->huart = huart;
    uart->topic = topic;
    uart->buf.capacity = RECON_BUF_LEN;
    uart->buf.size = 0;
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

uint8_t* get_buffer_write_pointer(uart_buf* buf, uint32_t size) {
	uint32_t metadata_size = 4; // 4 for metadata about string size
	uint32_t size_with_padding = size+(4-(size%4))%4;
	uint32_t to_reserve = size_with_padding+metadata_size;
	
	if (size > buf->capacity) {
		return NULL;
	}
	int available = buf->capacity - buf->size;
	uint8_t* write_position;
	if (available < to_reserve) {
		// Drop everything currently in buffer
		buf-> read_offset = 0;
		buf -> size = to_reserve;
		write_position = buf -> buf;
	} else {
		write_position = buf -> buf+buf -> size;
		buf -> size += to_reserve;
	}
	((uint32_t*) write_position)[0] = size;
	write_position = write_position+metadata_size;
	return write_position;
}

void process_uart() {
    for (int i = 0; i < 6; i++) {
        if (uarts[i].data_size) {
            uint8_t buffer = !uarts[i].active_rx;
            int start = 0;
            for (int end = 0; end < uarts[i].data_size; end++) {
                if (uarts[i].rx[buffer][end] == '\n') {
                    uint8_t* write_start = get_buffer_write_pointer(&uarts[i].buf, end - start);
                    memcpy(write_start, &uarts[i].rx[buffer][start], end - start);
                    start = end + 1;
                }
            }
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
