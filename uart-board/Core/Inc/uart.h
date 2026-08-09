#ifndef __UART_H
#define __UART_H

#include "main.h"
#include "rosjam.h"
#include <stdint.h>

#define UART_BUF_LEN 32
#define RECON_BUF_LEN 512

#define LPUART1_TOPIC "uart0"
#define UART1_TOPIC "uart1"
#define UART2_TOPIC "uart2"
#define UART3_TOPIC "uart3"
#define UART4_TOPIC "uart4"
#define UART5_TOPIC "uart5"

typedef struct {
	int capacity;
	int size;
	int read_offset;
	uint8_t buf[RECON_BUF_LEN];
} uart_buffer;

typedef struct {
    UART_HandleTypeDef* huart;
    uint8_t* topic;
    uart_buffer buffer;
    uint8_t tx[RECON_BUF_LEN];
    uint8_t rx[2][UART_BUF_LEN];
    uint16_t rx_size;
    uint16_t tx_size;
    uint8_t active_rx;
    uint8_t active_tx;
} uart;

extern uart uarts[6];

void init_uart(uart* uart, UART_HandleTypeDef* huart, uint8_t* topic);
uart* get_uart(UART_HandleTypeDef *huart);
uint8_t* get_buffer_write_pointer(uart_buffer* buffer, uint32_t size);
void process_uart();
void transmit_uart(uart* uart);

#endif /* __UART_H */