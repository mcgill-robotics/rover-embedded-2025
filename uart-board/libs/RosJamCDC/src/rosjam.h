#ifndef ROSJAM_H
#define ROSJAM_H

#include <cstdint>
typedef struct rosjam_endpoint {
	char topic[20];
	int id;
	uint8_t rx_buf[2048];
	uint8_t tx_buf[2048];
} rosjam_endpoint;

void enable_interface(rosjam_endpoint* endpoint, UART_HandleTypeDef huart);

void sendToUSB(rosjam_endpoint* endpoint, char* message);

void receivedFromUSB(char* id, char* message);

void process();

#endif