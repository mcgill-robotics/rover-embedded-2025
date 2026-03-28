#ifndef ROSJAM_H
#define ROSJAM_H

#include "stm32g4xx_hal.h"

#ifndef USB_CDC_ITF
#define USB_CDC_ITF 0
#endif

#ifndef USB_CDC_FIFO_SIZE
#define USB_CDC_FIFO_SIZE 4096
#endif

#ifndef USB_CDC_EP_BUF_SIZE
#define USB_CDC_EP_BUF_SIZE 2048
#endif

#ifndef ENDPOINT_COUNT 
#define ENDPOINT_COUNT 7
#endif

#ifndef ENDPOINT_BUF_LEN
#define ENDPOINT_BUF_LEN 2048
#endif

#ifdef __cplusplus
extern "C" {
#endif

typedef struct Buffer {
	int capacity;
	int size;
	int read_offset;
	uint8_t buf[ENDPOINT_BUF_LEN];
} Buffer;

typedef struct RosjamRxBuffer {
	int capacity;
	int size;
	int read_offset;
	uint8_t buf[ENDPOINT_BUF_LEN*ENDPOINT_COUNT];
} RosjamRxBuffer;

uint8_t* get_write_space(Buffer* buf, uint32_t size);


typedef struct RosjamEndpoint {
	const char* topic;
	Buffer tx_buf;
	Buffer rx_buf;
} RosjamEndpoint;

typedef struct ActiveEndpoints {
	int size;
	int nextTxEndpoint;
	RosjamEndpoint* endpoints[ENDPOINT_COUNT];
	RosjamRxBuffer global_rx_buffer;
} ActiveEndpoints;

int add_interface(RosjamEndpoint* endpoint, const char* topic);

void send_msg(RosjamEndpoint* endpoint, char* message);

/**

	Message is a pointer to the start of the message
	The message is stored in the rx buffer of endpoint

*/
void receivedFromUSB(RosjamEndpoint* endpoint, char* message);

void setup();

void process();

#ifdef __cplusplus
}
#endif

#endif