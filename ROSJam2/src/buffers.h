#ifndef ROSJAM_BUFFER_H
#define ROSJAM_BUFFER_H

#include "default_rosjam_config.h"
#include "stdint.h"

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
uint32_t get_size_with_pad(uint32_t size);
int get_first_message(Buffer* buf, int* str_size, uint8_t** start);
void mark_read(Buffer* buf, int read);
void mark_read_global(RosjamRxBuffer* buf, int read);
uint8_t* get_write_space_global(RosjamRxBuffer* buf, int size);

#ifdef __cplusplus
}
#endif

#endif