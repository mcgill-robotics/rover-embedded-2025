#ifndef ROSJAM_BUFFER_H
#define ROSJAM_BUFFER_H

#include "default_rosjam_config.h"
#include "stdint.h"

#ifdef __cplusplus
extern "C" {
#endif


typedef struct Buffer {
	uint8_t* buf;
	int capacity;
	int size;
	int read_offset;
} Buffer;


uint8_t* get_tagged_write_space(Buffer* buf, uint32_t size);
uint32_t get_size_with_pad(uint32_t size);
int get_first_message(Buffer* buf, int* str_size, uint8_t** start);
void mark_read(Buffer* buf, int read);
uint8_t* get_write_space(Buffer* buf, int size);

#ifdef __cplusplus
}
#endif

#endif