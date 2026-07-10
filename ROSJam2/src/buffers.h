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

typedef enum buffer_alloc_state_t {
	BUFFER_OK,
	BUFFER_CLEARED,
	BUFFER_MOVED,
	BUFFER_TOO_SMALL
} buffer_alloc_state_t;

buffer_alloc_state_t get_tagged_write_space(Buffer* buf, uint32_t size, uint8_t** write_head);
uint32_t get_size_with_pad(uint32_t size);
void reclaim_allocated(Buffer* buf, int size);
int get_first_message(Buffer* buf, int* str_size, uint8_t** start);
void mark_read(Buffer* buf, int read);
buffer_alloc_state_t get_write_space(Buffer* buf, int size, uint8_t** write_head);

#ifdef __cplusplus
}
#endif

#endif