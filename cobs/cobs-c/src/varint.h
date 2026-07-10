#ifndef VARINT_H
#define VARINT_H
#include "stdint.h"

#define CONTINUE_MASK 0x80
#define DATA_MASK 0x7F

typedef enum varint_decode_state_t {
	VARINT_OK,
	VARINT_TOO_BIG
} varint_decode_state_t;

int encode_varint(uint32_t num, uint8_t* buf);
varint_decode_state_t decode_varint(uint8_t* buf, uint32_t* result);
#endif