#include "varint.h"
#include "stdint.h"

int encode_varint(uint32_t num, uint8_t* buf){
	int used_bytes = 0;
	while (1){
		if ((num & ~DATA_MASK) == 0){
			buf[used_bytes++] = num & DATA_MASK;
			break;
		}
		buf[used_bytes++] = ((num & DATA_MASK) | CONTINUE_MASK); 
		num >>= 7;
	}
	return used_bytes;
}

varint_decode_state_t decode_varint(uint8_t* buf, uint32_t* result){
	uint32_t output = 0;
	int pos = 0;
	while (1){
		uint8_t current_byte = *buf;
		output |= (current_byte&DATA_MASK) << pos;
		if ((current_byte & CONTINUE_MASK) == 0){
			break;
		}
		pos+=7;
		buf++;
		if (pos >= 32){
			return VARINT_TOO_BIG;
		}
	}
	*result = output;
	return VARINT_OK;
}