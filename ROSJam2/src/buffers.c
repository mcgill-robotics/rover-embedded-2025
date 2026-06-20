#include "buffers.h"

#include "stddef.h"

uint32_t get_size_with_pad(uint32_t size){
	// ensure alignment
	return size+(4-(size%4))%4;
	
}

int get_first_message(Buffer* buf, int* str_size, uint8_t** start){
	if (buf->read_offset < buf->size){
		int offset = buf ->read_offset;
		uint8_t* message_start = buf->buf+offset;
		*str_size = ((uint32_t*) message_start)[0];
		*start = message_start+4;
		int size_with_padding = get_size_with_pad(*str_size);
		return size_with_padding+4;
	}
	return 0;
	
}

void mark_read(Buffer* buf, int read){
	(buf ->read_offset)+=read;
}

void mark_read_global(RosjamRxBuffer* buf, int read){
	(buf ->read_offset)+=read;
}

uint8_t* get_write_space(Buffer* buf, uint32_t size){
	uint32_t metadata_size = 4; //+4 for metadata about string size
	uint32_t size_with_padding = get_size_with_pad(size);
	uint32_t to_reserve = size_with_padding+metadata_size;
	
	if (size > buf->capacity){
		return NULL;
	}
	int available = buf->capacity - buf->size;
	uint8_t* write_position;
	if (available < to_reserve){
		// drop everything currently in buffer
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


uint8_t* get_write_space_global(RosjamRxBuffer* buf, int size){
	if (size > buf->capacity){
		return NULL;
	}
	int available = buf->capacity - buf->size;
	if (available < size){
		// drop everything currently in buffer
		buf-> read_offset = 0;
		buf -> size = size;
		return buf -> buf;
	} else {
		uint8_t* write_position = buf -> buf+buf -> size;
		buf -> size += size;
		return write_position;
	}
}