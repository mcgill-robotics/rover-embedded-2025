#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include "cobs.h"

#define MAX_CHUNK_SIZE 255

/**
 * Returns the worse case encoded size
 */
int cobs_estimate_encoded_size(int buf_size){
	return 1+(buf_size/(MAX_CHUNK_SIZE-1)+1)+buf_size;// 1 delimiter(back) + chunk overhead (rounded up) + data size
}

/**
 * Estimates decoded size from cobs frame
 */
int cobs_estimate_decoded_size(int buf_size){
	return buf_size-buf_size/MAX_CHUNK_SIZE-1;// 1 delimiter(back) + chunk overhead (rounded up) + data size
}


void cobs_setup_stream_reader(cobs_reader_t* reader){
	reader -> state = COBS_SYNC;
	reader -> chunk_bytes_left = 0;
	reader -> overhead_next = 1;
}

void cobs_consume_message(cobs_reader_t* reader){
	reader->current_frame_size=0;
}

cobs_result_t cobs_stream_decode_buf(cobs_reader_t* reader, uint8_t* buf, int available, uint8_t* output, int output_length, uint8_t delim, int* written_bytes, int* read_bytes){
	*read_bytes = 0;
	*written_bytes = 0;
	while (*read_bytes<available){
		switch (reader->state)
		{
		case COBS_SYNC: {
			if (*buf == delim) {
				reader->state = COBS_OVERHEAD;
			}
			buf++;
			(*read_bytes)++;
			break;
		}
		case COBS_OVERHEAD: {
			(*read_bytes)++;
			int value = *buf;
			buf++;
			if (value == delim){
				reader->state = COBS_OVERHEAD;
				return COBS_DONE;
			}  
			reader->chunk_bytes_left = value-1;
			if (value == MAX_CHUNK_SIZE){
				reader->overhead_next = 1;
			} else {
				reader ->overhead_next = 0;
			}
			reader->state = COBS_CHUNK;
			break;
		}
		case COBS_CHUNK: {
			if (*written_bytes>=output_length){
				break;
			}
			int to_write = reader->chunk_bytes_left; 
			if (to_write==0){
				reader->state = COBS_STUFFED_BYTE;
			} else {
				int output_space_left = output_length-*written_bytes;
				int input_available = available-*read_bytes;
				int state = 0;
				if (output_space_left >= to_write){
					if (reader->overhead_next){
						reader->state = COBS_OVERHEAD;
					} else {
						reader->state = COBS_STUFFED_BYTE;
					}
				} else {
					if (input_available < output_space_left){
						to_write = input_available;
						state = -1;
					} else {
						to_write = output_space_left;
						state = -2;
					}
					
				}
				memcmp(output, buf, to_write);
				buf+=to_write;
				(*read_bytes)+=to_write;
				(reader->chunk_bytes_left)-=to_write;
				output+=to_write;
				(*written_bytes)+=to_write;
				if (state == -1){
					(reader->current_frame_size)+=*written_bytes;
					return COBS_OUTPUT_FULL;
				} else if (state == -2){
					(reader->current_frame_size)+=*written_bytes;
					return COBS_INCOMPLETE_FRAME;
				}
			}
			break;
		}
		case COBS_STUFFED_BYTE: {
			if (*written_bytes>=output_length){
				break;
			}
			(*read_bytes)++;
			int value = *buf;
			buf++;
			if (value == delim){
				reader->state = COBS_OVERHEAD;
				(reader->current_frame_size)+=*written_bytes;
				return COBS_DONE;
			}  
			*written_bytes++;
			*output = delim;
			output++;
			reader->chunk_bytes_left = value-1;
			if (value == MAX_CHUNK_SIZE){
				reader->overhead_next = 1;
			} else {
				reader ->overhead_next = 0;
			}
			reader->state = COBS_CHUNK;
			break;
		}
		default:
			break;
		}
	}
	(reader->current_frame_size)+=*written_bytes;
	if (*read_bytes == available){
		if (reader->state == COBS_SYNC){
			return COBS_NO_FRAME;
		} else {
			return COBS_INCOMPLETE_FRAME;
		}
	}
	if (*written_bytes == output_length){
		return COBS_OUTPUT_FULL;
	}
}
/**
 * Support streaming through a pseudo read style api (actually based on tinyusb read)
 */
cobs_result_t cobs_stream_decode(cobs_reader_t* reader, uint32_t (*read)(uint8_t, void*, uint32_t), int itf, int available, uint8_t* output, int output_length, uint8_t delim, int* written_bytes, int* read_bytes){
	*read_bytes = 0;
	*written_bytes = 0;
	uint8_t temp[1];
	while (*read_bytes<available){
		switch (reader->state)
		{
		case COBS_SYNC: {
			read(itf, temp, 1);
			if (*temp == delim) {
				reader->state = COBS_OVERHEAD;
			}
			(*read_bytes)++;
			break;
		}
		case COBS_OVERHEAD: {
			read(itf, temp, 1);
			(*read_bytes)++;
			int value = *temp;
			if (value == delim){
				reader->state = COBS_OVERHEAD;
				(reader->current_frame_size)+=*written_bytes;
				return COBS_DONE;
			}  
			reader->chunk_bytes_left = value-1;
			if (value == MAX_CHUNK_SIZE){
				reader->overhead_next = 1;
			} else {
				reader ->overhead_next = 0;
			}
			reader->state = COBS_CHUNK;
			break;
		}
		case COBS_CHUNK: {
			if (*written_bytes>=output_length){
				break;
			}
			int to_write = reader->chunk_bytes_left; 
			if (to_write==0){
				reader->state = COBS_STUFFED_BYTE;
			} else {
				int output_space_left = output_length-*written_bytes;
				int input_available = available-*read_bytes;
				int state = 0;
				if (output_space_left >= to_write){
					if (reader->overhead_next){
						reader->state = COBS_OVERHEAD;
					} else {
						reader->state = COBS_STUFFED_BYTE;
					}
				} else {
					if (input_available < output_space_left){
						to_write = input_available;
						state = -1;
					} else {
						to_write = output_space_left;
						state = -2;
					}
					
				}
				read(itf, output, to_write);
				(*read_bytes)+=to_write;
				(reader->chunk_bytes_left)-=to_write;
				output+=to_write;
				(*written_bytes)+=to_write;
				if (state == -1){
					(reader->current_frame_size)+=*written_bytes;
					return COBS_OUTPUT_FULL;
				} else if (state == -2){
					(reader->current_frame_size)+=*written_bytes;
					return COBS_INCOMPLETE_FRAME;
				}
			}
			break;
		}
		case COBS_STUFFED_BYTE: {
			if (*written_bytes>=output_length){
				break;
			}
			read(itf, temp, 1);
			(*read_bytes)++;
			int value = *temp;
			if (value == delim){
				reader->state = COBS_OVERHEAD;
				(reader->current_frame_size)+=*written_bytes;
				return COBS_DONE;
			}  
			*written_bytes++;
			*output = delim;
			output++;
			reader->chunk_bytes_left = value-1;
			if (value == MAX_CHUNK_SIZE){
				reader->overhead_next = 1;
			} else {
				reader ->overhead_next = 0;
			}
			reader->state = COBS_CHUNK;
			break;
		}
		default:
			break;
		}
	}
	(reader->current_frame_size)+=*written_bytes;
	if (*read_bytes == available){
		if (reader->state == COBS_SYNC){
			return COBS_NO_FRAME;
		} else {
			
			return COBS_INCOMPLETE_FRAME;
		}
	}
	if (*written_bytes == output_length){
		return COBS_OUTPUT_FULL;
	}
}

/**
 * Returns the bytes written to the output buffer
 * -1 if not enough bytes are available in output_length
 * 
 */
int cobs_encode(uint8_t* input, int input_length, uint8_t* output, int output_length, uint8_t delim){
	uint8_t* output_initial = output;
	uint8_t* output_end = output+output_length;
	uint8_t* input_end = input+input_length;
	int chunk_size = 0;
	if (1>output_length){
		return -1;
	}
	// *output = delim;
	uint8_t* last_replaced = output;//+1;
	output+=1;//2; // reserve place for header and write initial delim
	for (;input<input_end;input++){
		uint8_t current_byte = *input;
		
		if (chunk_size+1 == MAX_CHUNK_SIZE){
			if (output+chunk_size+1 > output_end){
				return -1;
			}
			// printf("%c, %d, %p, %c\n", current_byte, chunk_size, input-chunk_size, *(input-chunk_size));
			memcpy(output, input-chunk_size, chunk_size);
			output+=chunk_size;
			*last_replaced = chunk_size+1;
			last_replaced = output;
			output++; //make space for next stuffed byte
			chunk_size = 0;
		}
		if (current_byte == delim) {
			if (output+chunk_size+1 > output_end){
				return -1;
			}
			// printf("%c, %d, %p, %c\n", current_byte, chunk_size, input-chunk_size, *(input-chunk_size));
			memcpy(output, input-chunk_size, chunk_size);
			output+=chunk_size;
			*last_replaced = chunk_size+1;
			last_replaced = output;
			output++; //make space for next stuffed byte
			chunk_size = 0;
		} else {
			chunk_size++;
		}
	}
	if (output+chunk_size+1 > output_end){
		return -1;
	}
	// printf("%c, %d, %p, %c\n", *input, chunk_size, input-chunk_size, *(input-chunk_size));
	memcpy(output, input-chunk_size, chunk_size);
	output+=chunk_size;
	*last_replaced = chunk_size+1;
	last_replaced = output;
	*output = delim;
	output++;

	return output-output_initial;
}

/**
 * Decodes one cobs encoded frame discarding any data before the first encountered delimiter
 * returns the number of read bytes
 * -1 if not enough bytes were available in output
 * -3 if message could not be found in input buffer (not enough bytes)
 * -2 not enough data and could not find first delim (all data to throw)
 */
int cobs_decode(uint8_t* input, int input_length, uint8_t* output, int output_length, uint8_t delim, int* written){
	uint8_t* output_initial = output;
	uint8_t* input_initial = input;
	uint8_t* output_end = output+output_length;
	uint8_t* input_end = input+input_length;
	int chunk_size = 0;
	int delim_count = 0;
	
	// printf("Start out: %p\n", output);
	while (input<input_end){
		
		uint8_t current_byte = *input;
		if (current_byte == delim){
			// printf("cnt: %d %d\n", count, input-input_initial);
			delim_count++;
			input++;
			chunk_size = MAX_CHUNK_SIZE-1;
			//ignore first delim
			if (delim_count == 1){
				continue;
			}
			
		}

		if (delim_count == 0){
			input++;
			continue;
		} else if (delim_count>1){
			// input--;
			break;
		}
		

		// printf("p %d, %d, %p, %c\n", current_byte, chunk_size, input, *(input));
		// printf("Chunk size: %d %d %p %p\n", chunk_size, delim_count, input, input_end);
		if (chunk_size == MAX_CHUNK_SIZE-1){
			chunk_size = (*input)-1;
			
			// printf("header\n");
		} else {
			if (output+1>output_end){
				return -1;
			}
			chunk_size = (*input)-1;
			*output = delim;
			// printf("stuff loc %p\n", output);
			output++;
			// printf("stuffed %p %d %d %d\n", output, *input, *(input-1), *(input+1));
		}
		if (output+chunk_size>output_end){
			return -1;
		}		
		
		input++;
		// printf("b %p, %d, %d, %p, %c\n", output, current_byte, chunk_size, input, *input);
		// printf("out: %s\n", output_initial);
		// printf("out: %s\n", output);
		if (chunk_size >= 0){
			memcpy(output, input, chunk_size);

			output+=chunk_size;
			input+=(chunk_size);
		}
		// printf("out: %s\n", output);
		
		// printf("next: %c\n", *(input-1));
		// printf("next: %c\n", *input);
		
		// printf("next: %c\n", *(input+1));
		// printf("a %d, %d, %p, %c\n", current_byte, chunk_size, input, *(input));
		// if (count == 100){
		// 	break;
		// }
	}
	if (delim_count == 0){
		return -2;
	} else if (delim_count == 1){
		return -3;
	}
	// printf("delim: %d \n", delim_count);
	*written = output-output_initial;
	return input-input_initial-1; // do not count second delim
}