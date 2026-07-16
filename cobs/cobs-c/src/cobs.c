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
	reader -> state = COBS_OVERHEAD;
	reader -> chunk_bytes_left = 0;
	reader -> overhead_next = 1;
	reader -> last_read_bytes  = 0;
	reader -> last_written_bytes = 0;
}

void cobs_consume_message(cobs_reader_t* reader){
	reader->current_frame_size=0;
}

cobs_result_t cobs_stream_decode(cobs_reader_t* reader, uint8_t* buf, int available, uint8_t* output, int output_length, uint8_t delim){
	uint32_t read_bytes = 0;
	uint32_t written_bytes = 0;
	while (read_bytes<available){
		switch (reader->state)
		{
		case COBS_OVERHEAD: {
			read_bytes++;
			int value = *buf;
			buf++;
			if (value == delim){
				reader->state = COBS_OVERHEAD;
				reader -> last_read_bytes = read_bytes;
				reader -> last_written_bytes = written_bytes;
				return COBS_DONE;
			}  
			reader->chunk_bytes_left = value-1;
			//Ser werher next non chunk byte is overhead or stuffed
			if (value == MAX_CHUNK_SIZE){
				reader -> overhead_next = 1;
			} else {
				reader -> overhead_next = 0;
			}
			reader->state = COBS_CHUNK;
			break;
		}
		case COBS_CHUNK: {
			int to_write = reader->chunk_bytes_left; 
			if (to_write==0){
				reader->state = COBS_STUFFED_BYTE;
			} else {
				int output_space_left = output_length-written_bytes;
				int input_available = available-read_bytes;
				while (to_write>0){
					if (output_space_left<=0){
						reader -> last_read_bytes = read_bytes;
						reader -> last_written_bytes = written_bytes;
						return COBS_OUTPUT_FULL;
					}
					if (input_available>0){
						// detect desync
						uint8_t value = *buf;
						*output = value;
						//move buffer pointers
						buf++;
						output++;
						// update r/w state
						read_bytes++;
						written_bytes++;
						// update decoder state
						(reader->chunk_bytes_left)--;
						(reader->current_frame_size)++;
						// update local variables
						output_space_left--;
						input_available--;
						to_write--;
						if (value==0){
							reader->state = COBS_OVERHEAD;
							reader -> last_read_bytes = read_bytes;
							reader -> last_written_bytes = written_bytes;
							return COBS_RESET;
						}
					}else {
						reader -> last_read_bytes = read_bytes;
						reader -> last_written_bytes = written_bytes;
						return COBS_INCOMPLETE_FRAME;
					}
				}
				if (reader->overhead_next){
					reader->state = COBS_OVERHEAD;
				} else {
					reader->state = COBS_STUFFED_BYTE;
				}
			}
			break;
		}
		case COBS_STUFFED_BYTE: {
			if (written_bytes>=output_length){
				break;
			}
			read_bytes++;
			int value = *buf;
			buf++;
			if (value == delim){
				reader->state = COBS_OVERHEAD;
				(reader->current_frame_size)+=written_bytes;
				reader -> last_read_bytes = read_bytes;
				reader -> last_written_bytes = written_bytes;
				return COBS_DONE;
			}  
			written_bytes++;
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
	(reader->current_frame_size)+=written_bytes;
	if (written_bytes == output_length){
		reader -> last_read_bytes = read_bytes;
		reader -> last_written_bytes = written_bytes;
		return COBS_OUTPUT_FULL;
	}
	reader -> last_read_bytes = read_bytes;
	reader -> last_written_bytes = written_bytes;
	return COBS_INCOMPLETE_FRAME;
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

int cobs_decode(uint8_t* input, int input_length, uint8_t* output, int output_length, uint8_t delim, int* written, int* read){
	cobs_reader_t reader;
	cobs_setup_stream_reader(&reader);
	cobs_result_t result = cobs_stream_decode(&reader, input, input_length, output, output_length, delim);
	if (written) *written = reader.last_written_bytes;
	if (read) *read = reader.last_read_bytes;
	return (result == COBS_DONE) ? (int)reader.last_read_bytes : -1;
}


