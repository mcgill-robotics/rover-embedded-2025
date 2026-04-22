#include <stdint.h>
#include <string.h>
#include <stdio.h>

#define MAX_CHUNK_SIZE 255

/**
 * Returns the bytes written to the output buffer
 * -1 if not enough bytes are available in output_length
 * 
 */
int encode(uint8_t* input, int input_length, uint8_t* output, int output_length, uint8_t delim){
	uint8_t* output_initial = output;
	uint8_t* output_end = output+output_length;
	uint8_t* input_end = input+input_length;
	int chunk_size = 0;
	int non_stuffed_count = 0;
	if (2>=output_length){
		return -1;
	}
	*output = delim;
	uint8_t* last_replaced = output+1;
	output+=2; // reserve place for header and write initial delim
	for (;input<input_end;input++){
		uint8_t current_byte = *input;
		if (output+chunk_size+1 >= output_end){
			return -1;
		}
		if (chunk_size+1 == MAX_CHUNK_SIZE){
			printf("%c, %d, %p, %c\n", current_byte, chunk_size, input-chunk_size, *(input-chunk_size));
			memcpy(output, input-chunk_size, chunk_size);
			output+=chunk_size;
			*last_replaced = chunk_size+1;
			last_replaced = output;
			output++; //make space for next stuffed byte
			chunk_size = 0;
		}
		if (current_byte == delim) {
			printf("%c, %d, %p, %c\n", current_byte, chunk_size, input-chunk_size, *(input-chunk_size));
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
	if (output+chunk_size+1 >= output_end){
		return -1;
	}
	printf("%c, %d, %p, %c\n", *input, chunk_size, input-chunk_size, *(input-chunk_size));
	memcpy(output, input-chunk_size, chunk_size);
	output+=chunk_size;
	*output = delim;
	output++;

	return output-output_initial;
}

/**
 * Decodes one cobs encoded frame discarding any data before the first encountered delimiter
 * returns the number of read bytes
 * -1 if not enough bytes were available in output
 * -3 if message could not be found in input buffer
 * -2 not enough data and could not find first delim (all data to throw)
 */
int decode(uint8_t* input, int input_length, uint8_t* output, int output_length, uint8_t delim, int* written){
	*written = 0;
	int delim_count = 0;
	uint8_t* output_write_head = output;
	int next_delim_replacement;
	int chunk_size = 0;
	int read;
	for (read=0; read<input_length;read++){
		// printf("check written\n");
		// fflush(stdout);
		if (*written >= output_length){
			return -1;
		}

		// printf("find delim\n");
		// fflush(stdout);
		if (input[read] == delim){
			delim_count++;
			// printf("Found delim\n");
			// fflush(stdout);
			chunk_size = 255;
			next_delim_replacement = 255;
			// ignore first delim
			if (delim_count == 1){
				continue;
			}
		}

		// skip anything before first delim
		// stop on second delim
		if (delim_count==0){
			continue;
		} else if (delim_count>1){
			// printf("Last: %c, %d\n", input[read], delim_count);
			break;
		}

		uint8_t current_byte = input[read];
	
		if (chunk_size == next_delim_replacement){
			// printf("handling stuffed byte\n");
			// fflush(stdout);
			if (next_delim_replacement != 255){
				// printf("writing stuffed to output\n");
				// fflush(stdout);
				*output_write_head = delim;
				output_write_head++;
				(*written)++;
			}
			next_delim_replacement = current_byte;
			// printf("Next: %d\n", next_delim_replacement);
			chunk_size = 1;
			continue;
		}

		// printf("writing normal byte\n");
		// fflush(stdout);
		*output_write_head = current_byte;
		// printf("wrote normal byte\n");
		// fflush(stdout);
		output_write_head++;
		(*written)++;
		chunk_size++;
		
	}
	if (delim_count == 0){
		return -2;
	} else if (delim_count == 1){
		return -3;
	}
	return read+1; // + 1 because read is an index so it starts at 0
}