#ifndef COBS_H
#define COBS_H
#include <stdint.h>

typedef enum cobs_state_t {
	COBS_SYNC,
	COBS_OVERHEAD,
	COBS_CHUNK,
	COBS_STUFFED_BYTE
} cobs_state_t;

typedef enum cobs_result_t {
	COBS_DONE,
	COBS_INCOMPLETE_FRAME,
	COBS_OUTPUT_FULL,
	COBS_RESET
} cobs_result_t;

typedef struct cobs_reader_t {
	uint8_t chunk_bytes_left;
	cobs_state_t state;
	uint8_t overhead_next;
	uint32_t current_frame_size;
	uint32_t last_read_bytes;
	uint32_t last_written_bytes;
} cobs_reader_t;


int cobs_estimate_encoded_size(int buf_size);
int cobs_encode(uint8_t* input, int input_length, uint8_t* output, int output_length, uint8_t delim);
void cobs_setup_stream_reader(cobs_reader_t* reader);
cobs_result_t cobs_stream_decode(cobs_reader_t* reader, uint8_t* buf, int available, uint8_t* output, int output_length, uint8_t delim);
int cobs_estimate_decoded_size(int buf_size);
void cobs_consume_message(cobs_reader_t* reader);
int cobs_decode(uint8_t* input, int input_length, uint8_t* output, int output_length, uint8_t delim, int* written, int* read);
#endif
