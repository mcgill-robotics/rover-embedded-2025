#ifndef COBS_H
#define COBS_H
#include <stdint.h>
int cobs_estimate_encoded_size(int buf_size);
int cobs_decode(uint8_t* input, int input_length, uint8_t* output, int output_length, uint8_t delim, int* written);
int cobs_encode(uint8_t* input, int input_length, uint8_t* output, int output_length, uint8_t delim);
#endif