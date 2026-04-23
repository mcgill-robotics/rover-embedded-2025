#ifndef COBS_H
#define COBS_H
#include <stdint.h>
int estimate_encoded_size(int buf_size);
int decode(uint8_t* input, int input_length, uint8_t* output, int output_length, uint8_t delim, int* written);
int encode(uint8_t* input, int input_length, uint8_t* output, int output_length, uint8_t delim);
#endif