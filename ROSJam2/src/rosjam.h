#ifndef ROSJAM_H
#define ROSJAM_H

#include "default_rosjam_config.h"
#include "stm32g4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

double string_to_float(char* string);

int float_to_string(double number, int precision, char* buf, int buf_len);

int int_to_string(int number, char* buf, int buf_len);

void setup_simple();

void print_to_usb(char* message);

void send_msg_raw(char *message, int message_len);

int has_data();

char read_char();

void process_simple();


#ifdef __cplusplus
}
#endif

#endif