#ifndef UART_DEBUGGING_H
#define UART_DEBUGGING_H

#include <stdint.h>
#include <stdbool.h>
#include <stdarg.h>
#include <stdio.h>
#include "main.h"  // for huart2

extern volatile bool uartTxDone;

void uart_debug_print(const char *format, ...);

#endif // UART_DEBUGGING_H
