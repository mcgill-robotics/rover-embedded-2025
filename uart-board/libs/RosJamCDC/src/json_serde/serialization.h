#ifndef SERIALIZATION_H
#define SERIALIZATION_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

size_t serialize(uint8_t *output, size_t output_len, const char *topic, uint8_t *msg);
void send_msg(const char *topic, uint8_t *msg);
#ifdef __cplusplus
}
#endif

#endif