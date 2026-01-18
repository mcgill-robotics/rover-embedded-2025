#ifndef SERIALIZATION_H
#define SERIALIZATION_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

void serialize(uint8_t *output, size_t output_len, const char *topic, uint8_t *msg, size_t msg_len);
void send_msg(const char *topic, uint8_t *msg, size_t msg_len);
#ifdef __cplusplus
}
#endif

#endif