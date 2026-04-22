#ifndef SERIALIZATION_H
#define SERIALIZATION_H

#include <stdint.h>
#include <stddef.h>

#include "buffers.h"

#ifdef __cplusplus
extern "C" {
#endif

size_t serialize(Buffer* buffer, const char *topic, uint8_t *msg);
void serialize_simple(const char *topic, uint8_t *msg);
// void send_msg(const char *topic, uint8_t *msg);
#ifdef __cplusplus
}
#endif

#endif