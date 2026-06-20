#ifndef DESERIALIZATION_H
#define DESERIALIZATION_H

#include "rosjam_internal.h"
#include "rosjam.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct DeserializationResult {
	RosjamEndpoint* endpoint;
	char* message;
} DeserializationResult;

RosjamEndpoint* deserialize(ActiveEndpoints* endpoints, char *json, char* message_buf, int msg_buf_len);

#ifdef __cplusplus
}
#endif

#endif