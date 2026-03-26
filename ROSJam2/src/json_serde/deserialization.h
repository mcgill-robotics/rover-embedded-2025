#ifndef DESERIALIZATION_H
#define DESERIALIZATION_H

#include "rosjam.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct DeserializationResult {
	RosjamEndpoint* endpoint;
	char* message;
} DeserializationResult;

DeserializationResult deserialize(ActiveEndpoints* endpoints, char *json);

#ifdef __cplusplus
}
#endif

#endif