#ifndef ROSJAM_INTERNAL_H
#define ROSJAM_INTERNAL_H

#include "default_rosjam_config.h"
#include "buffers.h"
#include "rosjam.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct ActiveEndpoints {
	int size;
	int nextTxEndpoint;
	int first_message;
	RosjamEndpoint* endpoints[ENDPOINT_COUNT];
	Buffer global_rx_buffer;
	int hasPending;
	RosjamEndpoint* diag;
} ActiveEndpoints;

#ifdef __cplusplus
}
#endif

#endif