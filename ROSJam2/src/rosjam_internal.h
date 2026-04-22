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
	RosjamEndpoint* endpoints[ENDPOINT_COUNT];
	RosjamRxBuffer global_rx_buffer;
} ActiveEndpoints;

#ifdef __cplusplus
}
#endif

#endif