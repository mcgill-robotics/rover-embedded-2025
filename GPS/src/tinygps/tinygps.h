#ifndef TINYGPS_C_H
#define TINYGPS_C_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include "stm32g4xx_hal.h"
#include "tinyubx.h"

#define GPS_UBX  0
#define GPS_NMEA 1

typedef struct {
    double  lat;      // degrees
    double  lon;      // degrees
    double  alt;      // m above sea level
    double  gSpeed;   // m/s ground speed
    double  headMot;  // deg heading of motion
    int     numSV;    // number of satellites
    int     fixType;
} gps_pvt_t;

typedef struct {
    UART_HandleTypeDef *huart;
    int                 type;   // GPS_UBX or GPS_NMEA
    union {
        ubx_parser_t    ubx;
        void           *nmea;   // TinyGPSPlus*, opaque from C
    };
    volatile bool       frame_ready;
    gps_pvt_t           snapshot;
} gps_t;

void gps_init(gps_t *g, int type, UART_HandleTypeDef *huart);
bool gps_process(gps_t *g, uint8_t byte);
bool gps_read_snapshot(gps_t *g, gps_pvt_t *out);

#ifdef __cplusplus
}
#endif

#endif
