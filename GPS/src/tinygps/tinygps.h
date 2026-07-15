#ifndef TINYGPS_C_H
#define TINYGPS_C_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include "stm32g4xx_hal.h"
#include "tinyubx.h"

#define EKF_N 2
#define EKF_M 2
#include "tinyekf.h"

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
} gps_data_t;

typedef struct {
    UART_HandleTypeDef *huart;
    int                 type;   // GPS_UBX or GPS_NMEA
    union {
        ubx_parser_t    ubx;    // Used for UBX GPS by TinyUBX
        void           *nmea;   // Used for NMEA GPS by TinyGPS
    };
    volatile bool       frame_ready;
    gps_data_t          snapshot;   // Last useful GPS data, use read_snapshot to get this info
    bool                use_ekf;
    ekf_t               ekf;
} gps_t;

void gps_init(gps_t *g, int type, UART_HandleTypeDef *huart, bool use_ekf);
bool gps_process(gps_t *g, uint8_t byte);
bool gps_read_snapshot(gps_t *g, gps_data_t *out);              // Used to get gps_data_t from a single GPS
bool gps_read_combined(gps_t *a, gps_t *b, gps_data_t *out);    // Used to get gps_data_t from two different GPS

#ifdef __cplusplus
}
#endif

#endif