#ifndef UBX_H
#define UBX_H

#include <stdbool.h>
#include <stdint.h>
#include "stm32g4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct __attribute__((packed)) {
    uint32_t iTOW;
    uint16_t year;
    uint8_t  month, day, hour, min, sec, valid;
    uint32_t tAcc;
    int32_t  nano;
    uint8_t  fixType, flags, flags2, numSV;
    int32_t  lon;        // deg * 1e-7
    int32_t  lat;        // deg * 1e-7
    int32_t  height;     // mm above ellipsoid
    int32_t  hMSL;       // mm above MSL
    uint32_t hAcc;       // mm
    uint32_t vAcc;       // mm
    int32_t  velN;       // mm/s
    int32_t  velE;       // mm/s
    int32_t  velD;       // mm/s
    int32_t  gSpeed;     // mm/s ground speed
    int32_t  headMot;    // deg * 1e-5
    uint32_t sAcc;       // mm/s
    uint32_t headAcc;    // deg * 1e-5
    uint16_t pDOP;       // * 0.01
    uint16_t flags3;
    uint8_t  reserved0[4];
    int32_t  headVeh;    // deg * 1e-5
    int16_t  magDec;     // deg * 1e-2
    uint16_t magAcc;     // deg * 1e-2
} ubx_nav_pvt_t;

_Static_assert(sizeof(ubx_nav_pvt_t) == 92, "ubx_nav_pvt_t size mismatch");

static inline bool   ubx_pvt_fix_valid(const ubx_nav_pvt_t *p) {
    return (p->fixType == 2 || p->fixType == 3 || p->fixType == 4)
        && (p->flags  & 0x01)
        && !(p->flags3 & 0x01);
}
static inline double ubx_pvt_lat_deg(const ubx_nav_pvt_t *p) { return p->lat * 1e-7; }
static inline double ubx_pvt_lon_deg(const ubx_nav_pvt_t *p) { return p->lon * 1e-7; }

// huart2 may be NULL for single-GPS mode
void gps_init(UART_HandleTypeDef *huart1, UART_HandleTypeDef *huart2);

// Call in main loop. heading_deg may be NULL.
// Dual: heading from antenna baseline. Single: course-over-ground from GPS.
bool gps_process(ubx_nav_pvt_t *out, float *heading_deg);

// Wire these to HAL_UARTEx_RxEventCallback and HAL_UART_ErrorCallback
void gps_uart_rx_event(UART_HandleTypeDef *huart, uint16_t size);
void gps_uart_error(UART_HandleTypeDef *huart);

// Total valid NAV-PVT packets received from GPS idx (0 or 1)
uint32_t gps_packet_count(int idx);

#ifdef __cplusplus
}
#endif

#endif // UBX_H
