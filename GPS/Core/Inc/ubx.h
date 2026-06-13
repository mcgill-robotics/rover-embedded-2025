// ubx.h — u-blox UBX NAV-PVT parser + GPS integration for M10 (SPG 5.00).

#ifndef UBX_H
#define UBX_H

#include <stdbool.h>
#include <stdint.h>
#include "stm32g4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

// UBX-NAV-PVT payload (0x01 0x07), 92 bytes, little-endian
typedef struct __attribute__((packed)) {
    uint32_t iTOW;
    uint16_t year;
    uint8_t  month;
    uint8_t  day;
    uint8_t  hour;
    uint8_t  min;
    uint8_t  sec;
    uint8_t  valid;
    uint32_t tAcc;       // ns
    int32_t  nano;       // ns
    uint8_t  fixType;    // 0=no fix, 2=2D, 3=3D, 4=GNSS+DR
    uint8_t  flags;
    uint8_t  flags2;
    uint8_t  numSV;
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

#define UBX_PVT_VALID_DATE           (1u << 0)
#define UBX_PVT_VALID_TIME           (1u << 1)
#define UBX_PVT_VALID_FULLY_RESOLVED (1u << 2)

#define UBX_PVT_FLAGS_GNSS_FIX_OK   (1u << 0)
#define UBX_PVT_FLAGS3_INVALID_LLH  (1u << 0)

static inline bool ubx_pvt_fix_valid(const ubx_nav_pvt_t *pvt)
{
    return (pvt->fixType == 2 || pvt->fixType == 3 || pvt->fixType == 4)
        && (pvt->flags & UBX_PVT_FLAGS_GNSS_FIX_OK)
        && !(pvt->flags3 & UBX_PVT_FLAGS3_INVALID_LLH);
}

static inline double ubx_pvt_lat_deg(const ubx_nav_pvt_t *pvt) { return pvt->lat * 1e-7; }
static inline double ubx_pvt_lon_deg(const ubx_nav_pvt_t *pvt) { return pvt->lon * 1e-7; }

void gps_init(UART_HandleTypeDef *huart);
void gps_uart_rx_event(uint16_t size);
void gps_uart_error(void);
bool gps_process(ubx_nav_pvt_t *out);

#ifdef __cplusplus
}
#endif

#endif // UBX_H
