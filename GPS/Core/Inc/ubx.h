#ifndef UBX_H
#define UBX_H

#include <stdbool.h>
#include <stdint.h>
#include "stm32g4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

// ── NAV-PVT payload (0x01 0x07), 92 bytes, little-endian ───────────────────

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

// ── Parser internals (exposed so gps_t can embed them) ──────────────────────

#define UBX_MAX_PAYLOAD 128

typedef enum {
    S_SYNC1 = 0, S_SYNC2, S_CLASS, S_ID, S_LEN1, S_LEN2, S_PAYLOAD, S_CK_A, S_CK_B,
} ubx_state_t;

typedef struct {
    ubx_state_t state;
    uint8_t  msg_class, msg_id;
    uint16_t length, count;
    bool     oversized;
    uint8_t  ck_a, ck_b;
    uint8_t  payload[UBX_MAX_PAYLOAD];
} ubx_parser_t;

typedef struct {
    UART_HandleTypeDef *huart;
    uint8_t             rxbuf[256];
    ubx_parser_t        parser;
    volatile uint8_t    frame_ready;
    ubx_nav_pvt_t       pvt_snapshot;
} gps_t;

void gps_init(gps_t *gps, UART_HandleTypeDef *huart);
void gps_uart_rx_event(gps_t *gps, uint16_t size);
void gps_uart_error(gps_t *gps);
bool gps_process(gps_t *gps, ubx_nav_pvt_t *out);

#ifdef __cplusplus
}
#endif

#endif // UBX_H
