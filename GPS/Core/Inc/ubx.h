// ubx.h — Minimal u-blox UBX protocol parser for M10 (SPG 5.00, UBX-20053845).
// One ubx_parser_t per receiver; feed bytes in, get complete frames out.

#ifndef UBX_H
#define UBX_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// NAV-PVT is 92 bytes; 128 leaves headroom for ACKs. Oversized frames are
// skipped (payload discarded) without losing sync.
#ifndef UBX_MAX_PAYLOAD
#define UBX_MAX_PAYLOAD 128
#endif

#define UBX_SYNC1 0xB5
#define UBX_SYNC2 0x62

#define UBX_CLASS_NAV     0x01
#define UBX_NAV_PVT       0x07

#define UBX_CLASS_ACK     0x05
#define UBX_ACK_NAK       0x00
#define UBX_ACK_ACK       0x01

#define UBX_CLASS_CFG     0x06
#define UBX_CFG_VALSET    0x8A

typedef enum {
    UBX_STATE_SYNC1 = 0,
    UBX_STATE_SYNC2,
    UBX_STATE_CLASS,
    UBX_STATE_ID,
    UBX_STATE_LEN1,
    UBX_STATE_LEN2,
    UBX_STATE_PAYLOAD,
    UBX_STATE_CK_A,
    UBX_STATE_CK_B,
} ubx_state_t;

typedef struct {
    ubx_state_t state;
    uint8_t  msg_class;
    uint8_t  msg_id;
    uint16_t length;
    uint16_t count;
    bool     oversized;
    uint8_t  ck_a, ck_b;
    uint8_t  payload[UBX_MAX_PAYLOAD];
    uint32_t frames_ok;
    uint32_t frames_crc_err;
    uint32_t frames_oversize;
} ubx_parser_t;

void ubx_parser_init(ubx_parser_t *p);

// Returns true once per complete, checksum-valid frame. Frame data lives in
// p->msg_class/msg_id/length/payload until the next byte is fed.
bool ubx_parser_feed(ubx_parser_t *p, uint8_t byte);

typedef void (*ubx_frame_cb_t)(const ubx_parser_t *p, void *user);
void ubx_parser_feed_buf(ubx_parser_t *p, const uint8_t *buf, size_t len,
                         ubx_frame_cb_t cb, void *user);

// UBX-NAV-PVT payload (0x01 0x07), 92 bytes, little-endian wire format.
typedef struct __attribute__((packed)) {
    uint32_t iTOW;       // ms
    uint16_t year;       // UTC
    uint8_t  month;      // UTC 1..12
    uint8_t  day;        // UTC 1..31
    uint8_t  hour;       // UTC 0..23
    uint8_t  min;        // UTC 0..59
    uint8_t  sec;        // UTC 0..60
    uint8_t  valid;      // see UBX_PVT_VALID_* bits
    uint32_t tAcc;       // ns
    int32_t  nano;       // ns, -1e9..1e9
    uint8_t  fixType;    // 0=no fix, 1=DR only, 2=2D, 3=3D, 4=GNSS+DR, 5=time only
    uint8_t  flags;      // see UBX_PVT_FLAGS_* bits
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
    int32_t  gSpeed;     // mm/s, 2D ground speed
    int32_t  headMot;    // deg * 1e-5, heading of motion
    uint32_t sAcc;       // mm/s
    uint32_t headAcc;    // deg * 1e-5
    uint16_t pDOP;       // * 0.01
    uint16_t flags3;     // see UBX_PVT_FLAGS3_* bits
    uint8_t  reserved0[4];
    int32_t  headVeh;    // deg * 1e-5, heading of vehicle
    int16_t  magDec;     // deg * 1e-2
    uint16_t magAcc;     // deg * 1e-2
} ubx_nav_pvt_t;

#define UBX_NAV_PVT_LEN 92
_Static_assert(sizeof(ubx_nav_pvt_t) == UBX_NAV_PVT_LEN,
               "ubx_nav_pvt_t must be exactly 92 bytes — check packing");

// valid field (offset 11)
#define UBX_PVT_VALID_DATE           (1u << 0)
#define UBX_PVT_VALID_TIME           (1u << 1)
#define UBX_PVT_VALID_FULLY_RESOLVED (1u << 2)
#define UBX_PVT_VALID_MAG            (1u << 3)

// flags field (offset 21)
#define UBX_PVT_FLAGS_GNSS_FIX_OK    (1u << 0)
#define UBX_PVT_FLAGS_DIFF_SOLN      (1u << 1)
#define UBX_PVT_FLAGS_HEAD_VEH_VALID (1u << 5)

// flags3 field (offset 78)
#define UBX_PVT_FLAGS3_INVALID_LLH   (1u << 0)

bool ubx_decode_nav_pvt(const ubx_parser_t *p, ubx_nav_pvt_t *out);

// Position is trustworthy: 2D/3D fix, fixOK set, coordinates valid.
static inline bool ubx_pvt_fix_valid(const ubx_nav_pvt_t *pvt)
{
    return (pvt->fixType == 2 || pvt->fixType == 3 || pvt->fixType == 4)
        && (pvt->flags & UBX_PVT_FLAGS_GNSS_FIX_OK)
        && !(pvt->flags3 & UBX_PVT_FLAGS3_INVALID_LLH);
}

static inline double ubx_pvt_lat_deg(const ubx_nav_pvt_t *pvt) { return pvt->lat * 1e-7; }
static inline double ubx_pvt_lon_deg(const ubx_nav_pvt_t *pvt) { return pvt->lon * 1e-7; }

size_t ubx_build_frame(uint8_t msg_class, uint8_t msg_id,
                       const uint8_t *payload, uint16_t payload_len,
                       uint8_t *out, size_t out_size);

size_t ubx_build_init_valset(uint16_t meas_rate_ms,
                             uint8_t *out, size_t out_size);

#ifdef __cplusplus
}
#endif

#endif // UBX_H
