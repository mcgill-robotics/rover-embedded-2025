// ubx.c — Minimal u-blox UBX protocol parser. See ubx.h for the API.

#include "ubx.h"
#include <string.h>

void ubx_parser_init(ubx_parser_t *p)
{
    memset(p, 0, sizeof(*p));
    p->state = UBX_STATE_SYNC1;
}

static inline void ck_update(ubx_parser_t *p, uint8_t byte)
{
    // 8-bit Fletcher over class, id, length, payload
    p->ck_a = (uint8_t)(p->ck_a + byte);
    p->ck_b = (uint8_t)(p->ck_b + p->ck_a);
}

bool ubx_parser_feed(ubx_parser_t *p, uint8_t byte)
{
    switch (p->state) {

    case UBX_STATE_SYNC1:
        if (byte == UBX_SYNC1)
            p->state = UBX_STATE_SYNC2;
        break;

    case UBX_STATE_SYNC2:
        if (byte == UBX_SYNC2) {
            p->ck_a = 0;
            p->ck_b = 0;
            p->state = UBX_STATE_CLASS;
        } else if (byte != UBX_SYNC1) {
            // 0xB5 0xB5 0x62 ... - stay armed on a repeated 0xB5
            p->state = UBX_STATE_SYNC1;
        }
        break;

    case UBX_STATE_CLASS:
        p->msg_class = byte;
        ck_update(p, byte);
        p->state = UBX_STATE_ID;
        break;

    case UBX_STATE_ID:
        p->msg_id = byte;
        ck_update(p, byte);
        p->state = UBX_STATE_LEN1;
        break;

    case UBX_STATE_LEN1:
        p->length = byte;
        ck_update(p, byte);
        p->state = UBX_STATE_LEN2;
        break;

    case UBX_STATE_LEN2:
        p->length |= (uint16_t)byte << 8;
        ck_update(p, byte);
        p->count = 0;
        p->oversized = (p->length > UBX_MAX_PAYLOAD);
        if (p->oversized)
            p->frames_oversize++;
        p->state = (p->length == 0) ? UBX_STATE_CK_A : UBX_STATE_PAYLOAD;
        break;

    case UBX_STATE_PAYLOAD:
        ck_update(p, byte);
        if (!p->oversized)
            p->payload[p->count] = byte;
        if (++p->count >= p->length)
            p->state = UBX_STATE_CK_A;
        break;

    case UBX_STATE_CK_A:
        if (byte == p->ck_a) {
            p->state = UBX_STATE_CK_B;
        } else {
            p->frames_crc_err++;
            p->state = UBX_STATE_SYNC1;
        }
        break;

    case UBX_STATE_CK_B:
        p->state = UBX_STATE_SYNC1;
        if (byte == p->ck_b) {
            if (!p->oversized) {
                p->frames_ok++;
                return true;
            }
        } else {
            p->frames_crc_err++;
        }
        break;
    }

    return false;
}

void ubx_parser_feed_buf(ubx_parser_t *p, const uint8_t *buf, size_t len,
                         ubx_frame_cb_t cb, void *user)
{
    for (size_t i = 0; i < len; i++) {
        if (ubx_parser_feed(p, buf[i]) && cb)
            cb(p, user);
    }
}

bool ubx_decode_nav_pvt(const ubx_parser_t *p, ubx_nav_pvt_t *out)
{
    if (p->msg_class != UBX_CLASS_NAV || p->msg_id != UBX_NAV_PVT ||
        p->length != UBX_NAV_PVT_LEN)
        return false;
    memcpy(out, p->payload, UBX_NAV_PVT_LEN);
    return true;
}

size_t ubx_build_frame(uint8_t msg_class, uint8_t msg_id,
                       const uint8_t *payload, uint16_t payload_len,
                       uint8_t *out, size_t out_size)
{
    const size_t total = 8u + payload_len;
    if (out_size < total)
        return 0;

    out[0] = UBX_SYNC1;
    out[1] = UBX_SYNC2;
    out[2] = msg_class;
    out[3] = msg_id;
    out[4] = (uint8_t)(payload_len & 0xFF);
    out[5] = (uint8_t)(payload_len >> 8);
    if (payload_len)
        memcpy(&out[6], payload, payload_len);

    uint8_t ck_a = 0, ck_b = 0;
    for (size_t i = 2; i < 6u + payload_len; i++) {
        ck_a = (uint8_t)(ck_a + out[i]);
        ck_b = (uint8_t)(ck_b + ck_a);
    }
    out[6 + payload_len] = ck_a;
    out[7 + payload_len] = ck_b;

    return total;
}

// M10 configuration keys (u-blox M10 interface description, SPG 5.00)
#define KEY_CFG_UART1OUTPROT_NMEA         0x10740002u  // L  (bool)
#define KEY_CFG_MSGOUT_UBX_NAV_PVT_UART1  0x20910007u  // U1
#define KEY_CFG_RATE_MEAS                 0x30210001u  // U2, ms

static size_t put_key(uint8_t *dst, uint32_t key)
{
    dst[0] = (uint8_t)(key);
    dst[1] = (uint8_t)(key >> 8);
    dst[2] = (uint8_t)(key >> 16);
    dst[3] = (uint8_t)(key >> 24);
    return 4;
}

size_t ubx_build_init_valset(uint16_t meas_rate_ms,
                             uint8_t *out, size_t out_size)
{
    uint8_t payload[4 + (4 + 1) + (4 + 1) + (4 + 2)];
    size_t n = 0;

    payload[n++] = 0x00; // version 0
    payload[n++] = 0x01; // layers: RAM only
    payload[n++] = 0x00;
    payload[n++] = 0x00;

    n += put_key(&payload[n], KEY_CFG_UART1OUTPROT_NMEA);
    payload[n++] = 0x00; // NMEA off

    n += put_key(&payload[n], KEY_CFG_MSGOUT_UBX_NAV_PVT_UART1);
    payload[n++] = 0x01; // one NAV-PVT per epoch

    if (meas_rate_ms != 0) {
        n += put_key(&payload[n], KEY_CFG_RATE_MEAS);
        payload[n++] = (uint8_t)(meas_rate_ms & 0xFF);
        payload[n++] = (uint8_t)(meas_rate_ms >> 8);
    }

    return ubx_build_frame(UBX_CLASS_CFG, UBX_CFG_VALSET,
                           payload, (uint16_t)n, out, out_size);
}
