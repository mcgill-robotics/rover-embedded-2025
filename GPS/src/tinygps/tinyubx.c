#include "tinyubx.h"
#include <string.h>

static bool ubx_parser_feed(ubx_parser_t *p, uint8_t byte) {
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
            p->state = UBX_STATE_SYNC1;
        }
        break;

    case UBX_STATE_CLASS:
        p->msg_class = byte;
        p->ck_a = (uint8_t)(p->ck_a + byte);
        p->ck_b = (uint8_t)(p->ck_b + p->ck_a);
        p->state = UBX_STATE_ID;
        break;

    case UBX_STATE_ID:
        p->msg_id = byte;
        p->ck_a = (uint8_t)(p->ck_a + byte);
        p->ck_b = (uint8_t)(p->ck_b + p->ck_a);
        p->state = UBX_STATE_LEN1;
        break;

    case UBX_STATE_LEN1:
        p->length = byte;
        p->ck_a = (uint8_t)(p->ck_a + byte);
        p->ck_b = (uint8_t)(p->ck_b + p->ck_a);
        p->state = UBX_STATE_LEN2;
        break;

    case UBX_STATE_LEN2:
        p->length |= (uint16_t)byte << 8;
        p->ck_a = (uint8_t)(p->ck_a + byte);
        p->ck_b = (uint8_t)(p->ck_b + p->ck_a);
        p->count = 0;
        p->oversized = (p->length > UBX_MAX_PAYLOAD);
        if (p->oversized) p->frames_oversize++;
        p->state = (p->length == 0) ? UBX_STATE_CK_A : UBX_STATE_PAYLOAD;
        break;

    case UBX_STATE_PAYLOAD:
        p->ck_a = (uint8_t)(p->ck_a + byte);
        p->ck_b = (uint8_t)(p->ck_b + p->ck_a);
        if (!p->oversized) p->payload[p->count] = byte;
        if (++p->count >= p->length) p->state = UBX_STATE_CK_A;
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

bool ubx_process(ubx_parser_t *p, ubx_nav_pvt_t *data, uint8_t byte) {
    if (!ubx_parser_feed(p, byte)) return false;
    if (p->msg_class != UBX_CLASS_NAV || p->msg_id != UBX_NAV_PVT || p->length != sizeof(ubx_nav_pvt_t))
        return false;
    memcpy(data, p->payload, sizeof(ubx_nav_pvt_t));
    return true;
}
