#include "ubx.h"
#include <string.h>

#define UBX_SYNC1     0xB5
#define UBX_SYNC2     0x62
#define UBX_CLASS_NAV 0x01
#define UBX_NAV_PVT   0x07

static void parser_init(ubx_parser_t *p) { memset(p, 0, sizeof(*p)); }

static inline void ck_update(ubx_parser_t *p, uint8_t b) {
    p->ck_a = (uint8_t)(p->ck_a + b);
    p->ck_b = (uint8_t)(p->ck_b + p->ck_a);
}

static bool parser_feed(ubx_parser_t *p, uint8_t b) {
    switch (p->state) {
    case S_SYNC1:
        if (b == UBX_SYNC1) p->state = S_SYNC2;
        break;
    case S_SYNC2:
        if      (b == UBX_SYNC2) { p->ck_a = 0; p->ck_b = 0; p->state = S_CLASS; }
        else if (b != UBX_SYNC1)  p->state = S_SYNC1;
        break;
    case S_CLASS:
        p->msg_class = b; ck_update(p, b); p->state = S_ID;
        break;
    case S_ID:
        p->msg_id = b; ck_update(p, b); p->state = S_LEN1;
        break;
    case S_LEN1:
        p->length = b; ck_update(p, b); p->state = S_LEN2;
        break;
    case S_LEN2:
        p->length |= (uint16_t)b << 8; ck_update(p, b);
        p->count = 0;
        p->oversized = (p->length > UBX_MAX_PAYLOAD);
        p->state = (p->length == 0) ? S_CK_A : S_PAYLOAD;
        break;
    case S_PAYLOAD:
        ck_update(p, b);
        if (!p->oversized) p->payload[p->count] = b;
        if (++p->count >= p->length) p->state = S_CK_A;
        break;
    case S_CK_A:
        if (b == p->ck_a) p->state = S_CK_B;
        else              p->state = S_SYNC1;
        break;
    case S_CK_B:
        p->state = S_SYNC1;
        if (b == p->ck_b && !p->oversized) return true;
        break;
    }
    return false;
}

// ── GPS integration ──────────────────────────────────────────────────────────

void gps_init(gps_t *gps, UART_HandleTypeDef *huart) {
    gps->huart = huart;
    gps->frame_ready = 0;
    parser_init(&gps->parser);
    HAL_UARTEx_ReceiveToIdle_IT(gps->huart, gps->rxbuf, sizeof(gps->rxbuf));
}

void gps_uart_rx_event(gps_t *gps, uint16_t size) {
    for (uint16_t i = 0; i < size; i++) {
        if (parser_feed(&gps->parser, gps->rxbuf[i])
                && gps->parser.msg_class == UBX_CLASS_NAV
                && gps->parser.msg_id    == UBX_NAV_PVT
                && gps->parser.length    == sizeof(ubx_nav_pvt_t)) {
            memcpy(&gps->pvt_snapshot, gps->parser.payload, sizeof(ubx_nav_pvt_t));
            gps->frame_ready = 1;
        }
    }
    HAL_UARTEx_ReceiveToIdle_IT(gps->huart, gps->rxbuf, sizeof(gps->rxbuf));
}

void gps_uart_error(gps_t *gps) {
    HAL_UARTEx_ReceiveToIdle_IT(gps->huart, gps->rxbuf, sizeof(gps->rxbuf));
}

bool gps_process(gps_t *gps, ubx_nav_pvt_t *out) {
    if (!gps->frame_ready) return false;
    gps->frame_ready = 0;
    *out = gps->pvt_snapshot;
    return true;
}
