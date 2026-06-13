#include "ubx.h"
#include <string.h>

#ifndef UBX_MAX_PAYLOAD
#define UBX_MAX_PAYLOAD 128
#endif

#define UBX_SYNC1     0xB5
#define UBX_SYNC2     0x62
#define UBX_CLASS_NAV 0x01
#define UBX_NAV_PVT   0x07

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

static UART_HandleTypeDef *s_huart;
static uint8_t             s_rxbuf[256];
static ubx_parser_t        s_parser;
static volatile uint8_t    s_frame_ready;
static ubx_nav_pvt_t       s_pvt_snapshot;

void gps_init(UART_HandleTypeDef *huart) {
    s_huart = huart;
    parser_init(&s_parser);
    HAL_UARTEx_ReceiveToIdle_IT(s_huart, s_rxbuf, sizeof(s_rxbuf));
}

void gps_uart_rx_event(uint16_t size) {
    for (uint16_t i = 0; i < size; i++) {
        if (parser_feed(&s_parser, s_rxbuf[i])
                && s_parser.msg_class == UBX_CLASS_NAV
                && s_parser.msg_id    == UBX_NAV_PVT
                && s_parser.length    == sizeof(ubx_nav_pvt_t)) {
            memcpy(&s_pvt_snapshot, s_parser.payload, sizeof(ubx_nav_pvt_t));
            s_frame_ready = 1;
        }
    }
    HAL_UARTEx_ReceiveToIdle_IT(s_huart, s_rxbuf, sizeof(s_rxbuf));
}

void gps_uart_error(void) {
    HAL_UARTEx_ReceiveToIdle_IT(s_huart, s_rxbuf, sizeof(s_rxbuf));
}

bool gps_process(ubx_nav_pvt_t *out) {
    if (!s_frame_ready) return false;
    s_frame_ready = 0;
    *out = s_pvt_snapshot;
    return true;
}
