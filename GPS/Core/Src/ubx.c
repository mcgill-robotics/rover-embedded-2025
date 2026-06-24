#ifndef USE_NMEA_GPS

#include "ubx.h"
#include <math.h>
#include <string.h>

#define UBX_SYNC1       0xB5
#define UBX_SYNC2       0x62
#define UBX_CLASS_NAV   0x01
#define UBX_NAV_PVT     0x07
#define UBX_MAX_PAYLOAD 128

typedef enum { S_SYNC1, S_SYNC2, S_CLASS, S_ID, S_LEN1, S_LEN2, S_PAYLOAD, S_CK_A, S_CK_B } state_t;

typedef struct {
    state_t  state;
    uint8_t  cls, id, ck_a, ck_b;
    uint16_t len, count;
    bool     oversized;
    uint8_t  payload[UBX_MAX_PAYLOAD];
} parser_t;

typedef struct {
    UART_HandleTypeDef *huart;
    uint8_t             rxbuf[256];
    parser_t            parser;
    volatile bool       ready;
    ubx_nav_pvt_t       snap;
    uint32_t            pkt_count;
} gps_t;

static gps_t         g_gps[2];
static int           g_count;
static ubx_nav_pvt_t g_last[2];
static bool          g_has[2];

#ifdef USE_HEADING
#ifdef USE_KALMAN_FILTER

typedef struct { float x; float P; bool init; } hdg_kf_t;
static hdg_kf_t g_kf;

static float wrap180(float a) {
    while (a >  180.0f) a -= 360.0f;
    while (a < -180.0f) a += 360.0f;
    return a;
}

// Scalar Kalman: random-walk process model, direct heading measurement.
// Predict: P⁻ = P + Q
// Update:  K = P⁻/(P⁻+R),  x += K·wrap180(z−x),  P = (1−K)·P⁻
static float filter_heading(float z) {
    if (!g_kf.init) {
        g_kf.x    = z;
        g_kf.P    = KF_R;
        g_kf.init = true;
        return z;
    }
    g_kf.P   += KF_Q;
    float K   = g_kf.P / (g_kf.P + KF_R);
    g_kf.x   += K * wrap180(z - g_kf.x);
    g_kf.P   *= (1.0f - K);
    if (g_kf.x <    0.0f) g_kf.x += 360.0f;
    if (g_kf.x >= 360.0f) g_kf.x -= 360.0f;
    return g_kf.x;
}

#else
#define filter_heading(z) (z)
#endif
#endif

static bool feed(parser_t *p, uint8_t b) {
    switch (p->state) {
    case S_SYNC1:  if (b == UBX_SYNC1) p->state = S_SYNC2; break;
    case S_SYNC2:
        if      (b == UBX_SYNC2) { p->ck_a = 0; p->ck_b = 0; p->state = S_CLASS; }
        else if (b != UBX_SYNC1)  p->state = S_SYNC1;
        break;
    case S_CLASS:   p->cls = b; p->ck_a += b; p->ck_b += p->ck_a; p->state = S_ID;   break;
    case S_ID:      p->id  = b; p->ck_a += b; p->ck_b += p->ck_a; p->state = S_LEN1; break;
    case S_LEN1:    p->len = b; p->ck_a += b; p->ck_b += p->ck_a; p->state = S_LEN2; break;
    case S_LEN2:
        p->len |= (uint16_t)b << 8; p->ck_a += b; p->ck_b += p->ck_a;
        p->count = 0; p->oversized = (p->len > UBX_MAX_PAYLOAD);
        p->state = p->len ? S_PAYLOAD : S_CK_A;
        break;
    case S_PAYLOAD:
        p->ck_a += b; p->ck_b += p->ck_a;
        if (!p->oversized) p->payload[p->count] = b;
        if (++p->count >= p->len) p->state = S_CK_A;
        break;
    case S_CK_A: p->state = (b == p->ck_a) ? S_CK_B : S_SYNC1; break;
    case S_CK_B:
        p->state = S_SYNC1;
        return b == p->ck_b && !p->oversized;
    }
    return false;
}

void gps_init(UART_HandleTypeDef *huart1, UART_HandleTypeDef *huart2) {
    UART_HandleTypeDef *uarts[2] = {huart1, huart2};
    g_count = 0;
    memset(g_has, 0, sizeof(g_has));
#ifdef USE_KALMAN_FILTER
    g_kf.init = false;
#endif
    for (int i = 0; i < 2; i++) {
        if (!uarts[i]) continue;
        g_gps[g_count].huart     = uarts[i];
        g_gps[g_count].ready     = false;
        g_gps[g_count].pkt_count = 0;
        memset(&g_gps[g_count].parser, 0, sizeof(g_gps[g_count].parser));
        HAL_UARTEx_ReceiveToIdle_IT(uarts[i], g_gps[g_count].rxbuf, sizeof(g_gps[g_count].rxbuf));
        g_count++;
    }
}

void gps_uart_rx_event(UART_HandleTypeDef *huart, uint16_t size) {
    for (int i = 0; i < g_count; i++) {
        if (g_gps[i].huart != huart) continue;
        gps_t *g = &g_gps[i];
        for (uint16_t j = 0; j < size; j++) {
            if (feed(&g->parser, g->rxbuf[j])
                    && g->parser.cls == UBX_CLASS_NAV
                    && g->parser.id  == UBX_NAV_PVT
                    && g->parser.len == sizeof(ubx_nav_pvt_t)) {
                memcpy(&g->snap, g->parser.payload, sizeof(ubx_nav_pvt_t));
                g->ready = true;
                g->pkt_count++;
            }
        }
        HAL_UARTEx_ReceiveToIdle_IT(huart, g->rxbuf, sizeof(g->rxbuf));
        return;
    }
}

void gps_uart_error(UART_HandleTypeDef *huart) {
    for (int i = 0; i < g_count; i++) {
        if (g_gps[i].huart == huart) {
            HAL_UARTEx_ReceiveToIdle_IT(huart, g_gps[i].rxbuf, sizeof(g_gps[i].rxbuf));
            return;
        }
    }
}

bool gps_process(ubx_nav_pvt_t *out, float *heading_deg) {
    if (g_count == 1) {
        if (!g_gps[0].ready) return false;
        g_gps[0].ready = false;
        *out = g_gps[0].snap;
#ifdef USE_HEADING
        if (heading_deg) *heading_deg = filter_heading(out->headMot * 1e-5f);
#endif
        return true;
    }

    if (!g_gps[0].ready && !g_gps[1].ready) return false;

    if (g_gps[0].ready) { g_gps[0].ready = false; g_last[0] = g_gps[0].snap; g_has[0] = true; }
    if (g_gps[1].ready) { g_gps[1].ready = false; g_last[1] = g_gps[1].snap; g_has[1] = true; }

    bool fix0 = g_has[0] && ubx_pvt_fix_valid(&g_last[0]);
    bool fix1 = g_has[1] && ubx_pvt_fix_valid(&g_last[1]);

    if (fix0 && fix1) {
        *out        = g_last[0];
        out->lat    = (int32_t)(((int64_t)g_last[0].lat    + g_last[1].lat)    / 2);
        out->lon    = (int32_t)(((int64_t)g_last[0].lon    + g_last[1].lon)    / 2);
        out->height = (int32_t)(((int64_t)g_last[0].height + g_last[1].height) / 2);
        out->hMSL   = (int32_t)(((int64_t)g_last[0].hMSL   + g_last[1].hMSL)   / 2);
        out->hAcc   = (uint32_t)(((uint64_t)g_last[0].hAcc  + g_last[1].hAcc)  / 2);
        out->vAcc   = (uint32_t)(((uint64_t)g_last[0].vAcc  + g_last[1].vAcc)  / 2);
        out->numSV  = g_last[0].numSV > g_last[1].numSV ? g_last[0].numSV : g_last[1].numSV;

#ifdef USE_HEADING
        double dlat  = (double)(g_last[0].lat - g_last[1].lat) * 1e-7;
        double dlon  = (double)(g_last[0].lon - g_last[1].lon) * 1e-7;
        double lat_r = (double)g_last[1].lat  * 1e-7 * (M_PI / 180.0);
        double hdg   = atan2(dlon * cos(lat_r), dlat) * (180.0 / M_PI);
        if (hdg < 0.0) hdg += 360.0;

        out->headVeh = (int32_t)(hdg * 1e5);
        if (heading_deg) *heading_deg = filter_heading((float)hdg);
#endif
    } else if (fix0) {
        *out = g_last[0];
#ifdef USE_HEADING
        if (heading_deg) *heading_deg = filter_heading(g_last[0].headMot * 1e-5f);
#endif
    } else if (fix1) {
        *out = g_last[1];
#ifdef USE_HEADING
        if (heading_deg) *heading_deg = filter_heading(g_last[1].headMot * 1e-5f);
#endif
    } else {
        *out = g_has[0] ? g_last[0] : g_last[1];
#ifdef USE_HEADING
        if (heading_deg) *heading_deg = filter_heading(out->headMot * 1e-5f);
#endif
    }

    return true;
}

uint32_t gps_packet_count(int idx) {
    if (idx < 0 || idx >= g_count) return 0;
    return g_gps[idx].pkt_count;
}

#endif