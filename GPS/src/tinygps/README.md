# TinyGPS STM32 Port

C/C++ GPS library port of TinyGPS for STM32G4 (HAL). Supports both UXB-NAV-PVT and standard NMEA GPS protocols.

---

## Data Structures

### `gps_data_t`
Holds a complete position snapshot.

| Field | Type | Unit | Description |
|-------|------|------|-------------|
| `lat` | `double` | degrees | Latitude (positive = North) |
| `lon` | `double` | degrees | Longitude (positive = East) |
| `alt` | `double` | metres | Altitude above mean sea level |
| `gSpeed` | `double` | m/s | Ground speed |
| `headMot` | `double` | degrees | Heading of motion (0 = North) |
| `numSV` | `int` | - | Number of satellites used |
| `fixType` | `int` | - | Fix type (0 = none, 1 = dead reckoning, 2 = 2D, 3 = 3D) |

### `gps_t`
Internal GPS state. Declare one per physical receiver. Do not access fields directly, use the API functions below.

---

## API

### `gps_init`
```c
void gps_init(gps_t *g, int type, UART_HandleTypeDef *huart, bool use_ekf);
```
Initializes the GPS instance and reconfigures the UART baud rate automatically:
- `GPS_UBX` uses 115200 baud (new M10G-5883 is pre-configured for this baud rate)
- `GPS_NMEA` uses 9600 baud (old GPS uses slower baud rate)

Call this after `HAL_Init()` and peripheral init, before starting UART interrupts.

`use_ekf = true` enables the Kalman filter.

---

### `gps_process`
```c
bool gps_process(gps_t *g, uint8_t byte);
```
Feeds one byte into the parser. Returns `true` when a complete, valid frame with a fix has been parsed. Call this from `HAL_UART_RxCpltCallback`.

---

### `gps_read_snapshot`
```c
bool gps_read_snapshot(gps_t *g, gps_data_t *out);
```
Reads the latest position snapshot from the main loop. Returns `true` if a new frame is available since the last call. Thread-safe since it disables IRQs during copy.

---

### `gps_read_combined`
```c
bool gps_read_combined(gps_t *a, gps_t *b, gps_data_t *out);
```
Fuses two GPS receivers into one `gps_data_t`. Weighted average by `numSV` for position, altitude, speed, and heading. `fixType` uses the pessimistic (lower) value. `numSV` is summed.

Fallback behavior:
- If one receiver has no new frame, it returns the other unmodified.
- If have no new frame, it returns `false`.

---

## Kalman Filter

The EKF models position as static (`F = I`) with a direct measurement (`H = I`).
It is seeded automatically on the first valid fix, so no warm-up drift occurs.

**Process noise (`Q_VAL`)** controls the smoothing/tracking trade-off:

| Value | Behaviour |
|-------|-----------|
| `1e-12` | Heavy smoothing. Best static accuracy, slow to track fast motion. |
| `1e-10` | Default. Good balance for slow-moving rovers. |
| `1e-8` | Light smoothing. Tracks fast motion, minimal noise reduction. |

Change `Q_VAL` in `tinygps.cpp` → `apply_ekf()`.

**Measurement noise (`R`)** is built automatically from the receiver's reported
horizontal accuracy (`hAcc` for UBX, HDOP for scale for NMEA). No manual tuning
needed.

---

## Integration in `main.c`

### Single GPS

**1. Declare variables**
```c
gps_t gps_1;
static uint8_t gps_1_byte;
```

**2. Initialize**
```c
gps_init(&gps_1, GPS_UBX, &huart4, true);
HAL_UART_Receive_IT(gps_1.huart, &gps_1_byte, 1);
```

**3. Feed bytes**
```c
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart == gps_1.huart) {
        gps_process(&gps_1, gps_1_byte);
        HAL_UART_Receive_IT(gps_1.huart, &gps_1_byte, 1);
    }
}
```

**4. Read in the main loop**
```c
gps_data_t data;
if (gps_read_snapshot(&gps_1, &data)) {
    // use data.lat, data.lon, data.alt, etc.
}
```

---

### Dual GPS

Add a second UART peripheral in CubeMX first, then regenerate.

**1. Declare variables**
```c
gps_t gps_1, gps_2;
static uint8_t gps_1_byte, gps_2_byte;
```

**2. Initialize**
```c
gps_init(&gps_1, GPS_UBX, &huart3, true);
gps_init(&gps_2, GPS_UBX, &huart4, true);
HAL_UART_Receive_IT(gps_1.huart, &gps_1_byte, 1);
HAL_UART_Receive_IT(gps_2.huart, &gps_2_byte, 1);
```

**3. Feed bytes**
```c
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart == gps_1.huart) {
        gps_process(&gps_1, gps_1_byte);
        HAL_UART_Receive_IT(gps_1.huart, &gps_1_byte, 1);
    }
    if (huart == gps_2.huart) {
        gps_process(&gps_2, gps_2_byte);
        HAL_UART_Receive_IT(gps_2.huart, &gps_2_byte, 1);
    }
}
```

**4. Read output**
```c
gps_data_t data;
if (gps_read_combined(&gps_1, &gps_2, &data)) {
    // Use data.numSV, data.lat, data.lon, data.headMot
}
```
