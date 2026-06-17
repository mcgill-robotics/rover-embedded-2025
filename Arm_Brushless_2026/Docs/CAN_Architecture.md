# CAN FD Architecture — B-G431B-ESC1

## 1. Overview

The CAN subsystem on the B-G431B-ESC1 handles two communication patterns over CAN FD at 500 kbps nominal / 2 Mbps data rate:

**Request-response** — the master (CANable adapter) sends a command frame, the ESC parses it, acts on it, and (for read commands) replies with the requested value. This is how the master issues motor commands (speed, position, stop) and polls for specific data on demand.

**Scheduled telemetry** — the ESC autonomously broadcasts sensor data at configurable intervals without being asked. The master receives a continuous stream of motor speed, position, voltage, temperature, currents, and status at rates from 100 ms to 1000 ms depending on the signal. This eliminates the need for the master to poll each ESC individually for routine data.

Both paths share the same 11-bit CAN ID encoding, the same sensor data cache, and the same CAN FD TX configuration. On the bus, solicited and unsolicited frames are structurally identical — the master distinguishes them by whether it recently sent a request for that ReadSpec.

---

## 2. File structure

```
Inc/
  can_common.h          ← Shared enums, bit-field masks, CAN ID builder
  esc_sensors.h         ← Sensor data cache structure and API
  can_telemetry.h       ← Telemetry scheduler table entry and API
  CAN_processing_v2.h   ← Request-response parser and TX API

Src/
  esc_sensors.c         ← All hardware reads (ADC, encoder, motor state)
  can_telemetry.c       ← Telemetry table, pack functions, 1 ms tick
  CAN_processing_v2.c   ← RX parsing, command dispatch, response TX
```

### Dependency graph

```
                 can_common.h
                /             \
  CAN_processing_v2.h     can_telemetry.h
        |                      |
  CAN_processing_v2.c     can_telemetry.c
        \                     /
          \                 /
           esc_sensors.h
                |
           esc_sensors.c
```

Both `.c` modules include `can_common.h` (through their headers) for the shared protocol definitions, and both call `ESC_Sensors_Get()` to read sensor data. Neither module touches hardware directly for sensor reads.

---

## 3. CAN ID encoding

All frames use 11-bit standard IDs with the following bit-field layout:

```
Bit:  [10]    [9]     [8]       [7]        [6:4]          [3:0]
      Sender  Action  Config    MotorType  Specification  DeviceID
```

| Bit(s) | Field         | Values |
|--------|---------------|--------|
| 10     | Sender        | 0 = Master, 1 = Slave/ESC |
| 9      | Action        | 0 = Run, 1 = Read |
| 8      | MotorConfig   | 0 = Multiple motors, 1 = Single motor |
| 7      | MotorType     | 0 = Drive, 1 = Steering |
| 6:4    | Specification | RunSpec (if Action=0) or ReadSpec (if Action=1) |
| 3:0    | Device ID     | 0–15, matched against ESC_ID |

The encoding is handled by a single inline function, `CAN_BuildID()`, defined in `can_common.h`. Both the telemetry scheduler and the response transmitter call it, so the ID construction logic exists in exactly one place.

For all ESC-originated frames (both telemetry broadcasts and solicited responses), Sender = 1 (SLAVE). This means other ESCs on the bus ignore these frames in `CAN_Parse_MSG()` since they only process frames where Sender = 0 (MASTER).

---

## 4. Protocol enums

All enums are defined once in `can_common.h`. The key specification enums are:

**RunSpec** (bits [6:4] when Action = RUN):

| Value | Name | Purpose |
|-------|------|---------|
| 0 | RUN_STOP | Stop the motor |
| 1 | RUN_ACKNOWLEDGE_FAULTS | Clear fault flags |
| 2 | RUN_SPEED | Set target speed (RPM) |
| 3 | RUN_POSITION | Set target position (degrees) |
| 4 | RUN_CALIBRATION | Start calibration sequence |
| 5–7 | Reserved | — |

**ReadSpec** (bits [6:4] when Action = READ):

| Value | Name | Purpose |
|-------|------|---------|
| 0 | READ_SPEED | Motor speed (rad/s or RPM) |
| 1 | READ_POSITION | Rotor / joint position |
| 2 | READ_VOLTAGE | DC bus voltage |
| 3 | READ_CURRENT | Phase currents |
| 4 | READ_TEMPERATURE | Board / FET temperature |
| 5 | READ_CURRENT_STATE | Fault flags, motor state |
| 6 | READ_PING | Heartbeat / ping |
| 7 | READ_CONTROL_MODE | Current control mode |

ReadSpec values are shared between the request-response path and the telemetry scheduler. The same value in bits [6:4] means the same signal regardless of how the frame was triggered.

---

## 5. File-by-file reference

### 5.1 `can_common.h`

The single source of truth for the CAN protocol layer. Contains:

- **Bit-field masks and shifts** (`CAN_SENDER_MASK`, `CAN_SENDER_SHIFT`, etc.) — used by the inline extractors and builder.
- **Protocol enums** — `Transmitter`, `Action`, `MotorConfig`, `MotorType`, `RunSpec`, `ReadSpec`, `MotorID`. Every module that touches CAN IDs includes this file.
- **`ParsedCANID` struct** — holds the decoded fields of a received frame. Uses an anonymous union for `runSpec` / `readSpec` since only one is valid at a time depending on the Action bit.
- **`CAN_BuildID()`** — inline function that assembles an 11-bit ID from its six constituent fields. Used by both `sendCANResponse()` and `Telemetry_Tick_1ms()`.
- **`CAN_Get*()` extractors** — inline functions that pull individual fields out of a raw 11-bit ID. Used by `CAN_Parse_MSG()`.

This file replaces the duplicate sets of `#define` masks and `static` extractor functions that previously existed independently in `CAN_processing_v2.c` and `can_telemetry.c`.

### 5.2 `esc_sensors.h` / `esc_sensors.c`

Centralizes all hardware sensor reads. The module owns:

- `ESC_Sensors_Update()` — called once per millisecond. Reads VBUS voltage from PA0 (ADC1 Rank 1 through a 19.32k/1.91k resistor divider), board temperature from the NTC on PB14 (ADC1 Rank 2, V_25 = 1400 mV, 19 mV/°C transfer function), motor speed, encoder position, phase currents, fault flags, and motor state. All values are stored in a static `ESC_SensorData` struct.
- `ESC_Sensors_Get()` — returns a const pointer to the latest snapshot. Both the telemetry pack functions and `Handle_Read_Command()` call this instead of performing their own ADC conversions.

The ADC read implementation handles the left-aligned 12-bit data format configured by CubeMX (data in bits [15:4], shifted right by 4 to get 0–4095 counts). The temperature and voltage conversion functions were moved here from `CAN_processing_v2.c` where they previously lived as `Read_Temperature_Celsius()` and `Read_VBUS_Voltage()`.

Motor speed, position, and phase currents currently return stub values (0.0f). These will be wired to the motor control library (`MC_GetMecSpeedAverageMotor1()`, encoder reads, shunt ADC outputs) as sensor integration progresses.

### 5.3 `can_telemetry.h` / `can_telemetry.c`

The scheduled broadcast system. Key components:

**Telemetry table** — an array of `TelemetryEntry` structs, one per signal. Each entry defines the signal name, `ReadSpec` value, transmission period in ms, DLC, a pack function pointer, and an enabled flag. The table is the single source of truth for what gets broadcast and how often:

| Signal | ReadSpec | Period | Stagger offset |
|--------|----------|--------|----------------|
| MotorSpeed | READ_SPEED | 100 ms | 0 ms |
| Position | READ_POSITION | 100 ms | 33 ms |
| BusVoltage | READ_VOLTAGE | 200 ms | 10 ms |
| PhaseCurrents | READ_CURRENT | 100 ms | 66 ms |
| Temperature | READ_TEMPERATURE | 1000 ms | 500 ms |
| Status | READ_CURRENT_STATE | 500 ms | 250 ms |

**Countdown staggering** — signals that share the same period have different initial countdown values so they don't all fire on the same tick. This spreads TX load and avoids bursting the 3-element TX FIFO. The stagger offsets are chosen as approximately `period / (number of signals at that rate)`.

**Pack functions** — each reads from `ESC_Sensors_Get()` and copies the relevant values into the TX buffer using `memcpy`. Single float signals produce 4 bytes; phase currents produce 8 bytes (Iu at bytes 0–3, Iv at bytes 4–7); status produces 4 bytes (fault flags, motor state, control mode, reserved).

**`Telemetry_Tick_1ms()`** — the scheduler entry point. For each enabled entry, decrements the countdown. When it reaches zero, reloads with `period_ms - 1` (the -1 accounts for tick-zero of the new cycle), calls the pack function, builds the CAN ID via `CAN_BuildID()`, populates the `FDCAN_TxHeaderTypeDef`, and queues the frame with `HAL_FDCAN_AddMessageToTxFifoQ()`. If the TX FIFO is full, the frame is silently skipped — the next cycle carries fresh data.

**Runtime control** — `Telemetry_SetEnabled()` and `Telemetry_SetPeriod()` allow enabling/disabling signals and changing their rates at runtime, for example in response to a CAN command from the master.

### 5.4 `CAN_processing_v2.h` / `CAN_processing_v2.c`

The request-response path. Handles incoming frames from the master and generates replies.

**RX parsing (`CAN_Parse_MSG()`)** — entry point called when a CAN frame arrives. Extracts the 11-bit ID using `CAN_Get*()` inline functions from `can_common.h`. Immediately discards frames with Sender = SLAVE (other ESCs). Routes to single-motor or multi-motor dispatch based on the MotorConfig bit. For single-motor frames, checks that DeviceID matches this ESC's `ESC_ID` before processing.

**Command dispatch** — `Process_Single_ESC_Command()` extracts a float from the payload and branches on `commandType`. Run commands go to `Handle_Run_Command()` (currently a stub with debug prints). Read commands go to `Handle_Read_Command()`.

**Read handler (`Handle_Read_Command()`)** — switches on `ReadSpec` and reads the appropriate value from the `ESC_SensorData` cache. Position is converted from radians to degrees via `radToDegrees()` (defined in `main.h`). The response is sent back via `sendCANResponse()`.

**Response TX (`sendCANResponse()`)** — builds the 11-bit response ID using `CAN_BuildID()` with Sender = SLAVE. The response mirrors the received frame's Action, Config, MotorType, and Spec fields. The float payload is packed into bytes 0–3 of a 64-byte zero-initialized buffer. Frames are CAN FD with bit rate switching (arbitration at 500 kbps, data at 2 Mbps), 8-byte DLC.

**Data extraction utilities** — `SingleExtractFloatFromCAN()` (4-byte float from bytes 0–3), `extract_multiple_speeds()` (2-byte signed int16 at offset `ESC_ID * 2`), `extract_multiple_positions_arm()` (half-float at offset `(ESC_ID - 8) * 2`, converted via `half_to_float()`). The half-float converter handles subnormals, infinities, and NaN per IEEE 754 half-precision.

---

## 6. Integration in `main.c`

### FDCAN initialization

After CubeMX code generation, three manual steps are required before CAN is operational:

```c
/* 1. Configure receive filter — without this, all frames are rejected */
FDCAN_FilterTypeDef sFilterConfig;
sFilterConfig.IdType       = FDCAN_STANDARD_ID;
sFilterConfig.FilterIndex  = 0;
sFilterConfig.FilterType   = FDCAN_FILTER_MASK;
sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
sFilterConfig.FilterID1    = 0x000;
sFilterConfig.FilterID2    = 0x000;  /* accept all IDs */
HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig);

/* 2. Enable RX interrupt */
HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);

/* 3. Start the peripheral */
HAL_FDCAN_Start(&hfdcan1);
```

### RX interrupt callback

The FDCAN RX interrupt copies the frame into shared buffers and sets a flag. CAN parsing is not done inside the ISR to avoid blocking.

```c
volatile uint8_t can_rx_flag = 0;
FDCAN_RxHeaderTypeDef can_rx_header;
uint8_t can_rx_data[64];

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if (RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE)
    {
        HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &can_rx_header, can_rx_data);
        can_rx_flag = 1;
    }
}
```

### 1 kHz tick handler

The sensor update, telemetry scheduler, and CAN RX processing all run from the same 1 kHz tick, in this order:

```c
/* Called from TIM6 ISR or from main loop via a 1 ms flag */
void App_Tick_1ms(void)
{
    /* 1. Refresh the sensor cache — must happen first so both
     *    telemetry and request-response see fresh data */
    ESC_Sensors_Update();

    /* 2. Run the telemetry scheduler — may queue 0 or more TX frames */
    Telemetry_Tick_1ms();

    /* 3. Process any incoming CAN frame */
    if (can_rx_flag)
    {
        can_rx_flag = 0;
        CAN_Parse_MSG(&can_rx_header, can_rx_data);
    }
}
```

The ordering matters: sensors are read once, then both the scheduled broadcasts and any on-demand read response use the same cached values for that tick.

---

## 7. TX frame configuration

All ESC-originated CAN FD frames (both telemetry and response) use identical TX settings:

| Field | Value | Meaning |
|-------|-------|---------|
| IdType | FDCAN_STANDARD_ID | 11-bit ID |
| TxFrameType | FDCAN_DATA_FRAME | Data (not remote) |
| DataLength | FDCAN_DLC_BYTES_8 | 8 bytes (expandable) |
| ErrorStateIndicator | FDCAN_ESI_ACTIVE | No error |
| BitRateSwitch | FDCAN_BRS_ON | Data phase at 2 Mbps |
| FDFormat | FDCAN_FD_CAN | CAN FD framing |
| TxEventFifoControl | FDCAN_NO_TX_EVENTS | No TX event logging |

The TX data buffer is always 64 bytes, zero-initialized. Unused bytes beyond the DLC remain zero. The DLC can be bumped to `FDCAN_DLC_BYTES_12`, `_16`, `_20`, `_24`, `_32`, `_48`, or `_64` when payloads grow.

---

## 8. Bus load and coexistence

With the default telemetry table, the ESC transmits approximately:

- 3 signals at 100 ms (speed, position, currents) = 30 frames/sec
- 1 signal at 200 ms (voltage) = 5 frames/sec
- 1 signal at 500 ms (status) = 2 frames/sec
- 1 signal at 1000 ms (temperature) = 1 frame/sec

That totals about 38 frames/sec per ESC. Each 8-byte CAN FD frame with BRS is roughly 80 bits on the bus (including overhead), which at 500 kbps nominal + 2 Mbps data amounts to well under 1% bus utilization per ESC.

The request-response path adds frames only when the master explicitly asks. Since telemetry already provides continuous data, the master rarely needs to poll. If both fire on the same tick (a telemetry broadcast and a solicited response for the same ReadSpec), the TX FIFO (3 elements) handles both. If the FIFO fills, the telemetry scheduler silently drops its frame — the next broadcast cycle carries fresh data. The response path calls `Error_Handler()` on TX failure, which is appropriate since a solicited response failing indicates a more serious bus or FIFO problem.

---

## 9. How to add a new telemetry signal

1. Add a new value to the `ReadSpec` enum in `can_common.h` (use one of the remaining slots, or expand the 3-bit range if needed).
2. Add the corresponding sensor field to `ESC_SensorData` in `esc_sensors.h` and populate it in `ESC_Sensors_Update()`.
3. Write a `pack_xxx()` function in `can_telemetry.c` that reads from the sensor cache and copies into `tx_data[]`.
4. Add a row to `telemetry_table[]` with the desired period, stagger offset, DLC, and pack function.
5. Add a case in `Handle_Read_Command()` in `CAN_processing_v2.c` so the master can also request the value on demand.

---

## 10. Design decisions and rationale

**Why a shared sensor cache instead of direct reads?** Two paths reading the same ADC independently can get different values on the same tick, trigger duplicate conversions (wasting CPU and potentially conflicting with the ADC state machine), and scatter hardware-access code across unrelated modules. A single update-once-read-many pattern keeps values consistent, saves CPU, and puts all ADC/sensor code in one maintainable place.

**Why a single `ReadSpec` enum instead of separate enums?** The telemetry scheduler and request-response handler produce frames with the same CAN ID format. If they used different enums with values that drifted apart, the master would have to maintain two decoding tables. A single enum enforced at compile time makes the protocol self-consistent.

**Why `CAN_BuildID()` instead of inline bit manipulation?** The original code had two independent ID construction paths (one in `sendCANResponse()`, one in `Telemetry_BuildCANID()`) with manually shifted fields. Any change to the ID layout required finding and updating both. A single builder function eliminates this class of bug entirely.

**Why silent drop on telemetry TX failure?** Telemetry is periodic and latest-value. If a frame can't be queued, the data isn't lost — it's superseded by the next cycle's value in a few hundred milliseconds at most. Calling `Error_Handler()` for a full TX FIFO would reset the ESC over a transient condition that resolves itself.
