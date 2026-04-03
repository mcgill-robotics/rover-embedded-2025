# CAN Debugging and Simulation Testing — Architecture and Integration Guide

## 1. Overview

This document covers the work done to enable end-to-end CAN-based testing and debugging of the B-G431B-ESC1 motor controller firmware, including two main areas:

**Firmware simulation mode** — running the S-curve planner and velocity controller on the STM32 without a physical motor or encoder, so that CAN commands produce simulated motion that can be observed on the dashboard.

**Bidirectional CAN tooling** — extending the Python-side data logger and dashboard to both receive telemetry and send commands from the browser UI, solving the serial port exclusivity problem that prevents two processes from sharing the CANable adapter.

```
 Browser (localhost:5000)
    │
    │  POST /api/cmd/position {degrees: 45}
    ▼
 can_dashboard.py ─── reads ──── can_log.db (SQLite WAL)
    │                                 ▲
    │  TCP :5555                      │ writes
    ▼                                 │
 can_logger.py ──── owns ──── CANable (COM4)
    │                              │
    │  bus.send() + bus.recv()     │ CAN FD 500k/2M
    ▼                              ▼
 B-G431B-ESC1 (firmware in SIM_MODE)
    │
    ├── CAN_Parse_MSG → Handle_Run_Command → planner / velocity ctrl
    ├── PosCtrl_ISRStep / velCtrl_ISRStep → updates tracker
    ├── ESC_Sensors_Update → reads tracker (not encoder)
    └── Telemetry_Tick_1ms → broadcasts position, speed, etc.
```

---

## 2. Firmware simulation mode

### 2.1 The problem

The S-curve planner and velocity controller were developed and tested in a standalone GCC simulation on Linux, producing CSV files for plotting. To test the same code running on the STM32 over CAN — without connecting a motor or encoder — the firmware needs a way to operate in a closed loop where the "encoder" returns the commanded position rather than reading real hardware.

### 2.2 Encoder touch-points in the firmware

There are exactly three places where real encoder data enters the system. All of them go through a single gateway function:

**`Read_Encoder_Position_Rad()` in `main.c`** — the sole hardware encoder function. On real hardware it calls `SPD_GetMecAngle(&ENCODER_M1._Super)` and converts the s16degree result to radians. Every other module that needs encoder data calls this function.

**`updateVelocityFilter()` in `SCurveTrajectory.c`** — called by `PosCtrl_ISRStep()` in the ISR. Reads `Read_Encoder_Position_Rad()` and computes velocity, acceleration, and jerk by finite differencing. This is the feedback path for the position controller.

**`read_encoder_position()` in `esc_sensors.c`** — also calls `Read_Encoder_Position_Rad()`. Feeds the telemetry system so position gets broadcast over CAN.

### 2.3 The simulation switch

Because everything flows through `Read_Encoder_Position_Rad()`, a single compile-time switch enables simulation:

```c
// main.c
float Read_Encoder_Position_Rad(void)
{
#ifdef SIM_MODE
    /* No encoder — return the commanded position from the tracker.
       The planner/velocity controller writes to tracker->theta each
       tick, so reading it back closes the loop without hardware. */
    return ((VelocityFilter *)motorTracker)->theta;
#else
    int32_t mec_angle_s16 = SPD_GetMecAngle(&ENCODER_M1._Super);
    return (float)mec_angle_s16 / (float)RADTOS16;
#endif
}
```

Similarly, `read_motor_speed()` in `esc_sensors.c` needs to return the tracker's velocity instead of zero:

```c
// esc_sensors.c
static float read_motor_speed(void)
{
#ifdef SIM_MODE
    return ((VelocityFilter *)motorTracker)->omega;
#else
    return MC_GetMecSpeedAverageMotor1();
#endif
}
```

And `read_motor_state()` should be stubbed since `MC_GetSTMStateMotor1()` won't produce meaningful output without the FOC loop running.

To enable simulation, add `-DSIM_MODE` to the compiler flags in STM32CubeIDE (Project Properties → C/C++ Build → Settings → MCU GCC Compiler → Preprocessor → Add `SIM_MODE`).

### 2.4 Why this works

In simulation mode, the data flow each 1 kHz tick is:

```
PosCtrl_ISRStep() / velCtrl_ISRStep()
    │
    ├── computes new setpoint, writes to plan->theta
    ├── calls updateVelocityFilter()
    │       └── calls Read_Encoder_Position_Rad()
    │               └── returns motorTracker->theta (the commanded position)
    │       └── differentiates to get omega, accel, jerk
    │
    ▼
ESC_Sensors_Update()
    │
    ├── sensor_cache.position = Read_Encoder_Position_Rad()
    │       └── returns motorTracker->theta (same value)
    ├── sensor_cache.motor_speed = motorTracker->omega
    │
    ▼
Telemetry_Tick_1ms()
    │
    └── broadcasts sensor_cache over CAN
```

The controller writes a setpoint, the "encoder" reads it back, the telemetry broadcasts it. On the dashboard you see the S-curve trajectory or velocity ramp playing out in real time — the position and speed sparklines show the planned motion profile exactly as the simulation CSV plots did.

### 2.5 Switching back to real hardware

Remove the `SIM_MODE` define. `Read_Encoder_Position_Rad()` goes back to calling `SPD_GetMecAngle()`, `read_motor_speed()` calls the motor control library, and the system reads real encoder data. No other code changes are needed — the planner and velocity controller are identical in both modes.

### 2.6 ISR tick ordering

The recommended ordering for the 1 kHz TIM6 ISR is:

```c
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM6) {

        /* 1. Run motion controller — computes setpoint, reads encoder
              via updateVelocityFilter() */
        if (controlMode == MODE_POSITION) {
            PosCtrl_ISRStep();
        } else if (controlMode == MODE_VELOCITY) {
            velCtrl_ISRStep(velCtrl, (VelocityFilter *)motorTracker);
        }

        /* 2. Feed setpoint to PID (on real hardware) */
        float positionSetpointForPID = motorTracker->theta;

        /* 3. Snapshot sensors — reads tracker for motion data,
              ADC for voltage/temperature */
        ESC_Sensors_Update();

        /* 4. Broadcast telemetry from the fresh snapshot */
        Telemetry_Tick_1ms();

        /* 5. Relay CAN RX flag to main loop */
        if (can_rx_flag) {
            can_rx_flag = 0;
            can_rx_ready = 1;
        }
    }
}
```

The key insight is that `ESC_Sensors_Update()` should run after the motion controller, not before. This way telemetry always broadcasts the most current setpoint/position, and the motion controller's call to `updateVelocityFilter()` reads the encoder at the correct point in the cycle.

### 2.7 Known bug in main.c

The main `while(1)` loop calls `Planner_MainStep()` twice (once at line 254 and again at line 263). The duplicate should be removed — it causes replanning to execute twice per loop iteration, which can produce unexpected behavior during mid-motion setpoint changes.

---

## 3. Command relay architecture

### 3.1 The serial port exclusivity problem

The CANable adapter presents as a serial port (e.g. COM4 on Windows, /dev/ttyACM0 on Linux). Only one process can open a serial port at a time. The original architecture had the logger opening the port for receiving, and the dashboard needing the same port for transmitting — but both cannot hold it simultaneously.

If the dashboard opens the port: it can send commands, but the logger can't receive, so no telemetry appears in the database and the dashboard shows nothing.

If the logger opens the port: it captures all telemetry, but the dashboard has no way to transmit commands.

### 3.2 The solution — TCP command relay

The logger process owns the serial port exclusively. It runs a lightweight TCP server (the `CmdServer`) on localhost:5555 alongside its main receive loop. The dashboard sends command requests to this server over TCP, and the server transmits them on the CAN bus and logs both the outgoing command and any incoming response.

```
can_logger.py (owns COM4)
    │
    ├── Main thread:
    │     while not stop:
    │         msg = bus.recv(timeout=0.1)
    │         logger.log_frame(msg, is_rx=True)
    │
    └── CmdServer thread (TCP :5555):
          accepts connections
          reads JSON line: {"cmd":"tx","arb_id":305,"data":"00003442..."}
          calls bus.send(msg)        → frame goes out on CAN
          calls logger.log_raw(...)  → TX frame logged to SQLite
          replies: {"ok":true,"arb_id":"0x131"}
```

The dashboard's `CANCommander` uses a `CmdClient` that maintains a persistent TCP connection to the server. When the user clicks "Send Position" in the browser, the request flows:

```
Browser POST → Flask → CANCommander._transmit()
    → CmdClient.send_frame() → TCP to localhost:5555
    → CmdServer._handle_tx() → bus.send() + logger.log_raw()
    → CAN bus → ESC processes command → ESC sends response
    → bus.recv() in logger main thread → logged to SQLite
    → dashboard SSE stream picks up both TX and RX from DB
```

### 3.3 TCP protocol

The protocol is JSON over TCP, newline-delimited. The server accepts two commands:

**Transmit a frame:**
```json
→ {"cmd":"tx","arb_id":305,"data":"00003442000000000","is_fd":false}\n
← {"ok":true,"arb_id":"0x131","tx_count":1}\n
```

**Health check:**
```json
→ {"cmd":"ping"}\n
← {"ok":true,"status":"running","tx_count":42}\n
```

The `CmdClient` handles connection management, automatic reconnection on failure, and thread-safe access via a lock.

---

## 4. File structure

### 4.1 New and modified files

```
esc_can/
    __init__.py           ← unchanged
    bus.py                ← unchanged
    protocol.py           ← unchanged — CAN ID encoding/decoding
    datalogger.py         ← unchanged — SQLite logger
    cmd_server.py         ← NEW — CmdServer (in logger) + CmdClient (in commander)
    commander.py          ← REWRITTEN — now uses CmdClient TCP, never opens serial

scripts/
    can_logger.py         ← REWRITTEN — starts CmdServer alongside RX loop
    can_dashboard.py      ← MODIFIED — command panel UI, routes via TCP
```

### 4.2 Dependency graph

```
can_dashboard.py (Flask, reads DB, sends commands)
    │
    ├── reads ──→ can_log.db (SQLite, WAL mode)
    │                 ▲
    │                 │ writes
    │                 │
    ├── imports ──→ commander.py
    │                 │
    │                 └── imports ──→ cmd_server.CmdClient
    │                                    │
    │                                    │ TCP :5555
    │                                    ▼
    │               can_logger.py ──→ cmd_server.CmdServer
    │                 │                    │
    │                 ├── imports ──→ datalogger.py ──→ can_log.db
    │                 └── imports ──→ python-can ──→ CANable (COM4)
    │
    └── imports ──→ protocol.py (shared CAN ID encoding)
```

---

## 5. CAN ID encoding verification

The Python-side encoding was verified to produce identical 11-bit CAN IDs to the firmware's `CAN_BuildID()` inline function. Both use the same bit layout:

```
Bit:  [10]    [9]     [8]       [7]        [6:4]          [3:0]
      Sender  Action  Config    MotorType  Specification  DeviceID
```

Verification results for the most common commands sent to ESC device ID 1:

| Command | Sender | Action | Config | Type | Spec | DevID | Arb ID |
|---------|--------|--------|--------|------|------|-------|--------|
| RUN_POSITION | MASTER(0) | RUN(0) | SINGLE(1) | DRIVE(0) | 3 | 1 | `0x131` |
| RUN_SPEED | MASTER(0) | RUN(0) | SINGLE(1) | DRIVE(0) | 2 | 1 | `0x121` |
| RUN_STOP | MASTER(0) | RUN(0) | SINGLE(1) | DRIVE(0) | 0 | 1 | `0x101` |
| READ_PING | MASTER(0) | READ(1) | SINGLE(1) | DRIVE(0) | 6 | 1 | `0x361` |
| READ_POSITION | MASTER(0) | READ(1) | SINGLE(1) | DRIVE(0) | 1 | 1 | `0x311` |
| READ_TEMPERATURE | MASTER(0) | READ(1) | SINGLE(1) | DRIVE(0) | 4 | 1 | `0x341` |

The payload format is a 4-byte little-endian IEEE-754 float at bytes 0–3 of an 8-byte frame, matching `SingleExtractFloatFromCAN()` on the firmware side (which does `memcpy(&value, data, sizeof(float))`).

### 5.1 Position command firmware path

When the dashboard sends a position command (e.g. 45 degrees):

```
Python: encode_can_id(MASTER, RUN, SINGLE, DRIVE, POSITION=3, dev=1) → 0x131
Python: pack_single_float(45.0) → bytes 00 00 34 42 00 00 00 00

Firmware CAN_Parse_MSG(0x131):
    CAN_GetSender(0x131) → 0 (MASTER) — not filtered out
    CAN_GetAction(0x131) → 0 (RUN)
    CAN_GetSpec(0x131)   → 3 (RUN_POSITION)
    CAN_GetConfig(0x131) → 1 (SINGLE)
    CAN_GetDeviceID(0x131) → 1 — matches ESC_ID

Handle_Run_Command case RUN_POSITION:
    information = SingleExtractFloatFromCAN(rxData) → 45.0
    positionSetpoint = degreesToRad(45.0) → 0.7854 rad
    newSetpointDetected = true
    controlMode = MODE_POSITION

Main loop: Planner_MainStep detects newSetpointDetected → buildNewCurve(0.7854)
ISR: PosCtrl_ISRStep executes the S-curve profile
Telemetry: position ramp appears on dashboard sparkline
```

### 5.2 Velocity command firmware path

When the dashboard sends a velocity command (e.g. 0.5 rad/s):

```
Python: encode_can_id(MASTER, RUN, SINGLE, DRIVE, SPEED=2, dev=1) → 0x121
Python: pack_single_float(0.5) → bytes 00 00 00 3F 00 00 00 00

Handle_Run_Command case RUN_SPEED:
    information = SingleExtractFloatFromCAN(rxData) → 0.5
    if controlMode != MODE_VELOCITY:
        velCtrlStart(velCtrl, motorTracker)  — seamless transition
        controlMode = MODE_VELOCITY
    velCtrlSetDemand(velCtrl, 0.5)  — value in rad/s directly

ISR: velCtrl_ISRStep produces jerk-limited velocity ramp
Telemetry: speed ramp and advancing position appear on dashboard
```

---

## 6. Dashboard command panel

### 6.1 UI layout

The dashboard includes a collapsible command panel at the top of the page with three sections:

**Target selector** — device ID (0–15) and motor type (Drive / Steering), shared by all commands.

**Position control** — labeled "S-Curve Planner" to indicate the firmware path. A degrees input field and Send button. The firmware converts degrees to radians internally.

**Velocity control** — labeled "Jerk-Limited Filter". A rad/s input field, Send button, and a "Zero Vel" shortcut that sends 0.0 to smoothly decelerate. The firmware passes the value directly to `velCtrlSetDemand()`.

**Quick actions** — STOP (red, danger styling), Ping (green), and read buttons for Position, Speed, Vbus, Temperature, and Control Mode.

### 6.2 Bus status indicator

A badge in the command panel header shows the connection status:

- **Green "CONNECTED"** — the CmdServer (inside `can_logger.py`) is reachable via TCP. Commands will be transmitted on the bus.
- **Orange "DRY RUN"** or **"CHECKING"** — the CmdServer is not reachable. Either `can_logger.py` is not running, or it's on a different port.

The status is polled every 5 seconds via `GET /api/cmd/status`, which calls `_commander.bus_connected` which attempts a TCP ping to the server.

### 6.3 Flask API endpoints

| Method | Endpoint | Body | Description |
|--------|----------|------|-------------|
| POST | `/api/cmd/position` | `{"device_id":1, "degrees":45.0}` | Send RUN_POSITION |
| POST | `/api/cmd/velocity` | `{"device_id":1, "rad_per_sec":0.5}` | Send RUN_SPEED |
| POST | `/api/cmd/stop` | `{"device_id":1}` | Send RUN_STOP |
| POST | `/api/cmd/ping` | `{"device_id":1}` | Send READ_PING |
| POST | `/api/cmd/read` | `{"device_id":1, "spec":"TEMPERATURE"}` | Send any READ |
| GET | `/api/cmd/status` | — | Commander connection status |

All POST endpoints return `{"ok":true, "arb_id":"0x131", "command":"POSITION", ...}` on success, or `{"ok":false, "error":"..."}` on failure.

---

## 7. Running the system

### 7.1 Simulation testing (no motor)

**Step 1 — Build firmware with SIM_MODE:**

Add `SIM_MODE` to the preprocessor defines in STM32CubeIDE, flash to the ESC.

**Step 2 — Start the logger:**

```bash
python scripts/can_logger.py --port COM4 --db can_log.db
```

Output:
```
CAN FD Logger
  Port       : COM4 (slcan)
  Bitrate    : 500000 / 2000000
  Database   : can_log.db
  Session    : 1
  Cmd server : localhost:5555

Logging... (Ctrl+C to stop)
```

**Step 3 — Start the dashboard:**

```bash
python scripts/can_dashboard.py --db can_log.db
```

Open `http://localhost:5000` in a browser.

**Step 4 — Send commands from the dashboard:**

Set Device ID to match the ESC's `ESC_ID` (default 1). Type 45 in the degrees field and click "Send Position". The ESC will execute the S-curve trajectory and broadcast position/speed telemetry that appears on the dashboard sparklines in real time.

### 7.2 Real hardware testing

Same as above, but build the firmware without `SIM_MODE`. The encoder reads real shaft position, the PID loop drives the motor to track the trajectory setpoint, and telemetry reflects actual motor behavior.

### 7.3 Command-line arguments

**can_logger.py:**

| Argument | Default | Description |
|----------|---------|-------------|
| `--port` | (required) | Serial port for CANable |
| `--interface` | `slcan` | python-can interface type |
| `--bitrate` | `500000` | Nominal bitrate |
| `--data-bitrate` | `2000000` | Data phase bitrate |
| `--db` | `can_log.db` | SQLite database path |
| `--desc` | (empty) | Session description |
| `--cmd-port` | `5555` | TCP port for command server |
| `--no-cmd-server` | (flag) | Disable command server (RX-only) |

**can_dashboard.py:**

| Argument | Default | Description |
|----------|---------|-------------|
| `--db` | `can_log.db` | SQLite database path |
| `--port` | `5000` | HTTP port |
| `--host` | `127.0.0.1` | Bind address |
| `--poll` | `0.20` | SSE poll interval (seconds) |
| `--cmd-port` | `5555` | TCP port of logger's CmdServer |

---

## 8. esc_can.commander API reference

The `CANCommander` class provides the Python API for sending CAN commands. It connects to the logger's CmdServer via TCP and never opens the CAN bus directly.

```python
from esc_can.commander import CANCommander

cmd = CANCommander(cmd_port=5555)

# Position control — value in degrees
# Firmware: degreesToRad(info) → S-curve planner
cmd.set_position(device_id=1, degrees=45.0)

# Velocity control — value in rad/s
# Firmware: velCtrlSetDemand(velCtrl, info) directly
cmd.set_velocity(device_id=1, rad_per_sec=0.5)

# Stop
cmd.stop(device_id=1)

# Read requests — ESC responds with the value
cmd.ping(device_id=1)              # ESC responds with 69.0
cmd.read_position(device_id=1)
cmd.read_speed(device_id=1)
cmd.read_voltage(device_id=1)
cmd.read_temperature(device_id=1)

# Generic read
from esc_can.protocol import ReadSpec
cmd.read(device_id=1, spec=ReadSpec.CONTROL_MODE)

# Status
cmd.bus_connected   # True if CmdServer is reachable
cmd.tx_count        # Total frames sent
```

All methods accept an optional `motor_type=MotorType.STEERING` keyword argument for arm joints (default is `MotorType.DRIVE`).

---

## 9. Latency budget

The total latency from clicking a button in the dashboard to seeing the ESC respond:

| Stage | Latency | Notes |
|-------|---------|-------|
| Browser → Flask | < 5 ms | Local HTTP POST |
| Flask → CmdServer (TCP) | < 2 ms | Localhost socket |
| CmdServer → CAN bus TX | < 1 ms | `bus.send()` |
| CAN bus propagation | < 1 ms | 500 kbps / 2 Mbps |
| ESC firmware processing | 1 ms | Next TIM6 tick |
| ESC telemetry response | 0–100 ms | Depends on telemetry period |
| Logger RX → SQLite flush | 0–250 ms | Batch interval |
| Dashboard SSE poll | 0–200 ms | Poll interval |
| **Total round-trip** | **~250–500 ms** | Button click to sparkline update |

The dominant contributors are the logger's flush interval (250 ms) and the dashboard's SSE poll interval (200 ms). Both can be reduced via command-line arguments for lower latency at the cost of slightly higher CPU usage.

---

## 10. Troubleshooting

**Dashboard shows "CHECKING" / orange badge:**
The CmdServer is not reachable. Make sure `can_logger.py` is running and the `--cmd-port` values match between logger and dashboard (default 5555 for both).

**Commands sent but no telemetry appears:**
The ESC is receiving commands but not responding. Check that the ESC's `ESC_ID` matches the device ID you're sending to. Check that CAN termination resistors are correct (the ESC has a software-controlled 120 Ω resistor on PC14).

**Position telemetry stuck at zero in SIM_MODE:**
`Read_Encoder_Position_Rad()` is not returning `motorTracker->theta`. Verify that `SIM_MODE` is defined in the preprocessor. Also check that `read_motor_speed()` in `esc_sensors.c` is returning `motorTracker->omega` (not the default `0.0f`).

**Duplicate `Planner_MainStep` calls:**
The `main.c` while loop has `Planner_MainStep` called twice. Remove the duplicate (keep only one call inside the `if (controlMode == MODE_POSITION)` block).

**`HAL_Delay(200)` in the main loop:**
This 200 ms delay throttles the main loop to ~5 Hz, which means deferred replans (`tooFastPending`, `wanderReplanPending`) can take up to 200 ms to service. For real motion testing, remove or reduce this delay. Use a counter-based LED toggle instead.
