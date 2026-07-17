# ArmController API — Documentation

## 1. Overview

The `ArmController` module provides a high-level Python API for controlling the robotic arm over CAN, designed to be called from external robot systems, test scripts, or automation pipelines. It wraps the existing `CANCommander` with arm-specific conveniences: named joints, coordinated multi-joint moves, sequencing helpers, and passive telemetry reads from the logger database.

The key design requirement is **concurrent access** — the API and the dashboard GUI must both work at the same time. The GUI lets operators observe telemetry visually, while the API lets the parent robot system send setpoints programmatically. Both paths share the same CAN bus without conflict.

---

## 2. Architecture

```
 Your robot system                   Dashboard GUI (browser)
     │                                     │
     │  import ArmController               │  HTTP / SSE
     ▼                                     ▼
 ArmController                        Flask app
 (arm_controller.py)                  (can_dashboard.py)
     │                                     │
     │  Direct Python calls                │  POST /api/cmd/*
     ▼                                     ▼
 CANCommander  ◄────── both use ──────►  CANCommander
     │                                     │
     └──────────►  CmdClient (TCP)  ◄──────┘
                       │
                  CmdServer (:5555)
                       │
                  can_logger.py
                       │
                  CANable → CAN FD bus → ESC(s)
```

### Why this works without conflicts

The `CmdServer` inside `can_logger.py` accepts multiple simultaneous TCP connections, each handled on its own thread. The `ArmController` creates its own `CmdClient` that connects independently of the dashboard's `CmdClient`. Both send JSON command packets over TCP, the server serializes access to `bus.send()`, and the logger captures everything (TX and RX) to the SQLite database. The GUI continues showing live telemetry via SSE regardless of whether the robot system is also sending commands.

Neither the API nor the GUI ever opens the serial port (CANable) directly — only `can_logger.py` does.

---

## 3. File placement

Place `arm_controller.py` alongside or inside your `esc_can/` package:

```
esc_can/
    __init__.py
    protocol.py        ← CAN ID encoding, enums, payload builders
    commander.py       ← CANCommander (encodes + sends via TCP)
    cmd_server.py      ← CmdServer / CmdClient (TCP relay)
    datalogger.py      ← SQLite logger
scripts/
    can_logger.py      ← owns the CAN bus, runs CmdServer
    can_dashboard.py   ← Flask GUI, reads DB + sends via TCP
arm_controller.py      ← this file (or inside esc_can/)
```

The module tries `from esc_can.protocol import ...` first, then falls back to flat imports (`from protocol import ...`) if the package isn't installed.

---

## 4. Prerequisites

Before using the API, the logger must be running:

```bash
# Terminal 1 — start the logger (owns the CAN bus + command server)
python scripts/can_logger.py --port COM4 --db can_log.db

# Terminal 2 — optionally start the dashboard for visual monitoring
python scripts/can_dashboard.py --db can_log.db

# Terminal 3 (or your robot process) — use the API
python my_robot_script.py
```

The `--cmd-port` must match between logger and API (default 5555 for both).

---

## 5. Quick start

```python
from arm_controller import ArmController

# Connect to the CmdServer
arm = ArmController(cmd_port=5555)
arm.wait_for_connection(timeout=10)

# Move individual joints (S-curve planner path)
arm.move_joint("waist", degrees=45.0)
arm.move_joint("shoulder", degrees=-20.0)

# Velocity control (jerk-limited filter path)
arm.jog_joint("elbow", rad_per_sec=0.3)
arm.jog_stop("elbow")         # smooth deceleration

# Emergency stop all joints
arm.stop_all()

# Read telemetry from the logger's database (no CAN traffic)
state = arm.get_telemetry_from_db()
print(state.joints["waist"].position_deg)

arm.close()
```

---

## 6. Joint definitions

The API uses named joints that map to firmware device IDs and motor types:

| Joint name | Device ID | Motor type | Firmware enum |
|------------|-----------|------------|---------------|
| `waist` | 8 | STEERING | `MotorID.WAIST` |
| `shoulder` | 9 | STEERING | `MotorID.SHOULDER` |
| `elbow` | 10 | STEERING | `MotorID.ELBOW` |

All joint parameters accept a string name (`"waist"`), a `Joint` enum value (`Joint.WAIST`), or a raw integer device ID (`8`).

---

## 7. API reference

### 7.1 Connection

| Method / Property | Description |
|---|---|
| `ArmController(cmd_host, cmd_port, is_fd)` | Constructor. Connects to CmdServer. |
| `arm.is_connected` | `True` if CmdServer is reachable via TCP. |
| `arm.wait_for_connection(timeout)` | Block until connected or timeout. Returns `bool`. |
| `arm.tx_count` | Total CAN frames sent by this instance. |
| `arm.close()` | Close the TCP connection. |

The controller supports context manager usage:

```python
with ArmController() as arm:
    arm.move_joint("waist", 45.0)
```

### 7.2 Position control (S-curve planner)

These commands send `RUN_POSITION` frames. The firmware converts degrees to radians and feeds the value to `buildNewCurve()`, producing a jerk-limited S-curve trajectory with constraints `a_max=100 rad/s²`, `j_max=20 rad/s³`, `v_max=1.4 rad/s`.

| Method | Description |
|---|---|
| `arm.move_joint(joint, degrees)` | Move one joint to an absolute position. |
| `arm.move_joints(waist=45, shoulder=-20, elbow=90)` | Move multiple joints (commands sent back-to-back). |
| `arm.home()` | Move all joints to 0°. |
| `arm.move_and_wait(joint, degrees, settle_time=2.0)` | Move and sleep for the trajectory to complete. |

### 7.3 Velocity control (jerk-limited filter)

These commands send `RUN_SPEED` frames. The firmware passes the value directly to `velCtrlSetDemand()`, which produces a smooth velocity ramp.

| Method | Description |
|---|---|
| `arm.jog_joint(joint, rad_per_sec)` | Set continuous velocity. Positive/negative for direction. |
| `arm.jog_stop(joint)` | Send 0.0 velocity for smooth deceleration. |

### 7.4 Safety

| Method | Description |
|---|---|
| `arm.stop(joint)` | Emergency stop one joint (`RUN_STOP`). |
| `arm.stop_all()` | Emergency stop all joints. |
| `arm.acknowledge_faults(joint)` | Clear fault flags on a joint. |
| `arm.calibrate(joint)` | Start calibration sequence. |

### 7.5 Read requests

These send read frames on the bus. The ESC responds via its normal telemetry/response path, and the response appears in the logger database and dashboard.

| Method | Description |
|---|---|
| `arm.ping(joint)` | Ping request (ESC responds with 69.0). |
| `arm.read_position(joint)` | Request current position. |
| `arm.read_speed(joint)` | Request current speed. |
| `arm.read_voltage(joint)` | Request bus voltage. |
| `arm.read_temperature(joint)` | Request board temperature. |
| `arm.read(joint, spec)` | Generic read with any `ReadSpec` or string. |

### 7.6 Telemetry (passive database read)

```python
state = arm.get_telemetry_from_db(db_path="can_log.db")
```

Returns an `ArmState` object with a `JointState` for each joint containing the latest `position_deg`, `speed_rad_s`, `temperature_c`, `voltage_v`, and `timestamp`. This reads directly from the SQLite database that `can_logger.py` writes to — no CAN frames are sent.

### 7.7 Sequencing

For scripted test routines:

```python
arm.run_sequence([
    {"joint": "waist",    "degrees": 45.0,  "wait": 2.0},
    {"joint": "shoulder", "degrees": -20.0, "wait": 2.0},
    {"joint": "elbow",    "degrees": 90.0,  "wait": 2.0},
    {"command": "stop",   "joint": "waist", "wait": 0.5},
    {"joint": "waist",    "degrees": 0.0,   "wait": 2.0},
])
```

Each step dict supports keys: `joint` (required), `degrees` (for moves), `rad_per_sec` (for jogs), `wait` (seconds to pause after), and `command` (`"move"`, `"jog"`, or `"stop"`, default `"move"`).

---

## 8. Integration example

A typical robot system integration looks like this:

```python
from arm_controller import ArmController
import time

class RobotSystem:
    def __init__(self):
        self.arm = ArmController(cmd_port=5555)
        if not self.arm.wait_for_connection(timeout=10):
            raise RuntimeError("CAN logger not running")

    def pick_and_place(self, pick_pos, place_pos):
        """Example pick-and-place routine."""
        # Move to pick position
        self.arm.move_joints(
            waist=pick_pos["waist"],
            shoulder=pick_pos["shoulder"],
            elbow=pick_pos["elbow"],
        )
        time.sleep(3.0)  # wait for trajectory

        # Check we arrived
        state = self.arm.get_telemetry_from_db()
        actual = state.joints["waist"].position_deg
        print(f"Waist at {actual:.1f}° (target {pick_pos['waist']:.1f}°)")

        # ... activate gripper ...

        # Move to place position
        self.arm.move_joints(
            waist=place_pos["waist"],
            shoulder=place_pos["shoulder"],
            elbow=place_pos["elbow"],
        )
        time.sleep(3.0)

    def shutdown(self):
        self.arm.stop_all()
        self.arm.close()
```

---

## 9. CAN frame details

For reference, here is how the API's method calls map to CAN frames on the bus.

### CAN ID encoding (11-bit standard)

```
Bit:  [10]    [9]     [8]       [7]        [6:4]          [3:0]
      Sender  Action  Config    MotorType  Specification  DeviceID
```

| Field | Value set by ArmController |
|-------|---------------------------|
| Sender | 0 (MASTER) |
| Action | 0 (RUN) for move/jog/stop, 1 (READ) for read/ping |
| MotorConfig | 1 (SINGLE) |
| MotorType | 1 (STEERING) for all arm joints |
| Specification | RunSpec or ReadSpec enum value |
| DeviceID | 8 (waist), 9 (shoulder), 10 (elbow) |

### Example: `arm.move_joint("shoulder", degrees=45.0)`

```
arb_id = 0 | 0 | 1 | 1 | 011 | 1001  =  0x1B9
         S   A   C   T   POSITION  dev=9

payload = struct.pack("<f", 45.0) + b"\x00"*4
        = [0x00, 0x00, 0x34, 0x42, 0x00, 0x00, 0x00, 0x00]
```

The firmware receives this, calls `degreesToRad(45.0)`, and feeds it to the S-curve planner.

---

## 10. Troubleshooting

**`ConnectionError: Cannot reach CmdServer`** — `can_logger.py` is not running, or the `--cmd-port` doesn't match. Start the logger first.

**Commands sent but no motion** — verify the ESC's `ESC_ID` matches the joint's device ID (waist=8, shoulder=9, elbow=10). Check CAN termination resistors.

**`get_telemetry_from_db()` returns all `None`** — the database doesn't exist yet or no telemetry has been received. Ensure the ESCs are powered and broadcasting.

**GUI stops updating when API sends commands** — this should not happen. Both share the CmdServer without conflict. If it does, check that only one `can_logger.py` instance is running.

**Import errors** — make sure `protocol.py`, `commander.py`, and `cmd_server.py` are importable, either as `esc_can.*` or from the same directory.
