# CAN_processing_v2 — Reference Notes

## What this is

`CAN_processing_v2.c` and `CAN_processing_v2.h` are the CAN FD protocol layer for the B-G431B-ESC1. They handle everything between a raw CAN frame arriving in hardware and a command being dispatched to application code. Motor control logic is **not** included — command handlers are currently stubbed out with debug prints and dummy responses.

These files were derived from the original `CAN_processing.c/.h` which combined CAN protocol handling with motor control (speed, position, calibration, limit switches, etc.). The v2 files strip all of that out so CAN communication can be tested independently.

## What was kept from the original

- The full 11-bit standard CAN ID encoding/decoding scheme
- `CAN_Parse_MSG()` — the main entry point that parses an incoming frame
- `sendCANResponse()` — builds and transmits a response frame back to the master
- All data extraction utilities: `SingleExtractFloatFromCAN()`, `extract_multiple_speeds()`, `extract_multiple_positions_arm()`, `half_to_float()`
- All enums (`Transmitter`, `Action`, `MotorConfig`, `MotorType`, `RunSpec`, `ReadSpec`, `MotorID`)
- The `ParsedCANID` struct

## What was removed

- All motor control includes (`mc_interface.h`, `encoder_speed_pos_fdbk.h`, `pmsm_motor_parameters.h`, etc.)
- All motor control functions: `ControlSingleMotor`, `runSingleMotorV2`, `IdleSingleMotor`, `StartSingleMotor`, `safeStopMotor`, `speedCheck`, `clippingCheck`, `computeRampTimeMs`, all position control, calibration sequences, `waitForStop`, `waitForRun`, gear ratio conversions, limit switch handlers
- All motor-related variables (ramp parameters, gear ratios, joint limits, startup watchdog, etc.)
- `Read_Temperature_Celsius()` (ADC-specific, will be re-added when sensor reads are implemented)
- The `StartWatchdog` struct and associated logic
- Calibration state variables (`isCalibrating`, `isCalibrated`, `switch1_opened`, `switch2_opened`, etc.)

## What was added

- `Handle_Run_Command()` — stub that logs the received `runSpec` and payload value via UART debug prints. Does not actuate anything.
- `Handle_Read_Command()` — stub that logs the received `readSpec`, then sends back a dummy float value via `sendCANResponse()` so the full RX-parse-TX pipeline can be verified.
- A `USE_UART_DEBUG` compile-time guard. If `uart_debugging.h` is available and the macro is defined, debug prints work. Otherwise they compile away to nothing.

## CAN ID bit layout (unchanged from original)

The 11-bit standard ID is structured as:

```
Bit:  [10]   [9]     [8]       [7]        [6:4]          [3:0]
      Sender Action  Config    MotorType  Specification  DeviceID
```

| Bit(s) | Field         | Values                                      |
|--------|---------------|---------------------------------------------|
| 10     | Sender        | 0 = Master, 1 = Slave/ESC                   |
| 9      | Action        | 0 = Run, 1 = Read                           |
| 8      | MotorConfig   | 0 = Multiple motors, 1 = Single motor       |
| 7      | MotorType     | 0 = Drive, 1 = Steering                     |
| 6:4    | Specification | RunSpec (if Action=Run) or ReadSpec (if Read)|
| 3:0    | Device ID     | 0-15, matched against ESC_ID                |

When the ESC responds, it mirrors the received ID but sets bit 10 to 1 (SLAVE), so other ESCs ignore the response.

## TX configuration (CAN FD with BRS)

`sendCANResponse()` transmits using CAN FD framing:

- `FDFormat = FDCAN_FD_CAN` — the frame is CAN FD, not classic
- `BitRateSwitch = FDCAN_BRS_ON` — arbitration at 500 kbps, data phase at 2 Mbps
- `DataLength = FDCAN_DLC_BYTES_8` — only 8 bytes used currently (4 for the float payload, 4 zeros). Can be bumped to 12/16/20/24/32/48/64 when the payload grows.
- The TX buffer is 64 bytes, zero-initialized, so unused bytes are always 0.

The 11-bit standard ID is still used (not extended 29-bit). This is compatible with CAN FD.

## Data extraction

Three extraction methods exist for different payload formats:

- **Single float (4 bytes):** `SingleExtractFloatFromCAN()` — `memcpy` from bytes 0-3 into a float. Used for single-motor commands.
- **Multiple int16 speeds (2 bytes per ESC):** `extract_multiple_speeds()` — reads 2 bytes at offset `ESC_ID * 2`, little-endian.
- **Multiple half-float positions (2 bytes per ESC):** `extract_multiple_positions_arm()` — reads 2 bytes at offset `(ESC_ID - 8) * 2`, converts IEEE-754 half-precision to float via `half_to_float()`. Used for arm joint commands where ESC_ID is 8 (waist), 9 (shoulder), or 10 (elbow).

## How it fits into main.c

The intended integration (from the FDCAN setup step) is:

1. The FDCAN RX FIFO0 interrupt fires and sets a flag (`can_rx_flag = 1`) after copying the message into `can_rx_header` / `can_rx_data`.
2. A 1 kHz TIM6 interrupt checks the flag and calls `CAN_Parse_MSG(&can_rx_header, can_rx_data)`.
3. `CAN_Parse_MSG` decodes the ID, routes to single or multi dispatch, which calls the stub handlers.
4. For read commands, the stub sends a dummy response back over CAN FD.

## Next steps

- Test the RX-parse-TX pipeline end-to-end using a CANable adapter
- Replace stub handlers with real motor control dispatch
- Consider migrating the ID scheme to take advantage of CAN FD features (larger payloads, different encoding)
