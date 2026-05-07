# ESP32 Bluetooth Kill Switch

This project implements a two-node Bluetooth kill switch for a rover:

- TX node (handheld/transmitter) reads a local button and sends commands.
- RX node (on rover) drives a transistor output that enables or disables the rover power/control path.

The design is intentionally fail-safe at the RX output:

- Any loss of link forces transistor output LOW (killed state).
- RX boot state starts with transistor output LOW.

## Quick System Model

- Radio transport: ESP32 Classic Bluetooth SPP (BluetoothSerial).
- Protocol: text line commands terminated with newline.
- Command semantics:
  - `OFF` = kill rover (transistor LOW)
  - `ON` = allow rover run (transistor HIGH)

## Pin Map

Defined in `src/hardware_pins.h`:

- `BUTTON_PIN = 0` (TX input)
- `LED_PIN = 2` (status on both TX and RX)
- `TRANSISTOR_PIN = 14` (RX output to kill transistor)
- `BUZZER = 5` (audible feedback)

## Build Role Selection

Role is selected at compile time in `src/hardware_pins.h`:

- `RADIO_MODE_TX` is `0`
- `RADIO_MODE_RX` is `1`
- `RADIO_MODE` chooses which source (`main_tx.cpp` or `main_rx.cpp`) is active.

Flash one ESP32 with TX mode and one ESP32 with RX mode.

## Runtime Behavior

## TX Node (`src/main_tx.cpp`)

1. Starts Bluetooth as master (`ESP32_TX`).
2. Connects to RX server name `ESP32_RX`.
3. Reads button each loop using inverted logic:
   - `button_state = !digitalRead(BUTTON_PIN)`
4. Sends commands with rate limiting (`interval_bt = 500 ms`):
   - Pressed -> sends `OFF` (kill)
   - Released -> sends `ON` (run)
5. LED indicates connection state:
   - HIGH when connected
   - LOW when disconnected/reconnecting

## RX Node (`src/main_rx.cpp`)

1. Starts Bluetooth server as `ESP32_RX`.
2. Initializes outputs:
   - `TRANSISTOR_PIN` LOW at setup (killed)
3. While connected, reads line messages:
   - `ON`:
     - If previously stopped, plays buzzer chirp
     - Sets `TRANSISTOR_PIN` HIGH (run)
   - `OFF`:
     - Sets `TRANSISTOR_PIN` LOW (kill)
     - Sets `rover_stopped = true`
     - Plays buzzer chirp
4. If disconnected:
   - Forces `TRANSISTOR_PIN` LOW (kill)

## RX Transistor Output Details

The `TRANSISTOR_PIN` (GPIO 14) on the RX board is a 3.3V logic output that controls a power transistor or MOSFET:

- **Output meaning:**
  - HIGH (3.3V) = transistor ON → rover enabled/running
  - LOW (0V) = transistor OFF → rover killed/disabled

- **Hardware role:**
  - GPIO 14 drives the gate of a power MOSFET or base of a BJT transistor
  - That transistor is in series with or controls the main power/motor path on the rover
  - The GPIO itself is a signal; the transistor handles the actual current/power switching

- **Fail-safe behavior:**
  - Initialized LOW at RX startup (rover disabled on boot)
  - Forced LOW on BT link loss (rover disabled on disconnect)
  - Only driven HIGH when valid `ON` command received while connected

- **Electrical validation (before field test):**
  - Confirm GPIO 14 is wired to the correct transistor gate/base
  - Test with a safe load first (LED + resistor or oscilloscope) to verify 3.3V logic levels
  - Measure gate voltage when HIGH: should be ~3.3V (sufficient to turn on MOSFET, or ~0.7V gate voltage drop for BJT)
  - Measure gate voltage when LOW: should be ~0V
  - Confirm transistor polarity and wiring: determine if it's low-side (N-channel/NPN pulling to ground) or high-side (P-channel/PNP pulling from supply)
  - Only after verifying safe levels, connect to actual motor/actuator power circuit

## State Truth Table

- TX button pressed -> TX sends `OFF` -> RX transistor LOW -> rover killed.
- TX button released -> TX sends `ON` -> RX transistor HIGH -> rover enabled.
- BT link lost -> RX transistor LOW regardless of previous state.
- RX reboot/power-up -> RX transistor LOW until valid `ON` received.

## Validation Procedure

Use this checklist to validate behavior on bench before field testing.

## Preconditions

- Two ESP32 boards powered (one TX, one RX).
- TX flashed in TX mode, RX flashed in RX mode.
- RX transistor output connected to safe test load first (LED/resistor or instrumented input), not directly to hazardous actuation.
- Serial monitor available at 115200 for both boards.

## Test 1: Pair and Idle Safety

1. Power RX first, then TX.
2. Wait for TX to connect.
3. Observe without pressing button.

Expected:

- TX connected LED ON.
- RX receives `ON` heartbeats and sets transistor HIGH (run) while released.

## Test 2: Kill Command on Press

1. Press and hold TX button.
2. Observe RX output and buzzer.

Expected:

- RX receives `OFF`.
- RX transistor LOW within one command interval (up to ~500 ms).
- RX buzzer chirps on transition to killed state.

## Test 3: Run Command on Release

1. Release TX button.
2. Observe RX output and buzzer.

Expected:

- RX receives `ON`.
- RX transistor HIGH within one command interval (up to ~500 ms).
- If previously killed, RX buzzer chirps once when re-enabling.

## Test 4: Link-Loss Fail-Safe

1. With rover enabled, power off TX (or move out of range / disable TX BT).
2. Observe RX behavior.

Expected:

- RX detects disconnection.
- RX forces transistor LOW and remains killed until link returns and `ON` is received.

## Test 5: RX Boot Fail-Safe

1. Reboot RX while TX is off or disconnected.
2. Measure RX transistor output during boot.

Expected:

- Transistor remains LOW during startup (safe default).

## Test 6: Command Polarity Sanity

1. Verify electrical meaning of transistor HIGH/LOW in your actual rover power stage.
2. Confirm HIGH truly means enable and LOW truly means kill.

Expected:

- No inversion mismatch between logic-level output and physical kill path.

## Suggested Acceptance Criteria

- Kill latency (button press to transistor LOW): <= 600 ms worst case.
- Fail-safe on disconnect: always LOW.
- Fail-safe on RX reboot: always LOW until explicit `ON`.
- No spontaneous transitions to HIGH without valid `ON` command.

## Notes for Validation Logging

Record these for each run:

- Firmware role and git revision.
- Time-to-connect after power-up.
- Measured kill latency.
- Disconnect test result.
- Any missed command or unexpected transition.

If all tests pass repeatedly under realistic RF conditions, the kill switch behavior is validated for integration testing.