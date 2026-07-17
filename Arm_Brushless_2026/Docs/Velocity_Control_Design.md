# Velocity Control Module — Design and Implementation

## 1. Purpose

The velocity control module provides jerk-limited velocity tracking for manual (joystick / CAN) control of robotic arm joints. It accepts a demanded velocity over CAN and produces a smooth, continuously advancing position setpoint that the existing PID position loop tracks.

This module exists alongside the S-curve point-to-point planner. The S-curve planner handles "go to angle X" commands where the target position is known. The velocity controller handles "move at speed X" commands where the user controls the rate of motion and the final position is wherever the motor ends up.

---

## 2. Why not use the S-curve planner for velocity control

The S-curve planner requires a known target position to compute switching times, phase durations, and case classification. A joystick provides a continuously changing velocity demand with no fixed target. Synthesizing a fake target position each tick creates problems: constant replanning, frequent triggering of the forced-braking path, and the case classification logic spending most of its time in edge cases. A dedicated velocity filter is simpler, more responsive, and avoids these issues entirely.

---

## 3. Architecture

### File structure

| File | Role |
|------|------|
| `velocity_ctrl.h` | Data structures (`VelCtrlHandle`) and function prototypes |
| `velocity_ctrl.c` | Jerk decision logic, integration, position limits, API functions |
| `vel_test.c` | Simulation test harness with five built-in scenarios |
| `plot_velocity.py` | Python plotter for velocity CSV output |

### Relationship to existing code

The velocity controller writes to the same `VelocityFilter` tracker struct that the S-curve planner uses. The rest of the system (PID loop, encoder feedback, CAN telemetry) sees identical output regardless of which mode is active. On the STM32, the 1 kHz timer ISR would select between the two execution paths based on a mode flag:

```c
if (controlMode == MODE_POSITION) {
    PosCtrl_ISRStep();
} else if (controlMode == MODE_VELOCITY) {
    velCtrl_ISRStep(velCtrl, tracker);
}
```

### Shared constraints

The module uses the same kinematic constraints as the S-curve planner: `A_MAX` (100 rad/s²), `J_MAX` (20 rad/s³), `MAX_VEL` (1.4 rad/s), and `SAMPLING_TIME` (0.001 s). These are defined in `SCurveTrajectory.h` and shared across both modules.

---

## 4. Core algorithm — jerk-limited velocity tracking

### Overview

Each ISR tick, the filter decides what jerk to apply so the commanded velocity tracks the demanded velocity smoothly. The jerk is always one of three values: `+j_max`, `-j_max`, or a proportional value for fine settling. After computing the jerk, the filter integrates `jerk → acceleration → velocity → position` using exact constant-jerk kinematics.

The result is S-shaped velocity ramps — smooth acceleration onset and offset — with no switching-time computation. The filter is fully reactive, making its decision fresh each tick based on the current state.

### The jerk decision

The decision has two regimes: a **dead zone** for fine settling when velocity is near the demand, and a **bang-bang** regime for large velocity errors.

**Dead zone regime** — when `|v_error| < v_deadzone` (where `v_deadzone = 0.5 * j_max * Ts`):

The motor is essentially at the demanded velocity. Instead of using bang-bang braking/acceleration (which causes limit-cycle oscillation), the filter applies proportional jerk to bleed off residual acceleration: `j = -a / Ts`, clamped to `±j_max`. This zeroes the acceleration in exactly one tick. Once acceleration is negligible, jerk is set to zero and acceleration is snapped to zero. The motor holds steady.

**Bang-bang regime** — when `|v_error| >= v_deadzone`:

The filter computes the "braking velocity" — the velocity the motor would reach if it started decelerating right now (applying jerk to bring acceleration to zero). This is computed from the current velocity and acceleration:

```
t_brake = |a| / j_max
v_brake = v + a * t_brake + 0.5 * j_opposing * t_brake²
```

If `v_brake` would overshoot the demand, the filter applies braking jerk (opposing the current acceleration). Otherwise, it applies jerk in the direction of the velocity error to accelerate toward the demand.

### Integration

After computing the jerk, the filter integrates using exact constant-jerk kinematics:

```
a_new = a + j * Ts                              (clamped to ±a_max)
j_eff = (a_new - a) / Ts                        (effective jerk after clamping)
v_new = v + a * Ts + 0.5 * j_eff * Ts²          (clamped to ±v_max)
p_new = p + v * Ts + 0.5 * a * Ts² + (1/6) * j_eff * Ts³
```

The effective jerk `j_eff` is recomputed after the acceleration clamp to ensure the velocity and position integrals remain consistent with the actual acceleration change.

---

## 5. Position limits

### Purpose

Optional software position limits prevent the arm from hitting mechanical hard stops. When enabled, the filter overrides the CAN demand and decelerates the motor before reaching a limit boundary, regardless of what the user is commanding.

### How it works

Each tick, if the motor has meaningful velocity (above a small dead zone), the filter computes the stopping distance from the current state using `computeStoppingDistance`. This accounts for any acceleration that would need to be ramped down before braking can begin.

If the motor is moving toward the upper limit and `position + stopping_distance >= pos_max`, the demand is overridden to zero. The same logic applies symmetrically for the lower limit. Additionally, if the motor is already at or past a limit, any demand that would push it further past is clamped to zero.

The position itself is also hard-clamped after integration as a safety backstop.

### Stopping distance computation

The stopping distance has two components:

1. **Pre-braking distance** — if acceleration is in the same direction as velocity (adding to speed), the motor must first ramp acceleration down to zero before it can start braking. The velocity gained and distance traveled during this ramp-down are computed from constant-jerk kinematics.

2. **Braking distance** — the distance to decelerate from the effective velocity to zero under `a_max` and `j_max` constraints. This uses the same formula as the S-curve planner's `computeDecelDistance`: for high velocities where `a_max` is reached, `d = (v/2)(v/a_max + a_max/j_max)`; for low velocities in the jerk-limited regime, `d = v * sqrt(v/j_max)`.

---

## 6. Dead zone tuning

The velocity dead zone (`v_deadzone = 0.5 * j_max * Ts`) is the critical parameter for settling behavior. It controls the tradeoff between settling precision and oscillation avoidance.

**Too narrow:** The motor brakes past the demand, overshoots by more than the dead zone width, triggers the opposite braking, and enters a limit cycle — alternating ±j_max every few ticks indefinitely. This was the original bug with `v_deadzone = j_max * Ts²` (0.00002 rad/s).

**Too wide:** The motor settles with a larger residual velocity error. At the current setting of `0.5 * j_max * Ts` (0.01 rad/s ≈ 0.57 deg/s), the motor holds steady with up to ~0.6 deg/s residual. On real hardware the PID position loop corrects this; in simulation it's visible but harmless.

**Current value:** 0.01 rad/s at 1 kHz. This provides clean settling with no oscillation across all test scenarios.

---

## 7. API reference

### Initialization

`VelCtrlHandle* velCtrlInit(float_t currentPos)` — allocates and initializes the filter. `currentPos` sets the initial accumulated position (typically the motor's current angle). Returns NULL on allocation failure.

### Demand control

`void velCtrlSetDemand(VelCtrlHandle *h, float_t demandedVel)` — sets the target velocity in rad/s. Called from CAN processing whenever a new velocity command arrives. The value is clamped to `±v_max`. Set to 0 to command a smooth stop.

### Position limits

`void velCtrlSetPositionLimits(VelCtrlHandle *h, float_t minPos, float_t maxPos)` — enables software position limits in radians. The filter will automatically decelerate before reaching either boundary.

`void velCtrlClearPositionLimits(VelCtrlHandle *h)` — disables position limits.

### Mode control

`void velCtrlStart(VelCtrlHandle *h, VelocityFilter *tracker)` — activates velocity control mode. Inherits the motor's current state (position, velocity, acceleration) from the tracker for a seamless transition. Demand is initialized to zero.

`void velCtrlStop(VelCtrlHandle *h)` — requests deactivation by setting demand to zero. The filter will smoothly decelerate on subsequent ticks. The caller should keep calling `velCtrl_ISRStep` and check `velCtrlGetVelocity` until the motor has stopped, then switch to position-hold or S-curve mode.

### ISR execution

`float_t velCtrl_ISRStep(VelCtrlHandle *h, VelocityFilter *tracker)` — called at 1 kHz from the timer interrupt. Computes one tick of the jerk-limited velocity filter and writes the result to the tracker. Returns the new position setpoint.

### State readback

`float_t velCtrlGetVelocity(VelCtrlHandle *h)` — returns current commanded velocity in rad/s.

`float_t velCtrlGetAccel(VelCtrlHandle *h)` — returns current commanded acceleration in rad/s².

`float_t velCtrlGetPosition(VelCtrlHandle *h)` — returns accumulated position setpoint in rad.

---

## 8. Mode switching between S-curve and velocity control

### Switching to velocity mode

1. Call `velCtrlStart(velCtrl, tracker)` — this copies the motor's current state into the velocity filter so there's no discontinuity.
2. Set the ISR mode flag to `MODE_VELOCITY`.
3. The velocity filter now owns the tracker output. CAN velocity commands go through `velCtrlSetDemand`.

### Switching back to S-curve mode

1. Call `velCtrlStop(velCtrl)` — sets demand to zero.
2. Keep ticking until `fabsf(velCtrlGetVelocity(velCtrl)) < threshold` (motor has stopped).
3. Set the ISR mode flag to `MODE_POSITION`.
4. The motor's current position (`tracker->theta`) becomes the S-curve planner's starting point for the next `buildNewCurve` call.

---

## 9. Simulation and testing

### Building

```bash
make vel_test       # or: make (builds both sim and vel_test)
```

### Running

```bash
./vel_test          # runs all five built-in scenarios
make run-vel        # same thing via make
```

### Test scenarios

| Scenario | Description | What it tests |
|----------|-------------|---------------|
| Step | 0 → 0.5 rad/s → stop | Basic ramp-up, cruise, and smooth stop |
| Reversal | +0.8 → -0.8 → stop | Full direction reversal through zero |
| Gradual | Ramp through 0.2/0.5/0.8/1.0 then ease off | Joystick-style progressive speed changes |
| Rapid | ±0.6/0.4/0.8/0.6 quick changes | Stress test with frequent direction changes |
| Limit | Full speed toward ±45° wall | Position limit auto-deceleration |

### Plotting

```bash
python3 plot_velocity.py test/vel_*.csv
```

Produces 4-panel plots (position, velocity with demand overlay, acceleration, jerk) with acceleration-based background shading (blue = accelerating, green = coasting, orange = decelerating). Each scenario gets an individual PNG, and an overlay is created when plotting multiple files.

### CSV output format

```
time_s,position,velocity,acceleration,jerk,demand,cmd_vel
```

All kinematic values are in radians. The Python plotter converts to degrees for display.

---

## 10. STM32 integration checklist

1. Place `velocity_ctrl.c` in `src/` and `velocity_ctrl.h` in `lib/` (or wherever headers live in the firmware project).
2. Call `velCtrlInit(currentEncoderAngle)` once at startup.
3. Optionally call `velCtrlSetPositionLimits` with the arm's mechanical range.
4. Add a mode flag (`controlMode`) and a mode-select branch in the 1 kHz timer ISR.
5. In CAN processing, route velocity commands to `velCtrlSetDemand` and position commands to `buildNewCurve`, setting the mode flag accordingly.
6. On hardware, `tracker->theta` is the position setpoint fed to the PID loop. The PID reads the encoder independently and drives the motor to track this setpoint. The velocity filter doesn't need to know about the encoder — it just produces the reference.
