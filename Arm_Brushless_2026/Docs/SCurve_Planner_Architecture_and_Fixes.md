# S-Curve Trajectory Planner — Architecture and Debugging Record

## 1. Project overview

This project implements an online (real-time) jerk-limited S-curve trajectory generator for controlling robotic arm joints on B-G431B-ESC1 boards (STM32G431, Cortex-M4). Position setpoints arrive over CAN FD. The planner produces smooth, jerk-limited motion profiles that can be updated mid-motion when a new setpoint arrives.

The algorithm is based on the paper "An On-line Reference-Trajectory Generator for Smooth Motion of Impulse-Controlled Industrial Manipulators" by Steven Liu (AMC 2002, IEEE).

The code currently lives in a standalone simulation repo that compiles with GCC on Linux and produces CSV logs for plotting.

---

## 2. File structure and responsibilities

### SCurveTrajectory.h

Defines all shared data structures and constants.

`PosCtrlHandle` stores the full state of one trajectory plan: constraint limits (`a_max`, `v_max`, `j_max`), the eight absolute phase-switch times (`profileSwitchingTimes[0..7]`), the current phase (0=idle, 1–7), execution and replan flags (`isTrajExecuting`, `tooFastPending`, `wanderReplanPending`, `isWandering`, `isPastTooFast`), motion direction (`dir`), and the planned position (`theta`).

`VelocityFilter` tracks the actual motor state: position, velocity, acceleration, jerk, and filter parameters. In simulation the ISR writes these directly; on real hardware they come from encoder derivatives.

Constants: `A_MAX = 100 rad/s²`, `J_MAX = 20 rad/s³`, `MAX_VEL = 1.4 rad/s`, `SAMPLING_TIME = 0.001 s` (1 kHz ISR).

### SCurveTrajectory.c

Initialization and low-level utilities. `STrajectoryInit` allocates a `PosCtrlHandle` with the runtime velocity limit. `velocityFilterInit` allocates a zeroed `VelocityFilter`. `selectJerk` maps the current phase (1–7) to the correct jerk sign per the paper's eq (5). Helper functions handle degree/radian conversion and the simulated encoder stub.

### planner.h

Declares the shared variables and function prototypes that link `planner.c` to `main.c`. Defines the `virtualHistory` struct (virtual initial velocity and position) and the `switchingTimes` struct (the eight phase transition times plus the wandering flag).

### planner.c

The core planning and execution engine. Contains all trajectory computation, the ISR step function, and the replanning logic.

Key functions:

- `plannerInit` — allocates both plan buffers and the motor tracker.
- `buildNewCurve` — entry point when a new setpoint arrives. Decides whether to replan immediately or defer via forced braking.
- `determineSwitchingTimes` — classifies the trajectory into cases (e, c, b, a, d) and computes the seven phase durations.
- `decellerate2Stop` — computes a braking-only profile (phases 5–7) for wandering cases.
- `calculateNewRamp` — populates the inactive plan buffer with switching times, direction, and flags, then raises `plan_ready`.
- `PosCtrl_ISRStep` — the 1 kHz interrupt handler. Swaps plans, runs forced braking if needed, or executes the normal boundary-splitting trajectory.

### main.c

Simulation harness. Provides `Planner_MainStep` (the foreground service step that checks for new commands, wandering replans, and forced-braking resolution), the `runScenario` function that drives the sim loop, and six built-in test scenarios plus an interactive mode.

---

## 3. Execution architecture

### Double-buffered plan swapping

Two `PosCtrlHandle` objects live in `paths_planned[2]`. One is "active" (read by the ISR), one is "inactive" (written by the planner). When `plan_ready` is set, the ISR swaps the indices on the next tick. This avoids race conditions between the foreground planner and the 1 kHz ISR.

### ISR / main loop split

The ISR (`PosCtrl_ISRStep`) only executes motion — it selects the jerk for the current phase, integrates position/velocity/acceleration, and updates the tracker. It raises flags (`wanderReplanPending`, `tooFastPending`) when the foreground needs to make decisions.

The main loop (`Planner_MainStep`) handles replanning: it checks for new setpoint commands, wandering replan requests, and forced-braking resolution. All calls to `buildNewCurve` and `calculateNewRamp` happen in the foreground.

### Boundary splitting

Each ISR tick may cross one or more phase-switching boundaries. Rather than assuming constant jerk over the full 1 ms, the ISR subdivides the tick at each boundary and integrates each sub-interval with the correct jerk. This eliminates the small endpoint errors that accumulate from naive fixed-timestep integration.

---

## 4. Seven-phase motion profile

The trajectory is controlled entirely by a jerk function `j(t)` that takes only three values: `+j_max`, `0`, or `-j_max`. Position, velocity, and acceleration are computed by successive integration.

| Phase | Jerk     | Description                 |
|-------|----------|-----------------------------|
| I     | +j_max   | Acceleration increasing     |
| II    | 0        | Constant max acceleration   |
| III   | -j_max   | Acceleration decreasing     |
| IV    | 0        | Constant velocity (cruise)  |
| V     | -j_max   | Deceleration increasing     |
| VI    | 0        | Constant max deceleration   |
| VII   | +j_max   | Deceleration decreasing     |

Not all phases are present in every move. Short moves skip phases II, IV, and/or VI.

### Case classification

Given initial conditions `(s0, v0, a=0)` and target `sE`, the algorithm classifies the trajectory:

- **Case e:** Remaining distance is less than the minimum stopping distance. The motor must overshoot, stop, and reverse (wandering).
- **Case c:** `v_max` reachable without hitting `a_max`. Jerk-limited acceleration with shortened phases I/III and no phase II.
- **Case b:** `a_max` reached but not `v_max`. Peak velocity solved via quadratic equation.
- **Case a:** Both `a_max` and `v_max` reached. Full 7-phase profile with cruising.
- **Case d:** Neither `a_max` nor `v_max` reached. Very short moves using a suboptimal strategy.

Evaluation order in code: e → c → b → a → d.

---

## 5. Mid-motion replanning strategy

When a new setpoint arrives while the motor is moving, the planner must build a new trajectory that respects the motor's current kinematic state. The paper's approach uses "virtual history" to reconstruct equivalent initial conditions with zero acceleration. However, this approach has a fundamental limitation: when the motor has significant acceleration at the moment of replanning, the new trajectory (which assumes `a=0` at its start) does not match the motor's actual state, causing discontinuities.

### The generalized forced-braking approach (implemented)

Rather than using virtual history for mid-motion replans, the current implementation uses a simpler and more robust strategy:

1. When a new setpoint arrives and the motor has significant acceleration (`|a| > j_max * Ts`), the replan is **deferred**. The `tooFastPending` flag is set on the active plan.

2. The ISR detects `tooFastPending` and enters a **forced-braking mode**: it applies `+j_max` or `-j_max` (whichever drives acceleration toward zero) each tick. This is checked before the normal trajectory completion check so it takes priority.

3. Once `|a|` drops below the threshold, the ISR snaps acceleration to zero.

4. The main loop detects that acceleration has reached zero and calls `calculateNewRamp` directly with the motor's actual `(theta, omega)`. Since `a ≈ 0`, the trajectory computed from these values matches the motor's real state perfectly.

This approach eliminates the entire class of virtual-history mismatch bugs. The tradeoff is a brief transient (typically 5–15 ms at the given constraints) while acceleration ramps to zero, but the resulting trajectory is always smooth and physically consistent.

### Wandering (stop-and-reverse)

When the motor is moving away from the target or cannot stop in time, the planner builds a braking-only profile (phases 5–7) to bring the motor to a stop. Once stopped, `wanderReplanPending` is raised and the main loop builds a fresh trajectory from rest toward the target.

---

## 6. Bugs fixed

### Round 1 — Initial code review (prior sessions)

Seventeen bugs were found and fixed in the initial code review, covering operator precedence errors, dangling pointers, variable shadowing, missing initializations, dead code, incorrect math library usage (`abs` vs `fabsf`), and simulation integration issues. These are documented in the original `SCurve_Development_Context.md`.

### Round 2 — Mid-motion replan failures (this session)

Three interconnected bugs prevented scenario 2 (extend: 30° → 60° mid-motion) from working.

**Bug 2a: ISR forced braking drove acceleration the wrong direction.** The original `tooFastPending` handler in the ISR computed jerk using `j = -j_max * dir`, which aimed to decelerate the motor (driving acceleration toward `-a_max`). The paper's intent is to drive `|a|` toward zero. Since the motor was in a decel phase with `a < 0`, the code made acceleration more negative instead of bringing it back toward zero.

*Fix:* The forced-braking handler now checks the sign of acceleration directly: if `a < 0`, apply `j = +j_max`; if `a > 0`, apply `j = -j_max`. When `|a|` falls below the threshold (`0.5 * j_max * Ts`), acceleration is snapped to zero.

**Bug 2b: `tooFastPending` was checked after trajectory completion.** The ISR checked `t_start >= t_final` before `tooFastPending`, so the original trajectory would finish (zeroing velocity and acceleration) before the forced-braking logic ever ran.

*Fix:* The `tooFastPending` block was moved above the trajectory completion check.

**Bug 2c: Main loop never replanned after forced braking resolved.** Even when forced braking brought `a` to zero, the main loop's handler called `calculateVirtualHistory` again, which would re-trigger `isPastTooFast` in an infinite loop because the flag was never properly cleared for a stopped motor.

*Fix:* The main loop now checks `fabsf(tracker->accel) <= threshold` directly instead of relying on the `isPastTooFast` flag. When the condition is met, it calls `calculateNewRamp` directly with the motor's actual state.

### Round 3 — Wandering profile failures (this session)

Two bugs caused velocity/acceleration discontinuities in wandering scenarios (4 and 6).

**Bug 3a: Braking direction was based on target direction, not velocity direction.** In `calculateNewRamp`, the `dir` field was computed as the direction toward the target: `dir = (targetPos - s0 >= 0) ? 1 : -1`. The ISR uses `dir` to compute jerk via `selectJerk(phase, j_max) * dir`. For a wandering case where the motor moves away from the target, `dir` pointed opposite to the motor's velocity, inverting the jerk signs — producing acceleration instead of braking.

*Fix:* When `isWandering` is true, `dir` is now based on the actual motor velocity direction: `dir = (motorTracker->omega >= 0) ? 1 : -1`.

**Bug 3b: Braking profiles were computed from virtual velocity, not actual velocity.** When a mid-motion replan triggered during acceleration or deceleration phases, the virtual history reconstructed a different starting velocity. The braking profile's duration was computed for this virtual velocity, but the ISR integrated from the actual motor state. The mismatch meant the motor would still be moving when the braking profile's time expired, causing a velocity snap to zero.

*Fix:* Both wandering call sites in `determineSwitchingTimes` now pass `motorTracker->omega` (actual velocity) to `decellerate2Stop` instead of the virtual `v0`. Additionally, `newPlan->theta` is set to `motorTracker->theta` (actual position) instead of the virtual `s0`.

**Bug 3c: Virtual history mismatch for all phases (generalization).** The same acceleration-mismatch problem that affected phases 5–7 also affected phases 1–3. When the motor had significant acceleration in any phase and a new trajectory was built assuming `a=0`, the ISR's integration from the actual nonzero acceleration caused the braking profile to be too short, resulting in velocity discontinuities at trajectory completion.

*Fix:* `buildNewCurve` was simplified to a single acceleration check. If the motor has significant acceleration (`|a| > j_max * Ts`) during any executing trajectory, replanning is deferred via `tooFastPending`. The ISR drives acceleration to zero, then the main loop replans from the clean constant-velocity state. This eliminated `calculateVirtualHistory` from the replan path entirely — virtual history is no longer called during mid-motion replans.

---

## 7. Test results after all fixes

All six built-in scenarios pass with zero velocity/acceleration discontinuities and sub-0.001° position error:

| Scenario | Description | Final error | Discontinuities |
|----------|-------------|-------------|-----------------|
| 1 Simple | 0 → 45° from rest | 0.0002° | 0 |
| 2 Extend | 30° → 60° mid-motion | 0.0001° | 0 |
| 3 Shorten | 60° → 25° mid-motion | 0.0000° | 0 |
| 4 Reverse | 45° → -20° mid-motion | 0.0003° | 0 |
| 5 Short | 0 → 0.5° (case d) | 0.0000° | 0 |
| 6 Rapid | 30→50→20→60° | 0.0002° | 0 |

---

## 8. Remaining integration tasks for STM32 firmware

1. Replace `getCurrentPosition()` stub with actual encoder read.
2. Switch ISR back to `updateVelocityFilter()` — in simulation the ISR writes integrated state directly; on hardware it should call `updateVelocityFilter()` which reads the encoder and computes derivatives.
3. Hook `buildNewCurve()` to `Handle_Run_Command()` in `CAN_processing_v2.c`.
4. Hook `PosCtrl_ISRStep()` into the TIM6 1 kHz interrupt.
5. Hook `Planner_MainStep()` into the main `while(1)` loop.
6. Enable velocity filter EMA once encoder noise is characterized.
7. Determine actual `A_MAX` and `J_MAX` values from motor/mechanical testing.
8. Consider critical sections or memory barriers for the `volatile` shared variables in production.

---

## 9. Build and test

```bash
# Build
gcc -Wall -Wextra -o sim main.c planner.c SCurveTrajectory.c -lm

# Run all scenarios
echo "0" | ./sim

# Run a single scenario
echo "2" | ./sim

# Run interactive mode
echo "7" | ./sim

# Plot results
python3 plot_trajectory.py test/sim_extend.csv
```
