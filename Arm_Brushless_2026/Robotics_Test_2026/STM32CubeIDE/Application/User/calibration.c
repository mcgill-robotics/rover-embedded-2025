/**
 * @file    calibration.c
 * @brief   Limit-switch calibration for the robotic arm ESC
 *
 * Startup calibration:
 *   On RUN_CALIBRATION CAN command, the arm waits for FOC to reach RUN
 *   state, then creeps at CAL_SEEK_SPEED until the lower limit switch
 *   (PB3) fires.  The raw encoder position at that moment is recorded,
 *   a position_offset is computed (raw_frame = calibrated_frame + offset),
 *   and all kinematic state is immediately hard-zeroed so the very next
 *   ISR tick sends a stable setpoint to the SDK PID with no jump.
 *   A 200 ms settle window confirms the motor is at rest before
 *   declaring calibration complete.
 *
 * Two-frame coordinate system:
 *   raw frame      — the SDK's internal wMecAngle accumulator.  Starts
 *                    at an arbitrary value at power-on.
 *   calibrated frame — zero at the lower limit switch.  All planner
 *                    setpoints, CAN telemetry, and the velocity controller
 *                    operate in this frame.
 *
 *   raw = calibrated + position_offset
 *
 *   The TIM6 ISR applies the offset every tick:
 *     raw_setpoint = calibrated_setpoint + Calibration_GetOffset()
 *     pid_setpoint = outputShaftToInput(raw_setpoint, gearRatio)
 *     MC_ProgramPositionCommandMotor1(pid_setpoint, 0)
 *
 *   This is the only correct approach while the SDK's wMecAngle
 *   accumulator is live.  Attempting to reset wMecAngle mid-session via
 *   ENC_SetMecAngle() causes a burst spin because the SDK position PID
 *   sees a huge instantaneous error between the new target and the
 *   stale wMecAngle value still held by the FOC loop internals.
 *
 * Key invariants:
 *   1. tracker->theta and velCtrl->cmd_pos must always agree.
 *      velCtrl_ISRStep() writes cmd_pos to tracker->theta every tick,
 *      so any external snap MUST also snap velCtrl->cmd_pos.
 *
 *   2. Both plan buffers must agree with tracker->theta after any snap,
 *      because PosCtrl_ISRStep reads plan->theta as its starting point.
 *      RuntimeSnap MUST sync both plan buffers and the SDK target — not
 *      doing so is what caused the original "jump back to boot position"
 *      bug: a bouncing limit switch fired RuntimeSnap after CAL_COMPLETE,
 *      which snapped tracker->theta and zeroed position_offset, but left
 *      the plan buffers and SDK target stale.  The next position command
 *      then stepped the SDK target from its stale value to the new
 *      computed value, producing the lurch.
 *
 *   3. At switch-hit, ALL kinematic state (cmd_vel, cmd_accel, omega,
 *      accel, jerk, *_prev fields) must be zeroed immediately — not
 *      deferred to CAL_STOPPING completion.  Even one ISR tick with
 *      the seek velocity still live integrates a position step that,
 *      multiplied by gearRatio=100, becomes a large SDK setpoint jump.
 *
 *   4. The SDK sync in CAL_STOPPING must use snapped + offset (raw
 *      frame), not snapped alone, so the SDK target and wMecAngle agree.
 *
 *   5. RuntimeSnap is locked out for RUNTIME_SNAP_LOCKOUT_MS after
 *      CAL_COMPLETE, and additionally re-reads the GPIO pin to confirm
 *      the switch is actually open — both are necessary because a
 *      mechanical NC limit switch bounces on opening, and any bounce
 *      that arrives after the mode flips out of MODE_CALIBRATING would
 *      otherwise corrupt position_offset.
 */

#include "calibration.h"
#include "velocity_ctrl.h"
#include "SCurveTrajectory.h"
#include "planner.h"
#include "mc_api.h"
#include "main.h"
#include <math.h>

/* ---- Tunables ---- */

#define CAL_SEEK_SPEED      (-0.15f)

/* Joint travel limits — from main.h (JOINT_MIN_RAD / JOINT_MAX_RAD).
   Single source of truth shared with CAN_processing_v2.c. */
#define LOWER_LIMIT_ANGLE    JOINT_MIN_RAD
#define UPPER_LIMIT_ANGLE    JOINT_MAX_RAD

/* Clearance from the lower switch so the arm does not rest on the contact. */
#define CAL_CLEARANCE       (degreesToRad(2.0f))

/* Settle window: 1 kHz ticks to hold zero velocity before CAL_COMPLETE. */
#define CAL_SETTLE_TICKS    200u

/* Lockout window after CAL_COMPLETE before runtime snaps are honored.
   Must be longer than the worst-case mechanical bounce time of the
   limit switch.  Most mechanical switches settle within 5–20 ms; 100 ms
   is a comfortable safety margin and still imperceptible to the user. */
#define RUNTIME_SNAP_LOCKOUT_MS   100u

/* ---- Static state ---- */

static volatile CalibrationState cal_state       = CAL_NOT_STARTED;
static volatile LimitSwitch      hit_switch      = LIMIT_NONE;
static volatile float            position_offset = 0.0f;
static volatile bool             switch_event    = false;
static volatile uint16_t         settle_ticks    = 0u;

/* Lockout countdown — set non-zero at CAL_COMPLETE, decremented at 1 kHz
   by Calibration_Tick_1ms().  RuntimeSnap is suppressed while > 0. */
static volatile uint16_t         runtime_snap_lockout_ms = 0u;

/* Raw encoder position (tracker->theta in raw frame) when switch fired. */
static volatile float            raw_pos_at_switch = 0.0f;

/* Diagnostic counters. */
volatile uint32_t Calibration_runtime_snap_attempts = 0u;
volatile uint32_t Calibration_runtime_snap_applied  = 0u;

/* ---- Externals ---- */

extern volatile PosCtrlHandle   *paths_planned[2];
extern VelCtrlHandle            *velCtrl;
extern float                     gearRatio;

/* ================================================================ */
/*  Public API                                                       */
/* ================================================================ */

void Calibration_Init(void)
{
    cal_state               = CAL_NOT_STARTED;
    hit_switch              = LIMIT_NONE;
    position_offset         = 0.0f;
    switch_event            = false;
    settle_ticks            = 0u;
    runtime_snap_lockout_ms = 0u;
    raw_pos_at_switch       = 0.0f;
    Calibration_runtime_snap_attempts = 0u;
    Calibration_runtime_snap_applied  = 0u;
}

void Calibration_LimitHit(LimitSwitch sw)
{
    hit_switch   = sw;
    switch_event = true;
}

void Calibration_MainStep(void)
{
    VelocityFilter *tracker = (VelocityFilter *)motorTracker;

    switch (cal_state) {

    /* ------------------------------------------------------------------ */
    case CAL_NOT_STARTED:
    /* ------------------------------------------------------------------ */
        /* Wait for FOC RUN state — the SDK goes IDLE→START→CHARGE_BOOT_CAP
           →OFFSET_CALIB→RUN.  Engaging velocity control before RUN causes
           a lurch because the current controller is not yet active. */
        if (MC_GetSTMStateMotor1() != RUN) {
            MC_StartMotor1();
            return;
        }
        velCtrlStart(velCtrl, tracker);
        velCtrlSetDemand(velCtrl, CAL_SEEK_SPEED);
        cal_state = CAL_SEEKING_LOWER;
        break;

    /* ------------------------------------------------------------------ */
    case CAL_SEEKING_LOWER:
    /* ------------------------------------------------------------------ */
        if (switch_event && hit_switch == LIMIT_LOWER) {
            switch_event = false;

            /* Kill demand so the jerk filter targets zero. */
            velCtrlSetDemand(velCtrl, 0.0f);

            /* Record raw position and compute offset.
               raw_frame = calibrated_frame + position_offset */
            raw_pos_at_switch = tracker->theta;
            position_offset   = raw_pos_at_switch - LOWER_LIMIT_ANGLE;

            /* Snap position to clearance in the calibrated frame. */
            float snapped = LOWER_LIMIT_ANGLE + CAL_CLEARANCE;

            /* --- Hard-stop ALL kinematic state immediately --- */
            velCtrl->cmd_vel    = 0.0f;
            velCtrl->cmd_accel  = 0.0f;
            velCtrl->cmd_pos    = snapped;

            tracker->theta      = snapped;
            tracker->theta_prev = snapped;
            tracker->omega      = 0.0f;
            tracker->omega_prev = 0.0f;
            tracker->accel      = 0.0f;
            tracker->accel_prev = 0.0f;
            tracker->jerk       = 0.0f;

            /* Enable joint limits now that the calibrated frame is known. */
            velCtrlSetPositionLimits(velCtrl,
                                     LOWER_LIMIT_ANGLE,
                                     UPPER_LIMIT_ANGLE);

            settle_ticks = CAL_SETTLE_TICKS;
            cal_state    = CAL_STOPPING;
        }
        break;

    /* ------------------------------------------------------------------ */
    case CAL_STOPPING:
    /* ------------------------------------------------------------------ */
        velCtrlSetDemand(velCtrl, 0.0f);

        if (settle_ticks == 0u) {
            float snapped = LOWER_LIMIT_ANGLE + CAL_CLEARANCE;

            /* Zero any residual kinematic state (safety net — should
               already be zero from the switch-hit snap above). */
            tracker->omega      = 0.0f;
            tracker->accel      = 0.0f;
            tracker->jerk       = 0.0f;
            tracker->omega_prev = 0.0f;
            tracker->accel_prev = 0.0f;
            velCtrl->cmd_vel    = 0.0f;
            velCtrl->cmd_accel  = 0.0f;

            /* Sync the SDK to the snapped calibrated position. */
            float raw_snapped  = snapped + position_offset;
            float pid_setpoint = outputShaftToInput(raw_snapped, gearRatio);
            MC_ProgramPositionCommandMotor1(pid_setpoint, 0);

            /* Sync both planner buffers to calibrated frame. */
            ((PosCtrlHandle *)paths_planned[0])->theta = snapped;
            ((PosCtrlHandle *)paths_planned[1])->theta = snapped;

            /* Arm the lockout BEFORE flipping to CAL_COMPLETE so any
               bounce that fires in the next few ms is suppressed even
               if main loop has already dropped us to MODE_IDLE. */
            runtime_snap_lockout_ms = RUNTIME_SNAP_LOCKOUT_MS;

            cal_state = CAL_COMPLETE;
        }
        break;

    /* ------------------------------------------------------------------ */
    case CAL_COMPLETE:
    /* ------------------------------------------------------------------ */
        break;

    } /* end switch */
}

void Calibration_ISRStep(void)
{
    if (cal_state == CAL_SEEKING_LOWER || cal_state == CAL_STOPPING) {
        velCtrl_ISRStep(velCtrl, (VelocityFilter *)motorTracker);
    }
    if (cal_state == CAL_STOPPING && settle_ticks > 0u) {
        settle_ticks--;
    }
}

void Calibration_Tick_1ms(void)
{
    /* This MUST run every tick, regardless of mode, so the lockout
       expires correctly after we drop into MODE_IDLE. */
    if (runtime_snap_lockout_ms > 0u) {
        runtime_snap_lockout_ms--;
    }
}

bool Calibration_IsDone(void)
{
    return (cal_state == CAL_COMPLETE);
}

float Calibration_GetOffset(void)
{
    return position_offset;
}

/* ----------------------------------------------------------------------
 *  Calibration_RuntimeSnap
 *
 *  Called from EXTI on a limit-switch rising edge during normal
 *  operation (controlMode != MODE_CALIBRATING).  Two gates:
 *
 *    1. Lockout — for RUNTIME_SNAP_LOCKOUT_MS after CAL_COMPLETE,
 *       suppress all snaps.  This catches contact bounces that arrive
 *       after the calibration state machine has handed control back
 *       to the main loop.
 *
 *    2. GPIO re-read — confirm the switch line is actually high right
 *       now.  A real over-travel produces a sustained high; a bounce
 *       glitch is back to low by the time we get here.
 *
 *  When the snap actually fires, EVERYTHING that depends on
 *  position_offset and tracker->theta must be updated together:
 *    - tracker->theta, velCtrl->cmd_pos                  (kinematic state)
 *    - both plan buffer thetas                           (planner uses these)
 *    - SDK target via MC_ProgramPositionCommandMotor1   (so wMecAngle
 *      and the new commanded position agree — without this, the next
 *      TIM6 tick steps the SDK target, producing a jump)
 * ---------------------------------------------------------------------- */
void Calibration_RuntimeSnap(LimitSwitch sw)
{
    Calibration_runtime_snap_attempts++;

    /* Gate 1: post-calibration lockout */
    if (runtime_snap_lockout_ms > 0u) {
        return;
    }

    /* Gate 2: confirm GPIO is genuinely high */
    GPIO_TypeDef *port;
    uint16_t      pin;
    float         known_angle;

    if (sw == LIMIT_LOWER) {
        port = GPIOB;
        pin  = LIMIT_SW_LOWER_Pin;
        known_angle = LOWER_LIMIT_ANGLE;
    } else if (sw == LIMIT_UPPER) {
        port = LIMIT_SW_UPPER_GPIO_Port;
        pin  = LIMIT_SW_UPPER_Pin;
        known_angle = UPPER_LIMIT_ANGLE;
    } else {
        return;
    }

    if (HAL_GPIO_ReadPin(port, pin) != GPIO_PIN_SET) {
        /* Pin is back to low — this was a bounce glitch. */
        return;
    }

    /* --- Snap is genuine.  Apply atomically. --- */

    VelocityFilter *tracker = (VelocityFilter *)motorTracker;

    /* Recompute offset from the current raw position. */
    position_offset = tracker->theta - known_angle;

    /* Snap kinematic state. */
    tracker->theta      = known_angle;
    tracker->theta_prev = known_angle;
    tracker->omega      = 0.0f;
    tracker->omega_prev = 0.0f;
    tracker->accel      = 0.0f;
    tracker->accel_prev = 0.0f;
    tracker->jerk       = 0.0f;

    velCtrl->cmd_pos    = known_angle;
    velCtrl->cmd_vel    = 0.0f;
    velCtrl->cmd_accel  = 0.0f;

    /* Sync both plan buffers — otherwise PosCtrl_ISRStep will read
       a stale plan->theta on the next position command and the SDK
       will see a step. */
    ((PosCtrlHandle *)paths_planned[0])->theta = known_angle;
    ((PosCtrlHandle *)paths_planned[1])->theta = known_angle;

    /* Sync SDK target to the new (raw-frame) position so the next
       TIM6 tick sends the same value the SDK is already holding. */
    float raw_known    = known_angle + position_offset;
    float pid_setpoint = outputShaftToInput(raw_known, gearRatio);
    MC_ProgramPositionCommandMotor1(pid_setpoint, 0);

    Calibration_runtime_snap_applied++;
}
