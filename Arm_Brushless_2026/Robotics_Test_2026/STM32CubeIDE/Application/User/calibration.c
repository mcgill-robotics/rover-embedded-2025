/**
 * @file    calibration.c
 * @brief   Limit-switch calibration with falling-edge-confirmed snap
 *
 * Calibration sequence:
 *   CAL_NOT_STARTED → start FOC, command CAL_SEEK_SPEED
 *   CAL_SEEKING_LOWER → wait for switch hit, snap tracker, set offset
 *   CAL_STOPPING → 200 ms settle to ensure motor at rest
 *   CAL_BACKING_OFF → write a position step to the SDK PID directly,
 *                     wait for the encoder to confirm we got there
 *   CAL_COMPLETE → controlMode = MODE_IDLE, motor parks 2° above lower
 *
 * The back-off uses the SDK's position PID directly (not the velocity
 * controller).  The velocity controller is left at zero throughout
 * the back-off — only the SDK target is updated.  This avoids the
 * issues we hit with velocity-controlled back-off where residual
 * cmd_vel could leak past the state transition.
 */

#include "calibration.h"
#include "velocity_ctrl.h"
#include "SCurveTrajectory.h"
#include "planner.h"
#include "mc_api.h"
#include "main.h"
#include <math.h>

/* ---- Tunables ---- */

#define CAL_SEEK_SPEED            (-0.15f)
#define LOWER_LIMIT_ANGLE         JOINT_MIN_RAD
#define UPPER_LIMIT_ANGLE         JOINT_MAX_RAD
#define CAL_SETTLE_TICKS          200u

/* Post-cal back-off target, in radians above LOWER_LIMIT_ANGLE. */
#define CAL_BACKOFF_OFFSET        (degreesToRad(2.0f))

/* How long to wait for the encoder to reach the back-off target before
   timing out and proceeding to CAL_COMPLETE anyway.  At gearRatio=100
   and the SDK's typical position-loop speed, 2° at the output should
   take well under 500 ms.  1 second is generous. */
#define CAL_BACKOFF_TIMEOUT_TICKS 1000u

/* How close (output-shaft radians) the encoder must get to the target
   before we consider the back-off "done".  Within this band, we accept
   it and move on. */
#define CAL_BACKOFF_TOLERANCE     (degreesToRad(0.3f))

/* Minimum settle time after reaching the target — even if the encoder
   shows we're at target, wait this long before transitioning to
   CAL_COMPLETE so the system is genuinely at rest. */
#define CAL_BACKOFF_SETTLE_TICKS  50u

/* Duration of the back-off ramp in ISR ticks (1 kHz, so this is ms).
   Long enough that the per-tick step in tracker->theta is small after
   gear-ratio amplification.  At 500 ticks and 2° back-off:
     per-tick output-shaft step = 2° / 500       = 0.004° / tick
     per-tick SDK-input step    = 0.004° * 100   = 0.4°  / tick
   The SDK position PID treats this as continuous motion at ~400°/s on
   the input shaft (~4°/s output), which is gentle and well within the
   PID's tracking bandwidth.  Increase this if the back-off still looks
   abrupt; decrease it if the back-off feels sluggish. */
#define CAL_BACKOFF_RAMP_TICKS    500u

#define SNAP_RELEASE_CONFIRM_MS   5u
#define SNAP_ARM_TIMEOUT_MS       2000u

/* ---- Static state ---- */

static volatile CalibrationState cal_state         = CAL_NOT_STARTED;
static volatile LimitSwitch      hit_switch        = LIMIT_NONE;
static volatile float            position_offset   = 0.0f;
static volatile bool             switch_event      = false;
static volatile uint16_t         settle_ticks      = 0u;
static volatile float            raw_pos_at_switch = 0.0f;

/* Active joint limits in the (possibly rezeroed) calibrated frame.
   Before first calibration, these mirror JOINT_MIN_RAD / JOINT_MAX_RAD.
   After first cal:    lower = 0, upper = JOINT_MAX_RAD - JOINT_MIN_RAD
                                        - CAL_BACKOFF_OFFSET
   After lower runtime hit: lower stays at 0 (rezero_shift re-anchors),
                            upper shrinks by the shift amount.
   After upper runtime hit: upper retightens to the new park position,
                            lower unchanged. */
static volatile float active_lower_limit = 0.0f;   /* set in Init */
static volatile float active_upper_limit = 0.0f;   /* set in Init */

/* Back-off state. */
static volatile float    backoff_target_calibrated = 0.0f;
static volatile uint16_t backoff_settle_ticks      = 0u;

/* Smooth back-off ramp state.  During CAL_BACKING_OFF, tracker->theta
   advances from backoff_start_calibrated toward backoff_target_calibrated
   over CAL_BACKOFF_RAMP_TICKS ISR ticks, giving the SDK position PID a
   moving target it can track without slamming.  A pure step in
   tracker->theta gets multiplied by gearRatio at the SDK input, which
   the PID interprets as a violent setpoint jump — visible as a twitch.
   A 1 kHz ramp over a few hundred ms produces tick-to-tick steps small
   enough that the PID treats it as continuous motion. */
static volatile uint16_t backoff_ramp_tick = 0u;
static volatile float    backoff_start_calibrated = 0.0f;

typedef enum {
    SNAP_IDLE = 0,
    SNAP_ARMED,
    SNAP_RELEASE_PENDING
} SnapState;

static volatile SnapState   snap_state          = SNAP_IDLE;
static volatile LimitSwitch snap_pending_switch = LIMIT_NONE;
static volatile uint16_t    snap_release_ms     = 0u;
static volatile uint16_t    snap_arm_age_ms     = 0u;

static volatile bool snap_wait_for_release_lower = false;
static volatile bool snap_wait_for_release_upper = false;

volatile uint32_t Calibration_runtime_snap_attempts = 0u;
volatile uint32_t Calibration_runtime_snap_armed    = 0u;
volatile uint32_t Calibration_runtime_snap_applied  = 0u;
volatile uint32_t Calibration_runtime_snap_timeouts = 0u;

extern volatile PosCtrlHandle *paths_planned[2];
extern VelCtrlHandle          *velCtrl;
extern float                   gearRatio;

static void          commit_snap(LimitSwitch sw);
static GPIO_PinState read_switch(LimitSwitch sw);
static void          finalize_calibration(VelocityFilter *tracker,
                                          float park_position);
static void          rezero_shift(float delta);

/* ================================================================ */
/*  Public API                                                       */
/* ================================================================ */

void Calibration_Init(void)
{
    cal_state                   = CAL_NOT_STARTED;
    hit_switch                  = LIMIT_NONE;
    position_offset             = 0.0f;
    switch_event                = false;
    settle_ticks                = 0u;
    raw_pos_at_switch           = 0.0f;

    backoff_target_calibrated   = 0.0f;
    backoff_settle_ticks        = 0u;
    backoff_ramp_tick           = 0u;
    backoff_start_calibrated    = 0.0f;

    snap_state                  = SNAP_IDLE;
    snap_pending_switch         = LIMIT_NONE;
    snap_release_ms             = 0u;
    snap_arm_age_ms             = 0u;

    snap_wait_for_release_lower = false;
    snap_wait_for_release_upper = false;

    Calibration_runtime_snap_attempts = 0u;
    Calibration_runtime_snap_armed    = 0u;
    Calibration_runtime_snap_applied  = 0u;
    Calibration_runtime_snap_timeouts = 0u;

    /* Pre-calibration defaults — replaced when calibration completes. */
    active_lower_limit = LOWER_LIMIT_ANGLE;
    active_upper_limit = UPPER_LIMIT_ANGLE;
}

void Calibration_Restart(void)
{
    snap_state                  = SNAP_IDLE;
    snap_pending_switch         = LIMIT_NONE;
    snap_release_ms             = 0u;
    snap_arm_age_ms             = 0u;

    snap_wait_for_release_lower = false;
    snap_wait_for_release_upper = false;

    cal_state                   = CAL_NOT_STARTED;
    hit_switch                  = LIMIT_NONE;
    switch_event                = false;
    settle_ticks                = 0u;
    raw_pos_at_switch           = 0.0f;

    backoff_target_calibrated   = 0.0f;
    backoff_settle_ticks        = 0u;
    backoff_ramp_tick           = 0u;
    backoff_start_calibrated    = 0.0f;

    velCtrlClearPositionLimits(velCtrl);
}

void Calibration_LimitHit(LimitSwitch sw)
{
    hit_switch   = sw;
    switch_event = true;
}

void Calibration_LimitEdge(LimitSwitch sw, bool rising)
{
    if (sw != LIMIT_LOWER && sw != LIMIT_UPPER) {
        return;
    }

    if (cal_state != CAL_COMPLETE) {
        if (rising && cal_state == CAL_SEEKING_LOWER) {
            Calibration_LimitHit(sw);
        }
        return;
    }

    volatile bool *wait_flag =
        (sw == LIMIT_LOWER) ? &snap_wait_for_release_lower
                            : &snap_wait_for_release_upper;
    if (*wait_flag) {
        if (!rising) {
            *wait_flag = false;
        }
        return;
    }

    if (rising) {
        Calibration_runtime_snap_attempts++;

        snap_state          = SNAP_ARMED;
        snap_pending_switch = sw;
        snap_release_ms     = 0u;
        snap_arm_age_ms     = 0u;
        Calibration_runtime_snap_armed++;

        if (velCtrl != NULL) {
            velCtrlSetDemand(velCtrl, 0.0f);
        }
        controlMode = MODE_IDLE;

    } else {
        if (snap_state == SNAP_ARMED && snap_pending_switch == sw) {
            snap_state      = SNAP_RELEASE_PENDING;
            snap_release_ms = 0u;
        }
    }
}

void Calibration_MainStep(void)
{
    VelocityFilter *tracker = (VelocityFilter *)motorTracker;

    switch (cal_state) {

    /* ------------------------------------------------------------------ */
    case CAL_NOT_STARTED:
    /* ------------------------------------------------------------------ */
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
            velCtrlSetDemand(velCtrl, 0.0f);

            /* tracker->theta is calibrated-frame; convert to raw frame
               by adding the existing offset. */
            raw_pos_at_switch = tracker->theta + position_offset;
            position_offset   = raw_pos_at_switch - LOWER_LIMIT_ANGLE;

            velCtrl->cmd_vel    = 0.0f;
            velCtrl->cmd_accel  = 0.0f;
            velCtrl->cmd_pos    = LOWER_LIMIT_ANGLE;

            tracker->theta      = LOWER_LIMIT_ANGLE;
            tracker->theta_prev = LOWER_LIMIT_ANGLE;
            tracker->omega      = 0.0f;
            tracker->omega_prev = 0.0f;
            tracker->accel      = 0.0f;
            tracker->accel_prev = 0.0f;
            tracker->jerk       = 0.0f;

            /* DON'T arm position limits yet — back-off would be blocked. */

            settle_ticks = CAL_SETTLE_TICKS;
            cal_state    = CAL_STOPPING;
        }
        break;

    /* ------------------------------------------------------------------ */
    case CAL_STOPPING:
    /* ------------------------------------------------------------------ */
        velCtrlSetDemand(velCtrl, 0.0f);

        if (settle_ticks == 0u) {
            /* Settle done.  Set up a smooth ramp in tracker->theta from
               the current resting position (LOWER_LIMIT_ANGLE, set by
               the snap in CAL_SEEKING_LOWER) to LOWER_LIMIT_ANGLE +
               CAL_BACKOFF_OFFSET.  The ramp is advanced one step per
               TIM6 tick by Calibration_ISRStep while in CAL_BACKING_OFF.

               Critically: tracker->theta is NOT stepped to the target
               here.  Stepping it produces a gearRatio-amplified jump at
               the SDK input that the PID slams toward, which the user
               sees as an abrupt twitch off the switch.  Instead, we
               leave tracker->theta where the snap put it and let the
               ISR walk it smoothly to the target. */

            backoff_start_calibrated  = tracker->theta;       /* = LOWER_LIMIT_ANGLE */
            backoff_target_calibrated =
                LOWER_LIMIT_ANGLE + CAL_BACKOFF_OFFSET;
            backoff_ramp_tick = 0u;

            /* Velocity controller stays at zero throughout — its cmd_pos
               tracks tracker->theta as the ramp progresses, so it
               inherits a clean state when MODE_IDLE switches in. */
            velCtrl->cmd_vel    = 0.0f;
            velCtrl->cmd_accel  = 0.0f;

            /* Plan buffers will be re-synced in finalize_calibration once
               the ramp completes; leaving them stale during the ramp is
               harmless because controlMode is still MODE_CALIBRATING and
               the planner ISR step doesn't run. */

            backoff_settle_ticks = 0u;
            settle_ticks         = CAL_BACKOFF_TIMEOUT_TICKS;

            cal_state = CAL_BACKING_OFF;
        }
        break;

    /* ------------------------------------------------------------------ */
    case CAL_BACKING_OFF: {
    /* ------------------------------------------------------------------ */
        /* The TIM6 ISR (Calibration_ISRStep) advances tracker->theta one
           ramp step per tick toward backoff_target_calibrated.  The
           normal TIM6 push path computes raw_setpoint = tracker->theta
           + offset and sends it to the SDK PID, which tracks the moving
           target smoothly.

           We declare the back-off done when ALL of:
             - the ramp has completed (tracker->theta has reached target)
             - the lower switch has released (rotor is physically off it)
             - the system has been settled for CAL_BACKOFF_SETTLE_TICKS
           If the timeout fires before the switch releases, finalize at
           wherever the rotor ended up. */

        bool switch_released =
            (HAL_GPIO_ReadPin(GPIOB, LIMIT_SW_LOWER_Pin) == GPIO_PIN_RESET);
        bool ramp_done = (backoff_ramp_tick >= CAL_BACKOFF_RAMP_TICKS);

        if (switch_released && ramp_done) {
            backoff_settle_ticks++;
            if (backoff_settle_ticks >= CAL_BACKOFF_SETTLE_TICKS) {
                /* Off the switch, ramp finished, settled.  Done. */
                finalize_calibration(tracker, backoff_target_calibrated);
            }
        } else {
            /* Either still ramping, switch still engaged, or both.
               Reset the post-release counter so it only counts AFTER
               both conditions are met. */
            backoff_settle_ticks = 0u;

            if (settle_ticks == 0u) {
                /* Timed out.  Either the SDK PID didn't track (problem),
                   or the switch is mechanically stuck (also a problem),
                   or the user is holding it (their problem).  In all
                   cases, declare cal complete with the wait-for-release
                   latch set so runtime snaps are muted until the switch
                   eventually goes low.  Park where we are. */
                snap_wait_for_release_lower = true;
                finalize_calibration(tracker, tracker->theta);
            }
        }
        break;
    }

    /* ------------------------------------------------------------------ */
    case CAL_COMPLETE:
    /* ------------------------------------------------------------------ */
        break;
    }
}

void Calibration_ISRStep(void)
{
    /* Tick the velocity controller while the seek is running.  In
       CAL_BACKING_OFF the velocity controller is at zero and we don't
       need to tick it — the SDK PID is doing the work. */
    if (cal_state == CAL_SEEKING_LOWER || cal_state == CAL_STOPPING) {
        velCtrl_ISRStep(velCtrl, (VelocityFilter *)motorTracker);
    }

    /* Smooth back-off ramp: advance tracker->theta one step per tick
       toward backoff_target_calibrated.  The TIM6 ISR's setpoint push
       (in main.c's HAL_TIM_PeriodElapsedCallback) computes
           raw_setpoint = tracker->theta + offset
       on every tick regardless of mode, so as tracker->theta inches
       up here, the SDK PID sees a moving target it can track smoothly
       instead of a single big step.

       Linear ramp formula:
           tracker->theta = start + (target - start) * (tick / RAMP_TICKS)
       which we evaluate cleanly at integer tick boundaries to avoid
       drift from incremental addition.  After RAMP_TICKS ticks,
       tracker->theta == target exactly. */
    if (cal_state == CAL_BACKING_OFF
        && backoff_ramp_tick < CAL_BACKOFF_RAMP_TICKS) {

        VelocityFilter *tracker = (VelocityFilter *)motorTracker;

        backoff_ramp_tick++;
        float frac = (float)backoff_ramp_tick / (float)CAL_BACKOFF_RAMP_TICKS;
        if (frac > 1.0f) frac = 1.0f;

        float new_theta = backoff_start_calibrated
                        + (backoff_target_calibrated - backoff_start_calibrated)
                          * frac;

        tracker->theta_prev = tracker->theta;
        tracker->theta      = new_theta;

        /* Keep velCtrl's cmd_pos shadowing tracker->theta so a later
           velCtrlStart inherits a consistent state.  cmd_vel/cmd_accel
           remain at zero — we are not running the velocity controller. */
        velCtrl->cmd_pos    = new_theta;
    }

    if ((cal_state == CAL_STOPPING || cal_state == CAL_BACKING_OFF)
        && settle_ticks > 0u) {
        settle_ticks--;
    }
}

void Calibration_Tick_1ms(void)
{
    switch (snap_state) {

    case SNAP_IDLE:
        break;

    case SNAP_ARMED:
        if (snap_arm_age_ms < UINT16_MAX) {
            snap_arm_age_ms++;
        }
        if (snap_arm_age_ms >= SNAP_ARM_TIMEOUT_MS) {
            snap_state          = SNAP_IDLE;
            snap_pending_switch = LIMIT_NONE;
            snap_release_ms     = 0u;
            snap_arm_age_ms     = 0u;
            Calibration_runtime_snap_timeouts++;
        }
        break;

    case SNAP_RELEASE_PENDING: {
        LimitSwitch sw = snap_pending_switch;
        if (read_switch(sw) == GPIO_PIN_RESET) {
            snap_release_ms++;
            if (snap_release_ms >= SNAP_RELEASE_CONFIRM_MS) {
                commit_snap(sw);
                snap_state          = SNAP_IDLE;
                snap_pending_switch = LIMIT_NONE;
                snap_release_ms     = 0u;
                snap_arm_age_ms     = 0u;
                Calibration_runtime_snap_applied++;
            }
        } else {
            snap_release_ms = 0u;
            snap_state      = SNAP_ARMED;
        }
        break;
    }
    }
}

bool  Calibration_IsDone(void)    { return cal_state == CAL_COMPLETE; }
float Calibration_GetOffset(void) { return position_offset; }

/* ================================================================ */
/*  Internals                                                        */
/* ================================================================ */

static GPIO_PinState read_switch(LimitSwitch sw)
{
    if (sw == LIMIT_LOWER) {
        return HAL_GPIO_ReadPin(GPIOB, LIMIT_SW_LOWER_Pin);
    } else if (sw == LIMIT_UPPER) {
        return HAL_GPIO_ReadPin(LIMIT_SW_UPPER_GPIO_Port,
                                LIMIT_SW_UPPER_Pin);
    }
    return GPIO_PIN_RESET;
}

/* Single point of transition to CAL_COMPLETE.  Forces all motion-related
   state to clean defaults and parks at park_position (calibrated frame). */
static void finalize_calibration(VelocityFilter *tracker, float park_position)
{
    velCtrlSetDemand(velCtrl, 0.0f);

    tracker->theta      = park_position;
    tracker->theta_prev = park_position;
    tracker->omega      = 0.0f;
    tracker->accel      = 0.0f;
    tracker->jerk       = 0.0f;
    tracker->omega_prev = 0.0f;
    tracker->accel_prev = 0.0f;

    velCtrl->cmd_pos    = park_position;
    velCtrl->cmd_vel    = 0.0f;
    velCtrl->cmd_accel  = 0.0f;

    ((PosCtrlHandle *)paths_planned[0])->theta = park_position;
    ((PosCtrlHandle *)paths_planned[1])->theta = park_position;

    /* Establish the active joint range in the current (pre-rezero) frame.
       The lower switch trip is at LOWER_LIMIT_ANGLE; we just backed off
       to park_position; the physical upper-switch position is unknown
       until it's hit, so we use the compile-time JOINT_MAX_RAD as a
       conservative ceiling for now.  If the upper switch later trips,
       commit_snap will tighten it. */
    active_lower_limit = LOWER_LIMIT_ANGLE;
    active_upper_limit = UPPER_LIMIT_ANGLE;

    /* Now rezero so park_position becomes calibrated 0.
       After this call:
         tracker->theta == 0
         active_lower_limit == LOWER_LIMIT_ANGLE - park_position  (negative)
         active_upper_limit == UPPER_LIMIT_ANGLE - park_position
         position_offset    incremented by park_position
       The raw position commanded to the SDK is unchanged by this shift,
       so no PID step.

       active_lower_limit ends up slightly negative (the lower switch
       physically sits below the new 0, which is correct — the user
       backed off from it).  Clamp it to 0 so position commands can't
       send the motor back into the switch. */
    rezero_shift(park_position);

    if (active_lower_limit < 0.0f) {
        active_lower_limit = 0.0f;
    }

    /* Now safe to arm position limits, using the active (rezeroed) range. */
    velCtrlSetPositionLimits(velCtrl,
                             active_lower_limit,
                             active_upper_limit);

    /* Latch wait-for-release for any switch still engaged.  After a
       successful back-off, both should be open. */
    if (HAL_GPIO_ReadPin(GPIOB, LIMIT_SW_LOWER_Pin) == GPIO_PIN_SET) {
        snap_wait_for_release_lower = true;
    }
    if (HAL_GPIO_ReadPin(LIMIT_SW_UPPER_GPIO_Port,
                         LIMIT_SW_UPPER_Pin) == GPIO_PIN_SET) {
        snap_wait_for_release_upper = true;
    }

    snap_state          = SNAP_IDLE;
    snap_pending_switch = LIMIT_NONE;
    snap_release_ms     = 0u;
    snap_arm_age_ms     = 0u;

    backoff_settle_ticks = 0u;
    settle_ticks         = 0u;

    cal_state   = CAL_COMPLETE;
    controlMode = MODE_IDLE;
}

/* ----------------------------------------------------------------------
 *  rezero_shift
 *
 *  Shifts the calibrated frame by `delta` (in radians).  After the call:
 *      new_calibrated = old_calibrated - delta
 *      new_offset     = old_offset     + delta
 *
 *  The relationship raw = calibrated + offset is preserved tick-by-tick,
 *  so the SDK's commanded raw position does not step.  This is safe to
 *  call while the motor is at rest in any mode; the next TIM6 ISR sees
 *  the same raw_setpoint it would have computed before.
 *
 *  Every variable that lives in the calibrated frame must be shifted
 *  here — missing one will produce a step at the SDK input on the next
 *  tick, multiplied by gearRatio.
 * ---------------------------------------------------------------------- */
static void rezero_shift(float delta)
{
    if (delta == 0.0f) return;

    VelocityFilter *tracker = (VelocityFilter *)motorTracker;

    /* Update offset first so any concurrent ISR read of the raw setpoint
       sees a consistent (calibrated, offset) pair.  The TIM6 ISR has
       higher priority than the main loop where this is called from, but
       individual float assignments are not atomic on Cortex-M4 — there
       is a sub-microsecond window where the two values disagree.  Since
       this function is only called when the motor is at rest with the
       velocity controller idle, the rare worst case is one tick of the
       SDK target being off by `delta * gearRatio`, which the PID may
       glance at.  In practice the calibration paths that call this run
       with controlMode about to switch to MODE_IDLE, so the TIM6 push
       is using a stable tracker->theta either way. */
    position_offset    += delta;

    tracker->theta      -= delta;
    tracker->theta_prev -= delta;

    velCtrl->cmd_pos    -= delta;

    ((PosCtrlHandle *)paths_planned[0])->theta -= delta;
    ((PosCtrlHandle *)paths_planned[1])->theta -= delta;

    active_lower_limit  -= delta;
    active_upper_limit  -= delta;

    /* If the velocity controller's position-limit feature was already
       enabled (it isn't during calibration but may be by the time
       commit_snap calls this), shift its bounds too. */
    if (velCtrl->pos_limits_en) {
        velCtrl->pos_min -= delta;
        velCtrl->pos_max -= delta;
    }
}
/* Post-snap back-off, in radians.  After a runtime limit-switch hit,
   the motor moves this far off the switch (toward the joint interior)
   so it isn't resting on the contact. */
#define RUNTIME_BACKOFF_OFFSET   (degreesToRad(1.0f))

/* Public getters for the active joint range.  CAN_processing_v2.c uses
   these to clamp incoming position commands to the current valid range. */
float Calibration_GetActiveLowerLimit(void) { return active_lower_limit; }
float Calibration_GetActiveUpperLimit(void) { return active_upper_limit; }

/* ----------------------------------------------------------------------
 *  commit_snap
 *
 *  Called from Calibration_Tick_1ms after a confirmed
 *  rising→falling→stable-low sequence on a runtime hit.
 *
 *  Behavior depends on which switch tripped:
 *
 *    Lower switch:
 *      - Park at  current_theta + RUNTIME_BACKOFF_OFFSET (in calibrated frame)
 *      - Rezero so the park position becomes calibrated 0
 *      - active_lower_limit ends up at 0; active_upper_limit shrinks by
 *        the rezero amount; position_offset increases by the same.
 *
 *    Upper switch:
 *      - Park at  current_theta - RUNTIME_BACKOFF_OFFSET
 *      - Tighten active_upper_limit to the park position
 *      - No rezero; active_lower_limit and position_offset are unchanged.
 *      - The SDK's commanded raw position is the same after the step
 *        (we set tracker->theta = park_position with offset unchanged),
 *        so the TIM6 ISR drives the rotor to the new park.
 *
 *  In both cases:
 *    - raw = calibrated + offset is preserved across the transition,
 *      so no PID step at the SDK input.
 *    - velocity controller is held at zero.
 *    - wait-for-release latch is set on the engaged switch so bounces
 *      during the back-off don't re-arm a new snap.
 * ---------------------------------------------------------------------- */
static void commit_snap(LimitSwitch sw)
{
    VelocityFilter *tracker = (VelocityFilter *)motorTracker;
    float current_theta = tracker->theta;
    float park_position;

    if (sw == LIMIT_LOWER) {
        park_position = current_theta + RUNTIME_BACKOFF_OFFSET;
    } else if (sw == LIMIT_UPPER) {
        park_position = current_theta - RUNTIME_BACKOFF_OFFSET;
    } else {
        return;
    }

    /* Defensive: park_position must lie inside whatever joint range we
       currently believe is valid.  If RUNTIME_BACKOFF_OFFSET is set
       absurdly large, or if the switches are wildly miscalibrated,
       clamp to active limits. */
    if (park_position < active_lower_limit) park_position = active_lower_limit;
    if (park_position > active_upper_limit) park_position = active_upper_limit;

    /* Step every calibrated-frame variable to park_position.  This is
       the same atomic snap pattern used at cal time — tracker, velCtrl,
       and both plan buffers all agree, with all derivatives zeroed. */
    tracker->theta      = park_position;
    tracker->theta_prev = park_position;
    tracker->omega      = 0.0f;
    tracker->omega_prev = 0.0f;
    tracker->accel      = 0.0f;
    tracker->accel_prev = 0.0f;
    tracker->jerk       = 0.0f;

    velCtrl->cmd_pos    = park_position;
    velCtrl->cmd_vel    = 0.0f;
    velCtrl->cmd_accel  = 0.0f;
    velCtrlSetDemand(velCtrl, 0.0f);

    ((PosCtrlHandle *)paths_planned[0])->theta = park_position;
    ((PosCtrlHandle *)paths_planned[1])->theta = park_position;

    if (sw == LIMIT_LOWER) {
        /* Rezero: park_position becomes the new calibrated 0.
           active_lower_limit was probably already 0 (if previous cal
           rezeroed), but if drift accumulated we re-anchor it.
           active_upper_limit shrinks by park_position so the absolute
           physical range is preserved. */
        rezero_shift(park_position);

        /* After rezero, tracker->theta == 0.  Snap any tiny FP residual
           in active_lower_limit to 0 — the user just re-established
           "this is the lower reference". */
        active_lower_limit = 0.0f;

        /* Push the new lower limit to the velocity controller. */
        if (velCtrl->pos_limits_en) {
            velCtrl->pos_min = active_lower_limit;
            velCtrl->pos_max = active_upper_limit;
        }

    } else {
        /* Upper hit: the upper switch's calibrated-frame position is
           now known to be `current_theta` (where the snap latched).
           After backing off to park_position, set the new active upper
           limit to park_position so the user can't command back into
           the contact.  No rezero — 0 stays anchored at the lower
           reference established at calibration time. */
        active_upper_limit = park_position;

        if (velCtrl->pos_limits_en) {
            velCtrl->pos_max = active_upper_limit;
        }
    }

    /* Suppress runtime-snap activity on this switch until it goes low.
       During the back-off move, the lever is still pressed (the rotor
       hasn't physically moved off yet — the SDK PID is just starting
       to drive it).  Bounces during this transient should not arm
       new snaps.  The latch clears on the next falling edge, which
       happens naturally when the lever lifts off the contact. */
    if (sw == LIMIT_LOWER) {
        snap_wait_for_release_lower = true;
    } else if (sw == LIMIT_UPPER) {
        snap_wait_for_release_upper = true;
    }

    /* DO NOT call MC_ProgramPositionCommandMotor1 — TIM6 handles it. */

    controlMode = MODE_IDLE;
}
