#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    CAL_NOT_STARTED,
    CAL_SEEKING_LOWER,
    CAL_STOPPING,
    CAL_COMPLETE
} CalibrationState;

typedef enum {
    LIMIT_NONE,
    LIMIT_LOWER,    /* PB3 */
    LIMIT_UPPER     /* PB4 */
} LimitSwitch;

/** Call once at startup after GPIO init. */
void  Calibration_Init(void);

/** Call from main loop when controlMode == MODE_CALIBRATING. */
void  Calibration_MainStep(void);

/** Call from 1 kHz TIM6 ISR when controlMode == MODE_CALIBRATING —
 *  runs velCtrl filter and settle counter. */
void  Calibration_ISRStep(void);

/** Call from 1 kHz TIM6 ISR every tick (regardless of mode) —
 *  decrements the runtime-snap lockout timer.  Separate from
 *  Calibration_ISRStep because it must keep ticking after CAL_COMPLETE
 *  and after MODE_IDLE so the lockout actually expires. */
void  Calibration_Tick_1ms(void);

/** Call from EXTI ISR — records which switch fired. */
void  Calibration_LimitHit(LimitSwitch sw);

/** Returns true once calibration is complete. */
bool  Calibration_IsDone(void);

/**
 * Returns the position offset (radians) between raw encoder frame and
 * the calibrated frame.
 *
 *   raw_setpoint = calibrated_setpoint + Calibration_GetOffset()
 *
 * Applied in the TIM6 ISR before MC_ProgramPositionCommandMotor1().
 * This is the correct approach — do not attempt to reset wMecAngle via
 * ENC_SetMecAngle() while the FOC loop is live; it causes a burst spin.
 */
float Calibration_GetOffset(void);

/**
 * Runtime re-snap: call from EXTI during normal operation when a limit
 * switch fires.  Recomputes offset and snaps tracker + velCtrl in place,
 * also syncs both plan buffers and the SDK target so the next command
 * does not produce a step.
 *
 * Internally gated by:
 *   1. A post-calibration lockout window (suppresses bounce-induced
 *      events that arrive after CAL_COMPLETE has flipped the mode out
 *      of MODE_CALIBRATING).
 *   2. A re-read of the GPIO pin to confirm the switch is genuinely
 *      open (suppresses single-cycle bounce glitches).
 *
 * If either gate fails, the call returns without modifying state.
 */
void  Calibration_RuntimeSnap(LimitSwitch sw);

/* Diagnostic counters — useful for confirming bounce activity on the
   limit switches via telemetry / debugger watch.
   Calibration_runtime_snap_attempts increments on every call.
   Calibration_runtime_snap_applied  increments only when the snap
   actually modified state. */
extern volatile uint32_t Calibration_runtime_snap_attempts;
extern volatile uint32_t Calibration_runtime_snap_applied;

#endif /* CALIBRATION_H */
