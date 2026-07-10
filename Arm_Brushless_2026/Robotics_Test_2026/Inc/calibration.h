#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    CAL_NOT_STARTED = 0,
    CAL_SEEKING_LOWER,
    CAL_STOPPING,
    CAL_BACKING_OFF,
    CAL_COMPLETE
} CalibrationState;

/* ... existing prototypes ... */

bool Calibration_BackOffTwoDegrees(void);

typedef enum {
    LIMIT_NONE = 0,
    LIMIT_LOWER,
    LIMIT_UPPER
} LimitSwitch;

/* Lifecycle */
void  Calibration_Init(void);          /* boot only — zeroes position_offset */
void  Calibration_Restart(void);       /* re-cal — preserves position_offset */

/* EXTI hooks */
void  Calibration_LimitHit(LimitSwitch sw);                  /* during cal */
void  Calibration_LimitEdge(LimitSwitch sw, bool rising);    /* always */

/* Foreground + ISR ticks */
void  Calibration_MainStep(void);      /* call from while(1) when MODE_CALIBRATING */
void  Calibration_ISRStep(void);       /* call from TIM6 when MODE_CALIBRATING */
void  Calibration_Tick_1ms(void);      /* call from TIM6 every tick, all modes */

/* Queries */
bool  Calibration_IsDone(void);
float Calibration_GetOffset(void);

/* Active joint limits in the (possibly rezeroed) calibrated frame.

   Before first calibration, these mirror the compile-time defaults
   JOINT_MIN_RAD / JOINT_MAX_RAD from main.h.

   After first calibration, the post-backoff rest position becomes
   calibrated 0, so:
     active_lower_limit == 0
     active_upper_limit == JOINT_MAX_RAD - JOINT_MIN_RAD - CAL_BACKOFF_OFFSET

   After a runtime lower-switch hit, the rezero re-anchors everything
   so active_lower_limit stays at 0 and active_upper_limit shrinks.
   After a runtime upper-switch hit, only active_upper_limit tightens
   to the new park position; active_lower_limit and the offset are
   unchanged.

   CAN_processing_v2.c uses these to clamp incoming RUN_POSITION and
   RUN_SPEED commands.  IMPORTANT: any caller of these getters must
   include this header — without a prototype, the implicit int return
   will scramble the float bit pattern and produce garbage limits. */
float Calibration_GetActiveLowerLimit(void);
float Calibration_GetActiveUpperLimit(void);

/* Diagnostics (optional — handy for debugging) */
extern volatile uint32_t Calibration_runtime_snap_attempts;
extern volatile uint32_t Calibration_runtime_snap_armed;
extern volatile uint32_t Calibration_runtime_snap_applied;
extern volatile uint32_t Calibration_runtime_snap_timeouts;

#endif /* CALIBRATION_H */
