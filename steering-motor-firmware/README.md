# steering-motor-firmware

## PID

The PID Loop is implemented using the systick timer.
In `stm32f4xx_it.c` The function SysTick_Handler is used to use the systick timer and calles the function `SysTickFunction` which processes our PID loop every ms.

`SysTickFunction` in `systick.c` runs the PID code and updates the code with the current encoder value that is automatically updated by the stm32 encoder mode.

PID is setup using 4 parameters.

`kPw` and `kdW` are the constants for PID control.  

`ALLOWED_ERROR` is the error range in counts where the motor is considered close enough to target so it can be stopped.

`ALLOWED_ERROR_ZERO` is the same as `ALLOWED_ERROR` but is used to hanfle strange oscillations when the goal is set to `0`.

## CAN

