@echo off
REM ============================================================
REM  launch_arm.bat — Arm test launcher
REM  Edit the variables below to match your setup.
REM ============================================================

set PORT=COM4
set DB=can_log.db
set SHOULDER_SPEED=0.5
set ELBOW_SPEED=0.5
set WAIST_SPEED=0.2

REM --- Terminal 1: CAN logger (capture + command server) ---
start "CAN Logger" cmd /k python scripts/can_logger.py --port %PORT% --db %DB%

REM Give the logger a moment to bind before the dashboard connects
timeout /t 2 /nobreak > nul

REM --- Terminal 2: Dashboard ---
start "CAN Dashboard" cmd /k python scripts/can_dashboard.py --db %DB%

REM --- Optional Terminal 3: PS4 controller ---
set /p USE_CONTROLLER="Launch PS4 controller? (y/n): "
if /i "%USE_CONTROLLER%"=="y" (
    start "PS4 Controller" cmd /k python scripts/ps4_arm_controller.py --shoulder-speed %SHOULDER_SPEED% --elbow-speed %ELBOW_SPEED% --waist-speed %WAIST_SPEED%
)

echo.
echo All processes launched. Close this window or press any key to exit.
pause > nul
