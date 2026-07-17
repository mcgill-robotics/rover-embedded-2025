"""
ps4_arm_controller.py — PS4 controller for 3-DOF robotic arm via CAN

Controls three arm joints using a DualShock 4 / DualSense controller:
    Left stick Y-axis   → Shoulder velocity        (ESC ID 9)
    Left stick X-axis   → Waist rotation velocity    (ESC ID 8)
    Right stick Y-axis  → Elbow velocity             (ESC ID 10)
    Square button        → Acknowledge faults (sent to all 3 ESCs)
    Circle button        → STOP all motors
    PS / Home button     → Quit

Requires:
    pip install pygame

Usage:
    # Make sure can_logger.py is running with --cmd-port 5555
    python ps4_arm_controller.py
    python ps4_arm_controller.py --shoulder-speed 0.8 --elbow-speed 0.6 --waist-speed 0.4
    python ps4_arm_controller.py --cmd-port 5555 --poll-rate 50

Architecture:
    PS4 controller  →  this script  →  TCP (CmdClient)  →  CmdServer
                                        (in can_logger.py)  →  CAN bus  →  ESCs
"""

from __future__ import annotations

import argparse
import struct
import sys
import time

# ---------------------------------------------------------------------------
# Import the project's CAN protocol and TCP client
# ---------------------------------------------------------------------------
# These live alongside this script in the esc_can package.
# If running from a different directory, adjust sys.path or install the package.
try:
    from esc_can.protocol import (
        Sender, Action, MotorConfig, MotorType,
        RunSpec, ReadSpec, MotorID,
        encode_can_id, pack_single_float,
        build_single_run_frame,
    )
    from esc_can.cmd_server import CmdClient
except ImportError:
    # Fallback: try importing from the same directory
    from esc_can.protocol import (
        Sender, Action, MotorConfig, MotorType,
        RunSpec, ReadSpec, MotorID,
        encode_can_id, pack_single_float,
        build_single_run_frame,
    )
    from esc_can.cmd_server import CmdClient

try:
    import pygame
except ImportError:
    print("ERROR: pygame is required.  Install it with:  pip install pygame")
    sys.exit(1)


# ---------------------------------------------------------------------------
# PS4 controller button / axis mapping (pygame on most platforms)
# ---------------------------------------------------------------------------
# These indices match the DualShock 4 as seen by SDL / pygame.
# If your controller reports different indices, adjust here.

AXIS_LEFT_X   = 0    # Left stick horizontal (−1 = left, +1 = right)
AXIS_LEFT_Y   = 1    # Left stick vertical   (−1 = up, +1 = down)
AXIS_RIGHT_Y  = 3    # Right stick vertical

BUTTON_SQUARE  = 2   # Square  (DS4) / X (DualSense)
BUTTON_CIRCLE  = 1   # Circle
BUTTON_L1      = 4   # Left bumper
BUTTON_R1      = 5   # Right bumper
BUTTON_PS      = 10  # PS / Home button

# On some systems these differ — we'll detect and print at startup


# ---------------------------------------------------------------------------
# ESC device IDs for the arm joints
# ---------------------------------------------------------------------------
# DEV_WAIST    = MotorID.WAIST       # 8
# DEV_SHOULDER = MotorID.SHOULDER    # 9
# DEV_ELBOW    = MotorID.ELBOW       # 10
DEV_SHOULDER = 1    # 9
DEV_ELBOW    = 2      # 10
DEV_WAIST    = 3      # 8


# ---------------------------------------------------------------------------
# Helper: send a velocity command to one joint
# ---------------------------------------------------------------------------
def send_velocity(client: CmdClient, device_id: int, velocity_rad_s: float) -> None:
    """Send a RUN_SPEED command with a float velocity (rad/s)."""
    arb_id, data = build_single_run_frame(
        device_id=device_id,
        run_spec=RunSpec.SPEED,
        value=velocity_rad_s,
        motor_type=MotorType.DRIVE,
    )
    try:
        client.send_frame(arb_id, data, is_fd=False)
    except ConnectionError as e:
        print(f"  [TX ERROR] {e}")


def send_stop(client: CmdClient, device_id: int) -> None:
    """Send a RUN_STOP command."""
    arb_id, data = build_single_run_frame(
        device_id=device_id,
        run_spec=RunSpec.STOP,
        value=0.0,
        motor_type=MotorType.DRIVE,
    )
    try:
        client.send_frame(arb_id, data, is_fd=False)
    except ConnectionError as e:
        print(f"  [TX ERROR] {e}")


def send_acknowledge_faults(client: CmdClient, device_id: int) -> None:
    """Send a RUN_ACKNOWLEDGE_FAULTS command."""
    arb_id, data = build_single_run_frame(
        device_id=device_id,
        run_spec=RunSpec.ACKNOWLEDGE_FAULTS,
        value=0.0,
        motor_type=MotorType.DRIVE,
    )
    try:
        client.send_frame(arb_id, data, is_fd=False)
    except ConnectionError as e:
        print(f"  [TX ERROR] {e}")


# ---------------------------------------------------------------------------
# Deadzone helper
# ---------------------------------------------------------------------------
def apply_deadzone(value: float, deadzone: float = 0.12) -> float:
    """Zero out small stick deflections, then rescale the remaining range."""
    if abs(value) < deadzone:
        return 0.0
    # Rescale so the output goes smoothly from 0 at the deadzone edge to ±1
    sign = 1.0 if value > 0 else -1.0
    return sign * (abs(value) - deadzone) / (1.0 - deadzone)


# ---------------------------------------------------------------------------
# Main control loop
# ---------------------------------------------------------------------------
def main() -> None:
    parser = argparse.ArgumentParser(
        description="PS4 controller for 3-DOF robotic arm via CAN"
    )
    parser.add_argument("--cmd-host", default="127.0.0.1",
                        help="CmdServer host (default: 127.0.0.1)")
    parser.add_argument("--cmd-port", type=int, default=5555,
                        help="CmdServer TCP port (default: 5555)")
    parser.add_argument("--poll-rate", type=int, default=50,
                        help="Controller poll rate in Hz (default: 50)")
    parser.add_argument("--shoulder-speed", type=float, default=0.5,
                        help="Max shoulder velocity in rad/s (default: 0.5)")
    parser.add_argument("--elbow-speed", type=float, default=0.5,
                        help="Max elbow velocity in rad/s (default: 0.5)")
    parser.add_argument("--waist-speed", type=float, default=0.5,
                        help="Max waist velocity in rad/s (default: 0.5)")
    parser.add_argument("--deadzone", type=float, default=0.12,
                        help="Stick deadzone threshold 0–1 (default: 0.12)")
    args = parser.parse_args()

    poll_interval = 1.0 / args.poll_rate

    # --- Connect to CmdServer ---
    client = CmdClient(host=args.cmd_host, port=args.cmd_port)
    print(f"Connecting to CmdServer at {args.cmd_host}:{args.cmd_port} ...")
    if client.is_connected():
        print("  Connected!")
    else:
        print("  WARNING: CmdServer not reachable. Commands will fail until it's up.")
        print("  Make sure can_logger.py is running with the command server enabled.")

    # --- Init pygame + joystick ---
    pygame.init()
    pygame.joystick.init()

    if pygame.joystick.get_count() == 0:
        print("\nERROR: No controller detected.")
        print("  Connect a PS4 controller via USB or Bluetooth and try again.")
        pygame.quit()
        sys.exit(1)

    js = pygame.joystick.Joystick(0)
    js.init()
    print(f"\nController: {js.get_name()}")
    print(f"  Axes: {js.get_numaxes()}, Buttons: {js.get_numbuttons()}")
    print(f"  HATs: {js.get_numhats()}")
    print()
    print("=== Control Mapping ===")
    print(f"  Left stick Y   → Shoulder (ESC {DEV_SHOULDER})  max {args.shoulder_speed} rad/s")
    print(f"  Left stick X   → Waist    (ESC {DEV_WAIST})  max {args.waist_speed} rad/s")
    print(f"  Right stick Y  → Elbow    (ESC {DEV_ELBOW})  max {args.elbow_speed} rad/s")
    print(f"  L1/R1 bumpers  → (unassigned)")
    print(f"  Square         → Acknowledge faults (all ESCs)")
    print(f"  Circle         → STOP all motors")
    print(f"  PS / Home      → Quit")
    print(f"\n  Poll rate: {args.poll_rate} Hz  |  Deadzone: {args.deadzone}")
    print("=" * 40)
    print("\nRunning... press PS/Home or Ctrl+C to exit.\n")

    # Track previous commands to only send on change
    prev_shoulder_vel = None
    prev_elbow_vel    = None
    prev_waist_vel    = None

    # Minimum change to trigger a new CAN frame (avoids flooding)
    vel_change_threshold = 0.005  # rad/s

    running = True
    try:
        while running:
            loop_start = time.monotonic()

            # Process pygame events (required for joystick updates)
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False

                elif event.type == pygame.JOYBUTTONDOWN:
                    # Square → acknowledge faults on all 3 ESCs
                    if event.button == BUTTON_SQUARE:
                        print("  [SQUARE] Acknowledge faults → all ESCs")
                        send_acknowledge_faults(client, DEV_WAIST)
                        send_acknowledge_faults(client, DEV_SHOULDER)
                        send_acknowledge_faults(client, DEV_ELBOW)

                    # Circle → emergency stop all
                    elif event.button == BUTTON_CIRCLE:
                        print("  [CIRCLE] STOP → all ESCs")
                        send_stop(client, DEV_WAIST)
                        send_stop(client, DEV_SHOULDER)
                        send_stop(client, DEV_ELBOW)
                        prev_shoulder_vel = 0.0
                        prev_elbow_vel = 0.0
                        prev_waist_vel = 0.0

                    # PS / Home → quit
                    elif event.button == BUTTON_PS:
                        print("  [PS] Quitting...")
                        running = False

            if not running:
                break

            # --- Read analog sticks ---
            raw_left_y  = js.get_axis(AXIS_LEFT_Y)
            raw_right_y = js.get_axis(AXIS_RIGHT_Y)

            # Apply deadzone and scale to max velocity
            # Stick up (negative axis) = positive velocity (forward rotation)
            shoulder_vel = -apply_deadzone(raw_left_y, args.deadzone) * args.shoulder_speed
            elbow_vel    = -apply_deadzone(raw_right_y, args.deadzone) * args.elbow_speed

            # --- Read left stick X for waist rotation ---
            raw_left_x = js.get_axis(AXIS_LEFT_X)
            waist_vel = apply_deadzone(raw_left_x, args.deadzone) * args.waist_speed

            # --- Send velocity commands (only when changed) ---
            def vel_changed(new: float, old: float | None) -> bool:
                if old is None:
                    return True
                return abs(new - old) > vel_change_threshold

            if vel_changed(shoulder_vel, prev_shoulder_vel):
                send_velocity(client, DEV_SHOULDER, shoulder_vel)
                prev_shoulder_vel = shoulder_vel

            if vel_changed(elbow_vel, prev_elbow_vel):
                send_velocity(client, DEV_ELBOW, elbow_vel)
                prev_elbow_vel = elbow_vel

            if vel_changed(waist_vel, prev_waist_vel):
                send_velocity(client, DEV_WAIST, waist_vel)
                prev_waist_vel = waist_vel

            # --- Print live status (overwrite same line) ---
            status = (
                f"  SH: {shoulder_vel:+.3f}  |  "
                f"EL: {elbow_vel:+.3f}  |  "
                f"WA: {waist_vel:+.3f}"
            )
            print(f"\r{status}", end="", flush=True)

            # --- Rate limit ---
            elapsed = time.monotonic() - loop_start
            sleep_time = poll_interval - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("\n\nCtrl+C received.")

    # --- Cleanup: stop all motors ---
    print("\nStopping all motors...")
    send_velocity(client, DEV_SHOULDER, 0.0)
    send_velocity(client, DEV_ELBOW, 0.0)
    send_velocity(client, DEV_WAIST, 0.0)
    time.sleep(0.05)
    send_stop(client, DEV_SHOULDER)
    send_stop(client, DEV_ELBOW)
    send_stop(client, DEV_WAIST)

    client.close()
    pygame.quit()
    print("Done.")


if __name__ == "__main__":
    main()