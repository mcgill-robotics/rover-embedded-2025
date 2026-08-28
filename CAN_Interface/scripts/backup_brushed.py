#!/usr/bin/env python3
"""
arm_teleop_ssh.py — terminal (curses) teleop for the STM32 3-motor board.

Works headless / over SSH — no X server, no pynput. Only needs pyserial.

    pip install pyserial
    python arm_teleop_ssh.py --port /dev/ttyACM0

How hold-to-run works in a terminal:
    A terminal cannot detect key RELEASES, only repeated characters while a
    key is held (OS auto-repeat). So: each repeat re-arms the motor, and a
    timeout stops it ~0.6 s after repeats cease. Expect a short stutter when
    you first press (the auto-repeat initial delay) and a ~0.6 s coast after
    release. For crisper stops, pair this with the firmware watchdog.

Keys:
    Up / Down     -> wrist PITCH  +/-
    Left / Right  -> GRIPPER      +/-
    ,  /  .       -> wrist ROLL   +/-
    [  /  ]       -> speed setpoint -/+ 100
    Space         -> EMERGENCY STOP (all motors)
    q             -> stop all and quit

Protocol (same as arm_teleop.py / usb_comms.c):
    "M<idx> <speed>\n" -> "OK" | "S\n" -> "OK" | "P\n" -> "PONG" | "?\n" -> status
"""

import argparse
import curses
import sys
import time

import serial

MOTOR_WRIST_ROLL = 0
MOTOR_WRIST_PITCH = 1
MOTOR_GRIPPER = 2

MOTOR_NAMES = {
    MOTOR_WRIST_ROLL: "roll",
    MOTOR_WRIST_PITCH: "pitch",
    MOTOR_GRIPPER: "grip",
}

# How long after the last key repeat before we stop that motor (seconds).
# Must exceed the OS auto-repeat initial delay (typically ~0.5 s).
HOLD_TIMEOUT = 0.05


class ArmLink:
    def __init__(self, port: str, baud: int = 115200, timeout: float = 0.2):
        self.ser = serial.Serial(port, baud, timeout=timeout)
        time.sleep(0.5)
        self.ser.reset_input_buffer()

    def _send(self, cmd: str) -> str:
        self.ser.write((cmd + "\n").encode("ascii"))
        return self.ser.readline().decode("ascii", errors="replace").strip()

    def ping(self) -> bool:
        return self._send("P") == "PONG"

    def set_motor(self, motor: int, speed: int) -> bool:
        speed = max(-1000, min(1000, int(speed)))
        return self._send(f"M{motor} {speed}") == "OK"

    def stop_all(self) -> bool:
        return self._send("S") == "OK"

    def close(self):
        try:
            self.stop_all()
        finally:
            self.ser.close()


def teleop(stdscr, arm: ArmLink, speed: int):
    curses.cbreak()
    stdscr.nodelay(True)      # non-blocking getch()
    stdscr.keypad(True)       # decode arrow keys into KEY_UP etc.

    keymap = {
        curses.KEY_UP:    (MOTOR_WRIST_PITCH, +1),
        curses.KEY_DOWN:  (MOTOR_WRIST_PITCH, -1),
        curses.KEY_LEFT:  (MOTOR_GRIPPER,     +1),
        curses.KEY_RIGHT: (MOTOR_GRIPPER,     -1),
        ord(","):         (MOTOR_WRIST_ROLL,  +1),
        ord("."):         (MOTOR_WRIST_ROLL,  -1),
    }

    # motor -> (direction, last_seen_timestamp)
    active = {}

    def hud(msg=""):
        stdscr.erase()
        stdscr.addstr(0, 0, "STM32 arm teleop (SSH mode) — q quits, Space = e-stop")
        stdscr.addstr(1, 0, "Hold arrows / , . to move (release = stop after ~0.6 s)")
        stdscr.addstr(3, 0, f"speed setpoint: {speed:4d}   ([ / ] to change)")
        line = "active: " + "  ".join(
            f"{MOTOR_NAMES[m]}{'+' if d > 0 else '-'}"
            for m, (d, _) in sorted(active.items())
        )
        stdscr.addstr(4, 0, line if active else "active: (none)")
        if msg:
            stdscr.addstr(6, 0, msg)
        stdscr.refresh()

    hud("ready")

    running = True
    while running:
        now = time.monotonic()

        # --- drain all pending keypresses -------------------------------
        while True:
            key = stdscr.getch()
            if key == -1:
                break

            if key in (ord("q"), ord("Q")):
                running = False
                break

            if key == ord(" "):
                active.clear()
                arm.stop_all()
                hud("E-STOP — all motors stopped")
                continue

            if key == ord("["):
                speed = max(100, speed - 100)
                hud()
                continue
            if key == ord("]"):
                speed = min(1000, speed + 100)
                hud()
                continue

            if key in keymap:
                motor, direction = keymap[key]
                prev = active.get(motor)
                active[motor] = (direction, now)
                # Only transmit when the commanded state changes; repeats
                # just refresh the timestamp.
                if prev is None or prev[0] != direction:
                    arm.set_motor(motor, direction * speed)
                    hud()

        # --- stop motors whose key repeats have ceased ------------------
        expired = [m for m, (_, t) in active.items() if now - t > HOLD_TIMEOUT]
        for m in expired:
            del active[m]
            arm.set_motor(m, 0)
        if expired:
            hud()

        time.sleep(0.02)   # ~50 Hz loop; keeps CPU usage negligible

    arm.stop_all()


def main():
    ap = argparse.ArgumentParser(description="STM32 3-motor teleop (SSH/curses)")
    ap.add_argument("--port", required=True, help="e.g. /dev/ttyACM0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--speed", type=int, default=500, help="initial setpoint 100..1000")
    args = ap.parse_args()

    try:
        arm = ArmLink(args.port, args.baud)
    except serial.SerialException as e:
        sys.exit(f"Could not open {args.port}: {e}")

    if not arm.ping():
        print("Warning: no PONG from the board — check firmware/port/baud.")
        time.sleep(1.5)

    try:
        curses.wrapper(teleop, arm, args.speed)
    finally:
        arm.close()
        print("Stopped all motors. Bye.")


if __name__ == "__main__":
    main()