"""
arm_teleop.py — PS4 controller teleoperation for the ESC arm (position-only)

Runs ALONGSIDE can_logger.py and can_dashboard.py as a third, standalone
process. It never opens the serial port. Instead it uses the same two
channels the dashboard already uses:

  * TX (send setpoints): a CANCommander connected to the CmdServer (TCP,
    localhost:5555) running inside can_logger.py. The logger owns the
    CANable and transmits our frames on the bus.

  * RX (read current joint angles): a read-only SQLite connection to the
    logger's database (WAL mode). The logger writes every ESC SLAVE
    POSITION telemetry frame there, so we can read live joint angles.

Safety model — why position-only:
  Every input, including the "velocity" style controls, is emitted as a
  RUN_POSITION frame. The firmware S-curve planner drives to and HOLDS the
  last commanded setpoint. So on any comms loss (script crash, Wi-Fi drop,
  controller unpair) the arm eases to the last target and stops, rather
  than continuing along a velocity. Because targets advance in small
  per-tick increments, the last setpoint is always ~= current position, so
  residual travel after a drop is tiny.

Control mapping (PS4):
  Left stick  Y  -> shoulder (id 9)  velocity   (deg/s, integrated to position)
  Right stick Y  -> elbow    (id 10) position   (stick->angle, slew-limited)
  L1 / R1 (hold) -> waist    (id 8)  hold-to-turn left / right
  X   (tap)      -> toggle teleop active/inactive (re-syncs on activate)
  Circle (tap)   -> emergency stop all arm joints + deactivate

Usage:
  # Terminal 1: logger owns the bus + runs CmdServer
  python scripts/can_logger.py --port COM4 --db can_log.db
  # (optional) Terminal 2: dashboard
  python scripts/can_dashboard.py --db can_log.db
  # Terminal 3: this teleop
  python scripts/arm_teleop.py --db can_log.db

  # First-time controller setup — print live axis/button indices:
  python scripts/arm_teleop.py --identify

Requires: pip install pygame
"""

from __future__ import annotations

import argparse
import json
import sqlite3
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

from esc_can.commander import CANCommander
from esc_can.protocol import MotorType, ReadSpec, MotorID


# ===========================================================================
# CONFIG — tune these to your hardware / controller
# ===========================================================================

# --- Arm joints -------------------------------------------------------------
# NOTE: ARM_MOTOR_TYPE must match how the ESC firmware matches the MotorType
# bit (bit 7) for the arm ESCs. The multi-arm payload builders in protocol.py
# use STEERING, so STEERING is the default here — but CONFIRM against your
# firmware's ESC_ID / motorType matching. If the arm doesn't respond, this is
# the first thing to flip to MotorType.DRIVE.
ARM_MOTOR_TYPE = MotorType.STEERING


@dataclass
class JointCfg:
    name: str
    dev_id: int
    lo_deg: float          # soft lower limit
    hi_deg: float          # soft upper limit
    mode: str              # "rate" or "absolute"
    max_dps: float         # rate mode: deg/s at full stick.
                           # absolute mode: max slew deg/s toward target.


JOINTS: dict[str, JointCfg] = {
    "waist":    JointCfg("waist",    3,    -180, 180.0, "rate",     20.0),
    "shoulder": JointCfg("shoulder", 1,  0,  90.0, "rate",     20.0),
    "elbow":    JointCfg("elbow",    2,    0, 110.0, "absolute", 20.0),
}

# Elbow (absolute mode): when the stick returns to center, hold the last
# angle instead of returning to 0. Set False for pure stick->angle mapping
# (release => slews back to 0). Hold-on-center is the safer/expected feel.
ELBOW_HOLD_ON_CENTER = True

# --- Loop / bus -------------------------------------------------------------
LOOP_HZ = 50.0
MIN_SEND_DELTA_DEG = 0.05   # don't resend a joint unless its target moved this much
BUS_CHECK_PERIOD_S = 0.5    # how often to verify the CmdServer is still reachable
IS_FD = False               # matches the working dashboard commander default

# --- Sync on enable ---------------------------------------------------------
SYNC_POLL_WAIT_S = 0.25     # wait after issuing read_position before reading DB
STALE_THRESHOLD_S = 1.0     # refuse to engage a joint whose POSITION is older than this

# --- PS4 controller mapping -------------------------------------------------
# Axis/button indices VARY by OS and driver (SDL/pygame on Linux-BT differs
# from Windows). Run with --identify to discover yours, then edit these.
AX_LEFT_Y = 1        # shoulder velocity
AX_RIGHT_Y = 3       # elbow position   (often 3 or 4 depending on driver)
BTN_X = 0            # toggle active     (SDL "cross")
BTN_CIRCLE = 1       # emergency stop
BTN_L1 = 4           # waist left  (hold)
BTN_R1 = 5           # waist right (hold)

INVERT_LEFT_Y = True     # push stick up => positive command
INVERT_RIGHT_Y = True
DEADZONE = 0.4          # ignore stick noise below this magnitude


# ===========================================================================
# DB access — read live joint angles from the logger's SQLite database
# ===========================================================================

def _open_db_ro(db_path: str) -> sqlite3.Connection:
    conn = sqlite3.connect(f"file:{db_path}?mode=ro", uri=True,
                           check_same_thread=False)
    conn.row_factory = sqlite3.Row
    conn.execute("PRAGMA journal_mode=WAL")
    return conn


def read_latest_position(conn: sqlite3.Connection,
                         dev_id: int) -> Optional[tuple[float, float]]:
    """Return (age_s, angle_deg) for a joint's newest SLAVE POSITION frame.

    ``age_s`` is measured against the newest frame in the whole table, so it
    stays in the logger's perf_counter clock domain (our own wall clock is a
    different domain and must not be mixed in). Returns None if there is no
    POSITION reading for the joint.
    """
    row = conn.execute(
        """
        SELECT timestamp_s, decoded_float FROM raw_frames
        WHERE device_id = ? AND spec_name = 'POSITION'
          AND sender = 'SLAVE' AND decoded_float IS NOT NULL
        ORDER BY frame_id DESC LIMIT 1
        """,
        (dev_id,),
    ).fetchone()
    if row is None:
        return None

    newest = conn.execute(
        "SELECT MAX(timestamp_s) AS t FROM raw_frames"
    ).fetchone()
    newest_ts = newest["t"] if newest and newest["t"] is not None else row["timestamp_s"]
    age = max(0.0, newest_ts - row["timestamp_s"])
    return age, float(row["decoded_float"])


# ===========================================================================
# Teleop core
# ===========================================================================

@dataclass
class _JointState:
    cfg: JointCfg
    target: float = 0.0        # commanded angle (deg)
    last_sent: float = field(default=float("nan"))


class ArmTeleop:
    def __init__(self, db_path: str, cmd_host: str, cmd_port: int,
                 is_fd: bool, status_file: Optional[str]) -> None:
        self._db = _open_db_ro(db_path)
        self._cmd = CANCommander(cmd_host=cmd_host, cmd_port=cmd_port,
                                 motor_type=ARM_MOTOR_TYPE, is_fd=is_fd)
        self._status_file = status_file

        self._joints = {name: _JointState(cfg) for name, cfg in JOINTS.items()}
        self._active = False

        self._js = None                 # pygame joystick (set in _init_input)
        self._prev_buttons: dict[int, bool] = {}
        self._last_bus_check = 0.0
        self._last_status_write = 0.0

    # -- input ---------------------------------------------------------------
    def _init_input(self) -> None:
        import pygame
        pygame.init()
        pygame.joystick.init()
        if pygame.joystick.get_count() == 0:
            raise RuntimeError("No controller detected. Pair the PS4 pad first.")
        self._js = pygame.joystick.Joystick(0)
        self._js.init()
        print(f"[teleop] Controller: {self._js.get_name()} "
              f"({self._js.get_numaxes()} axes, {self._js.get_numbuttons()} buttons)")

    def _axis(self, idx: int, invert: bool = False) -> float:
        try:
            v = self._js.get_axis(idx)
        except Exception:
            return 0.0
        if invert:
            v = -v
        return 0.0 if abs(v) < DEADZONE else v

    def _button(self, idx: int) -> bool:
        try:
            return bool(self._js.get_button(idx))
        except Exception:
            return False

    def _rising_edge(self, idx: int) -> bool:
        """True on the tick a button transitions from released to pressed."""
        now = self._button(idx)
        prev = self._prev_buttons.get(idx, False)
        self._prev_buttons[idx] = now
        return now and not prev

    # -- sync ----------------------------------------------------------------
    def _sync_to_arm(self) -> bool:
        """Poll fresh joint angles and seed targets. Returns False if unsafe."""
        # Ask each joint for a fresh POSITION so the DB has a current value.
        for js in self._joints.values():
            try:
                self._cmd.read_position(js.cfg.dev_id, motor_type=ARM_MOTOR_TYPE)
            except Exception as e:
                print(f"[teleop] read_position({js.cfg.name}) failed: {e}",
                      file=sys.stderr)
        time.sleep(SYNC_POLL_WAIT_S)

        ok = True
        for js in self._joints.values():
            res = read_latest_position(self._db, js.cfg.dev_id)
            if res is None:
                print(f"[teleop] SYNC FAIL: no POSITION telemetry for "
                      f"{js.cfg.name} (id {js.cfg.dev_id}).", file=sys.stderr)
                ok = False
                continue
            age, angle = res
            if age > STALE_THRESHOLD_S:
                print(f"[teleop] SYNC FAIL: {js.cfg.name} POSITION is stale "
                      f"({age:.2f}s old). Is the logger receiving telemetry?",
                      file=sys.stderr)
                ok = False
                continue
            js.target = _clamp(angle, js.cfg.lo_deg, js.cfg.hi_deg)
            js.last_sent = float("nan")
            print(f"[teleop] synced {js.cfg.name:8s} = {angle:+7.2f} deg "
                  f"(age {age*1000:.0f} ms)")
        return ok

    # -- activation ----------------------------------------------------------
    def _activate(self) -> None:
        if not self._cmd.bus_connected:
            print("[teleop] Cannot activate: CmdServer (logger) not reachable.",
                  file=sys.stderr)
            return
        print("[teleop] Activating — syncing to current arm position...")
        if not self._sync_to_arm():
            print("[teleop] Activation ABORTED (sync failed). Staying inactive.",
                  file=sys.stderr)
            return
        self._active = True
        print("[teleop] >>> TELEOP ACTIVE <<<")

    def _deactivate(self, reason: str = "") -> None:
        if self._active:
            print(f"[teleop] --- teleop inactive --- {reason}")
        self._active = False

    def _emergency_stop(self) -> None:
        print("[teleop] EMERGENCY STOP", file=sys.stderr)
        for js in self._joints.values():
            try:
                self._cmd.stop(js.cfg.dev_id, motor_type=ARM_MOTOR_TYPE)
            except Exception:
                pass
        self._deactivate("(e-stop)")

    # -- per-tick control ----------------------------------------------------
    def _update(self, dt: float) -> None:
        import pygame
        for ev in pygame.event.get():
            if ev.type == pygame.JOYDEVICEREMOVED:
                self._deactivate("(controller disconnected)")

        # Toggle / e-stop edges are handled whether or not we're active.
        if self._rising_edge(BTN_X):
            if self._active:
                self._deactivate("(X toggle)")
            else:
                self._activate()
        if self._rising_edge(BTN_CIRCLE):
            self._emergency_stop()

        # Keep edge state fresh for held buttons even while inactive.
        self._button(BTN_L1); self._button(BTN_R1)

        if not self._active:
            return

        # Periodically confirm the logger is still there.
        now = time.perf_counter()
        if now - self._last_bus_check > BUS_CHECK_PERIOD_S:
            self._last_bus_check = now
            if not self._cmd.bus_connected:
                self._deactivate("(lost CmdServer)")
                return

        # --- integrate inputs into position targets ---
        # Shoulder: velocity (rate mode)
        sh = self._joints["shoulder"]
        sh_v = self._axis(AX_LEFT_Y, INVERT_LEFT_Y) * sh.cfg.max_dps
        sh.target = _clamp(sh.target + sh_v * dt, sh.cfg.lo_deg, sh.cfg.hi_deg)

        # Waist: hold-to-turn (rate mode)
        wa = self._joints["waist"]
        wa_dir = (1.0 if self._button(BTN_R1) else 0.0) \
               - (1.0 if self._button(BTN_L1) else 0.0)
        wa.target = _clamp(wa.target + wa_dir * wa.cfg.max_dps * dt,
                           wa.cfg.lo_deg, wa.cfg.hi_deg)

        # Elbow: position (absolute mode, slew-limited)
        el = self._joints["elbow"]
        stick = self._axis(AX_RIGHT_Y, INVERT_RIGHT_Y)
        if el.cfg.mode == "absolute":
            in_deadzone = (stick == 0.0)
            if not (in_deadzone and ELBOW_HOLD_ON_CENTER):
                span = max(abs(el.cfg.lo_deg), abs(el.cfg.hi_deg))
                desired = _clamp(stick * span, el.cfg.lo_deg, el.cfg.hi_deg)
                el.target = _slew(el.target, desired, el.cfg.max_dps * dt)
        else:  # rate
            el.target = _clamp(el.target + stick * el.cfg.max_dps * dt,
                               el.cfg.lo_deg, el.cfg.hi_deg)

        # --- stream setpoints (only when they meaningfully changed) ---
        for js in self._joints.values():
            if _changed(js.target, js.last_sent, MIN_SEND_DELTA_DEG):
                self._cmd.set_position(js.cfg.dev_id, js.target,
                                       motor_type=ARM_MOTOR_TYPE)
                js.last_sent = js.target

        self._write_status(now)

    def _write_status(self, now: float) -> None:
        if not self._status_file or now - self._last_status_write < 0.2:
            return
        self._last_status_write = now
        payload = {
            "active": self._active,
            "ts": time.time(),
            "targets_deg": {n: round(j.target, 2)
                            for n, j in self._joints.items()},
        }
        try:
            Path(self._status_file).write_text(json.dumps(payload))
        except OSError:
            pass

    # -- main loop -----------------------------------------------------------
    def run(self) -> None:
        self._init_input()
        if not self._cmd.bus_connected:
            print("[teleop] WARNING: CmdServer not reachable yet — is "
                  "can_logger.py running? You can still press X to try.",
                  file=sys.stderr)
        print("[teleop] Ready. Press X to activate/deactivate, Circle to e-stop.")
        period = 1.0 / LOOP_HZ
        prev = time.perf_counter()
        try:
            while True:
                start = time.perf_counter()
                dt = start - prev
                prev = start
                self._update(dt)
                sleep = period - (time.perf_counter() - start)
                if sleep > 0:
                    time.sleep(sleep)
        except KeyboardInterrupt:
            print("\n[teleop] shutting down.")
        finally:
            self._deactivate("(exit)")
            self._db.close()


# ===========================================================================
# Small helpers
# ===========================================================================

def _clamp(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else hi if v > hi else v


def _slew(current: float, target: float, max_step: float) -> float:
    delta = target - current
    if delta > max_step:
        return current + max_step
    if delta < -max_step:
        return current - max_step
    return target


def _changed(a: float, b: float, thresh: float) -> bool:
    if b != b:  # b is NaN => never sent yet
        return True
    return abs(a - b) >= thresh


# ===========================================================================
# Controller identify mode
# ===========================================================================

def identify_controller() -> None:
    import pygame
    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        print("No controller detected.")
        return
    js = pygame.joystick.Joystick(0)
    js.init()
    print(f"Controller: {js.get_name()}")
    print(f"  axes={js.get_numaxes()} buttons={js.get_numbuttons()} "
          f"hats={js.get_numhats()}")
    print("Move sticks / press buttons to see indices. Ctrl-C to quit.\n")
    try:
        while True:
            pygame.event.pump()
            for i in range(js.get_numaxes()):
                v = js.get_axis(i)
                if abs(v) > 0.5:
                    print(f"  AXIS {i} = {v:+.2f}")
            for i in range(js.get_numbuttons()):
                if js.get_button(i):
                    print(f"  BUTTON {i} pressed")
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\ndone.")


# ===========================================================================
# Entry point
# ===========================================================================

def main() -> None:
    p = argparse.ArgumentParser(description="PS4 arm teleop (position-only).")
    p.add_argument("--db", default="can_log.db",
                   help="SQLite DB path (must match can_logger.py).")
    p.add_argument("--cmd-host", default="127.0.0.1",
                   help="CmdServer host (logger process).")
    p.add_argument("--cmd-port", type=int, default=5555,
                   help="CmdServer TCP port (default 5555).")
    p.add_argument("--fd", action="store_true",
                   help="Send frames as CAN FD (default off, matches dashboard).")
    p.add_argument("--status-file", default=None,
                   help="Optional path to write a JSON heartbeat the dashboard "
                        "could poll to show a REMOTE LIVE badge.")
    p.add_argument("--identify", action="store_true",
                   help="Print live axis/button indices and exit.")
    args = p.parse_args()

    if args.identify:
        identify_controller()
        return

    if not Path(args.db).exists():
        print(f"WARNING: DB '{args.db}' not found yet — start can_logger.py "
              f"first, or sync will fail.", file=sys.stderr)

    teleop = ArmTeleop(
        db_path=args.db,
        cmd_host=args.cmd_host,
        cmd_port=args.cmd_port,
        is_fd=args.fd or IS_FD,
        status_file=args.status_file,
    )
    teleop.run()


if __name__ == "__main__":
    main()
