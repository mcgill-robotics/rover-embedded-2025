"""
Curses TUI for PanTiltGPS.
Run: python pantilt_tui.py [port] [baud]

Control mode:
    Arrow keys  pan/tilt
    +/-         step size
    t           switch to terminal mode
    q           quit

Terminal mode:
    Any key     forwarded raw to secondary UART
    ESC         back to control mode
"""

import curses
import sys

from pantilt_firmware import PanTiltGPS

CONTROL, TERMINAL = "control", "terminal"


def main(stdscr, port, baud):
    board = PanTiltGPS(port, baud)
    try:
        board.connect()
    except ConnectionError as e:
        stdscr.addstr(0, 0, str(e) + "  (press any key to exit)")
        stdscr.getch()
        return

    curses.curs_set(0)
    stdscr.nodelay(True)
    stdscr.timeout(50)

    mode = CONTROL
    step = 5.0
    term_log = ""

    while True:
        board.run()

        stdscr.erase()
        gps_line = f"sats={board.gps_sats:.0f}  lat={board.coords[0]:.6f}  lon={board.coords[1]:.6f}  heading={board.heading:.1f}"
        stdscr.addstr(0, 0, f"GPS   {gps_line}  ({'locked' if board.is_gps_connected() else 'no lock'})")
        stdscr.addstr(1, 0, f"PAN/TILT  pan={board.pan_angle:.1f}  tilt={board.tilt_angle:.1f}  step={step:.1f}")
        stdscr.addstr(2, 0, f"GPS RX  gps1 ok={board.gps1_valid_frames} err={board.gps1_error_frames}   gps2 ok={board.gps2_valid_frames} err={board.gps2_error_frames}")
        board_mode = "terminal" if board.secondary_mode == "term" else "control"
        mismatch = "  (!! board disagrees, switch may not have taken effect)" if board_mode != mode else ""
        stdscr.addstr(3, 0, f"MODE  {mode}  (board confirms: {board_mode}){mismatch}")

        if mode == CONTROL:
            stdscr.addstr(5, 0, "arrows=pan/tilt  +/-=step  t=terminal mode  q=quit")
        else:
            term_log += board.read_terminal().decode("utf-8", errors="replace")
            term_log = term_log[-2000:]
            stdscr.addstr(5, 0, "keys forwarded raw to secondary UART  ESC=back to control mode")
            for i, line in enumerate(term_log.splitlines()[-(curses.LINES - 7):]):
                stdscr.addstr(7 + i, 0, line[: curses.COLS - 1])

        stdscr.refresh()

        ch = stdscr.getch()
        if ch == -1:
            continue

        if mode == CONTROL:
            if ch == ord("q"):
                break
            elif ch == curses.KEY_LEFT:
                board.add_pan_angle(-step)
            elif ch == curses.KEY_RIGHT:
                board.add_pan_angle(step)
            elif ch == curses.KEY_UP:
                board.add_tilt_angle(step)
            elif ch == curses.KEY_DOWN:
                board.add_tilt_angle(-step)
            elif ch in (ord("+"), ord("=")):
                step += 1.0
            elif ch in (ord("-"), ord("_")):
                step = max(1.0, step - 1.0)
            elif ch == ord("t"):
                board.set_secondary_mode("term")
                mode = TERMINAL
                term_log = ""
        else:
            if ch == 27: # ESC
                board.set_secondary_mode("gps")
                mode = CONTROL
            elif 0 <= ch < 256:
                data = bytes([ch])
                board.write_terminal(data)
                term_log += data.decode("utf-8", errors="replace")


if __name__ == "__main__":
    port = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"
    baud = int(sys.argv[2]) if len(sys.argv) > 2 else 115200
    curses.wrapper(main, port, baud)
