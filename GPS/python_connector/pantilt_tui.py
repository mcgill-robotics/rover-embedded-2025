"""
Curses TUI for PanTiltGPS.
Run: python pantilt_tui.py [port] [baud]

Arrow keys  pan/tilt
+/-         step size
ESC         quit
Any other key forwarded raw to the secondary UART
"""

import curses
import sys

from pantilt_firmware import PanTiltGPS


def main(stdscr, port, baud):
    """
    Runs the curses TUI loop.

    Parameters
    ----------
    stdscr : curses.window
        Curses window, provided by curses.wrapper.
    port : str
        USB CDC port to connect to.
    baud : int
        Baud rate of the connection.
    """

    board = PanTiltGPS(port, baud, True)
    try:
        board.connect()
    except ConnectionError as e:
        stdscr.addstr(0, 0, str(e) + "  (press any key to exit)")
        stdscr.getch()
        return

    curses.curs_set(0)
    stdscr.nodelay(True)
    stdscr.timeout(50)

    step = 5.0
    term_log = ""

    while True:
        board.run()
        term_log += board.read_terminal().decode("utf-8", errors="replace")
        term_log = term_log[-2000:]

        stdscr.erase()
        sats, lat, lon, heading = board.get_gps()
        pan_angle, tilt_angle = board.get_pantilt()
        gps1_ok, gps1_err = board.get_gps_diag()
        ptx_drop, ttx_drop, usb_drop, uart_err = board.get_drop_counts()
        gps_startups, pantilt_startups = board.get_startup_count()
        gps_line = f"sats={sats:.0f}  lat={lat:.6f}  lon={lon:.6f}  heading={heading:.1f}"
        stdscr.addstr(0, 0, f"GPS   {gps_line}  ({'locked' if board.is_gps_connected() else 'no lock'})")
        stdscr.addstr(1, 0, f"PAN/TILT  pan={pan_angle:.1f}  tilt={tilt_angle:.1f}  step={step:.1f}")
        stdscr.addstr(2, 0, f"GPS RX  ok={gps1_ok} err={gps1_err}  drops: ptx={ptx_drop} ttx={ttx_drop} usb={usb_drop} uart_err={uart_err}")
        stdscr.addstr(3, 0, f"STARTUPS  gps={gps_startups} pantilt={pantilt_startups}")
        stdscr.addstr(4, 0, "arrows=pan/tilt  +/-=step  ESC=quit  other keys forwarded to terminal")
        for i, line in enumerate(term_log.splitlines()[-(curses.LINES - 6):]):
            stdscr.addstr(6 + i, 0, line[: curses.COLS - 1])

        stdscr.refresh()

        ch = stdscr.getch()
        if ch == -1:
            continue

        if ch == 27:  # ESC
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
        elif 0 <= ch < 256:
            data = bytes([ch])
            board.write_terminal(data)
            term_log += data.decode("utf-8", errors="replace")


if __name__ == "__main__":
    port = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"
    baud = int(sys.argv[2]) if len(sys.argv) > 2 else 115200
    curses.wrapper(main, port, baud)
