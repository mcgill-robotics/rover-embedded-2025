"""
can_logger.py — Capture all CAN FD bus traffic to SQLite

Listens on the CANable adapter and logs every frame (telemetry broadcasts,
request-response, master commands) into a timestamped SQLite database using
the esc_can.datalogger module.

Usage:
    python can_logger.py                              # defaults: COM4, can_log.db
    python can_logger.py --port COM5
    python can_logger.py --port COM5 --db session.db
    python can_logger.py --port /dev/ttyACM0 --db session.db --desc "PID tuning run"

Press Ctrl+C to stop.  The database is flushed and closed cleanly on exit.
"""

import argparse
import signal
import sys
import time

import can

from esc_can.protocol import ARB_BITRATE, DATA_BITRATE, decode_can_id
from esc_can.datalogger import CANDataLogger


def open_bus(port: str) -> can.BusABC:
    return can.interface.Bus(
        interface="slcan",
        channel=f"{port}@115200",
        bitrate=ARB_BITRATE,
        data_bitrate=DATA_BITRATE,
        fd=True,
        ignore_config=True,
    )


def print_status(logger: CANDataLogger, elapsed: float) -> None:
    """Print a one-line status update (overwrites the current line)."""
    s = logger.stats
    fps = s["total_logged"] / elapsed if elapsed > 0 else 0
    sys.stdout.write(
        f"\r  Frames: {s['total_logged']:>8,}  |  "
        f"Flushed: {s['total_flushed']:>8,}  |  "
        f"Pending: {s['pending']:>4}  |  "
        f"Rate: {fps:>6.1f} f/s  |  "
        f"Avg flush: {s['avg_flush_ms']:.2f} ms   "
    )
    sys.stdout.flush()


def main() -> None:
    parser = argparse.ArgumentParser(description="Log all CAN FD traffic to SQLite")
    parser.add_argument("--port", default="COM4",
                        help="CANable serial port (default: COM4)")
    parser.add_argument("--db", default="can_log.db",
                        help="SQLite database path (default: can_log.db)")
    parser.add_argument("--desc", default="",
                        help="Session description stored in the database")
    parser.add_argument("--batch", type=int, default=64,
                        help="Flush after this many frames (default: 64)")
    parser.add_argument("--flush-interval", type=float, default=0.25,
                        help="Max seconds between flushes (default: 0.25)")
    parser.add_argument("--quiet", action="store_true",
                        help="Suppress per-frame console output")
    args = parser.parse_args()

    print(f"Opening CAN bus on {args.port} ...")
    bus = open_bus(args.port)

    logger = CANDataLogger(
        db_path=args.db,
        session_desc=args.desc or f"Capture on {args.port}",
        batch_size=args.batch,
        flush_interval_s=args.flush_interval,
        auto_flush=True,
    )

    print(f"Logging to {args.db}  (session {logger.session_id})")
    print(f"Batch size: {args.batch}  |  Flush interval: {args.flush_interval} s")
    print(f"{'=' * 60}")
    print("  Listening for CAN frames — press Ctrl+C to stop")
    print(f"{'=' * 60}\n")

    # Graceful shutdown on Ctrl+C
    stop = False

    def on_signal(sig, frame):
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, on_signal)

    t_start = time.perf_counter()
    last_status = t_start

    try:
        while not stop:
            msg = bus.recv(timeout=0.1)
            if msg is None:
                # No frame received — still update status line periodically
                now = time.perf_counter()
                if now - last_status >= 1.0:
                    print_status(logger, now - t_start)
                    last_status = now
                continue

            logger.log_frame(msg, is_rx=True)

            # Print decoded frame to console (unless --quiet)
            if not args.quiet:
                parsed = decode_can_id(msg.arbitration_id)
                ts = time.perf_counter() - t_start
                print(
                    f"  [{ts:8.3f}s]  0x{msg.arbitration_id:03X}  "
                    f"{parsed.sender.name:>6} {parsed.action.name:>4} "
                    f"{parsed.motor_config.name:>8} "
                    f"dev={parsed.device_id:<2}  "
                    f"data={msg.data[:8].hex(' ')}"
                )

            # Periodic status update
            now = time.perf_counter()
            if now - last_status >= 2.0:
                print_status(logger, now - t_start)
                print()  # newline so frame output isn't clobbered
                last_status = now

    finally:
        print(f"\n\n{'=' * 60}")
        print("  Stopping ...")

        logger.close()
        bus.shutdown()

        summary = logger.get_session_summary()
        print(f"\n  Session {summary['session_id']} summary:")
        print(f"    Total frames : {summary['total_frames']:,}")
        print(f"    Duration     : {summary['duration_s']:.1f} s")
        print(f"    Average rate : {summary['avg_fps']:.1f} frames/sec")
        print(f"    ESCs seen    : {summary['esc_count']}")
        print(f"    Signals      : {summary['signal_count']}")
        print(f"    Database     : {args.db}")

        perf = logger.stats
        print(f"\n  Logger performance:")
        print(f"    Flushes      : {perf['flush_count']}")
        print(f"    Avg flush    : {perf['avg_flush_ms']:.3f} ms")
        print(f"{'=' * 60}")


if __name__ == "__main__":
    main()