"""
test_read_all.py — Read all implemented fields from an ESC over CAN FD

Sends each ReadSpec command one at a time, waits for the ESC response,
and prints the decoded float value.

Usage:
    python test_read_all.py                       # defaults: COM4, device_id=1
    python test_read_all.py --port COM5
    python test_read_all.py --port COM5 --id 2
"""

import argparse
import time

import can

from esc_can.protocol import (
    ARB_BITRATE,
    DATA_BITRATE,
    ReadSpec,
    build_single_read_frame,
    decode_can_id,
    unpack_single_float,
)

TIMEOUT_S = 2.0

# Each entry: (ReadSpec value, label, optional expected value for validation)
READ_FIELDS = [
    (ReadSpec.PING,          "Ping",           69.0),
    (ReadSpec.TEMPERATURE,   "Temperature (°C)", None),
    (ReadSpec.VOLTAGE,       "VBUS Voltage (V)", None),
    (ReadSpec.POSITION,      "Position (deg)",   None),
    (ReadSpec.CURRENT,       "Current (A)",      None),
    (ReadSpec.CURRENT_STATE, "Motor State",       None),
    (ReadSpec.CONTROL_MODE,  "Control Mode",      None),
    (ReadSpec.CALIBRATION,   "Calibration",       None),
]


def open_bus(port: str) -> can.BusABC:
    return can.interface.Bus(
        interface="slcan",
        channel=f"{port}@115200",
        bitrate=ARB_BITRATE,
        data_bitrate=DATA_BITRATE,
        fd=True,
        ignore_config=True,
    )


def flush_rx(bus: can.BusABC):
    """Drain any stale frames sitting in the receive buffer."""
    while bus.recv(timeout=0.01) is not None:
        pass


def read_field(bus: can.BusABC, device_id: int, spec: ReadSpec, label: str):
    """Send a single read request and return (value, round_trip_ms) or (None, None) on timeout."""
    arb_id, data = build_single_read_frame(device_id=device_id, read_spec=spec)
    expected_resp_id = arb_id | (1 << 10)  # ESC sets sender bit = SLAVE

    flush_rx(bus)

    msg = can.Message(
        arbitration_id=arb_id,
        data=data,
        is_fd=True,
        bitrate_switch=True,
    )

    t_send = time.perf_counter()
    bus.send(msg)

    # Wait for the matching response
    deadline = time.time() + TIMEOUT_S
    while time.time() < deadline:
        rx = bus.recv(timeout=0.25)
        if rx is None:
            continue
        if rx.arbitration_id == expected_resp_id:
            t_recv = time.perf_counter()
            rtt_ms = (t_recv - t_send) * 1000.0
            return unpack_single_float(rx.data), rtt_ms
        # else: ignore unrelated frames

    return None, None


def main() -> None:
    parser = argparse.ArgumentParser(description="Read all fields from an ESC over CAN FD")
    parser.add_argument("--port", default="COM4", help="CANable serial port (default: COM4)")
    parser.add_argument("--id", type=int, default=1, help="Target ESC device ID 0–15 (default: 1)")
    args = parser.parse_args()

    device_id: int = args.id

    print(f"Opening CAN bus on {args.port} …")
    bus = open_bus(args.port)

    print(f"{'=' * 55}")
    print(f"  Reading all fields from ESC {device_id}")
    print(f"{'=' * 55}")

    passed = 0
    failed = 0
    errors = 0
    rtts = []

    for spec, label, expected in READ_FIELDS:
        arb_id, _ = build_single_read_frame(device_id=device_id, read_spec=spec)
        print(f"\n  TX  0x{arb_id:03X}  [{label}]")

        value, rtt_ms = read_field(bus, device_id, spec, label)

        if value is None:
            print(f"  RX  TIMEOUT — no response")
            errors += 1
            continue

        resp_id = arb_id | (1 << 10)
        print(f"  RX  0x{resp_id:03X}  → {label} = {value}    ({rtt_ms:.2f} ms)")
        rtts.append(rtt_ms)

        # Flag ADC error sentinel
        if value == -999.0:
            print(f"       ⚠  ADC conversion error (sentinel -999.0)")
            errors += 1
            continue

        # Validate if we have an expected value
        if expected is not None:
            if value == expected:
                print(f"       ✓  matches expected {expected}")
                passed += 1
            else:
                print(f"       ✗  expected {expected}, got {value}")
                failed += 1
        else:
            passed += 1

        time.sleep(0.05)

    print(f"\n{'=' * 55}")
    print(f"  Results: {passed} ok, {failed} fail, {errors} error/timeout")
    if rtts:
        print(f"  Round-trip:  min {min(rtts):.2f} ms  "
              f"avg {sum(rtts)/len(rtts):.2f} ms  "
              f"max {max(rtts):.2f} ms")
    print(f"{'=' * 55}")

    bus.shutdown()


if __name__ == "__main__":
    main()