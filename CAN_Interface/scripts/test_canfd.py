"""
test_ping.py — Send a GET_PING read command to the ESC and verify it responds with 69.0

Usage:
    python test_ping.py              # defaults: COM3, device_id=0
    python test_ping.py --port COM5
    python test_ping.py --port COM5 --id 2
"""

import argparse
import can

from esc_can.protocol import (
    ARB_BITRATE,
    DATA_BITRATE,
    Action,
    ReadSpec,
    Sender,
    build_single_read_frame,
    decode_can_id,
    unpack_single_float,
)

PING_EXPECTED = 69.0
TIMEOUT_S = 2.0


def open_bus(port: str) -> can.BusABC:
    return can.interface.Bus(
        interface="slcan",
        channel=f"{port}@115200",
        bitrate=ARB_BITRATE,
        data_bitrate=DATA_BITRATE,
        fd=True,
        ignore_config=True,
    )


def main() -> None:
    parser = argparse.ArgumentParser(description="Ping an ESC over CAN FD")
    parser.add_argument("--port", default="COM4", help="CANable serial port (default: COM3)")
    parser.add_argument("--id", type=int, default=0, help="Target ESC device ID 0–15 (default: 0)")
    args = parser.parse_args()

    # device_id: int = args.id
    device_id = 1
    arb_id, data = build_single_read_frame(device_id=device_id, read_spec=ReadSpec.PING)

    print(f"Opening CAN bus on {args.port} …")
    bus = open_bus(args.port)

    msg = can.Message(
        arbitration_id=arb_id,
        data=data,
        is_fd=True,
        bitrate_switch=True,
    )

    print(f"TX  →  id=0x{arb_id:03X}  {decode_can_id(arb_id)}")
    bus.send(msg)

    # Wait for the ESC's response
    # The ESC mirrors the ID with bit 10 set to 1 (SLAVE)
    expected_resp_id = arb_id | (1 << 10)

    print(f"Waiting for response (expecting id=0x{expected_resp_id:03X}) …")
    while True:
        rx = bus.recv(timeout=TIMEOUT_S)
        if rx is None:
            print("TIMEOUT — no response from ESC.")
            bus.shutdown()
            return

        parsed = decode_can_id(rx.arbitration_id)

        # Ignore frames that aren't the ESC's reply to our ping
        if rx.arbitration_id != expected_resp_id:
            print(f"  (ignoring id=0x{rx.arbitration_id:03X}  {parsed})")
            continue

        value = unpack_single_float(rx.data)
        print(f"RX  ←  id=0x{rx.arbitration_id:03X}  {parsed}")
        print(f"       payload float = {value}")

        if value == PING_EXPECTED:
            print(f"\n✓  PING OK — ESC {device_id} responded with {value}")
        else:
            print(f"\n✗  PING FAIL — expected {PING_EXPECTED}, got {value}")
        break

    bus.shutdown()


if __name__ == "__main__":
    main()