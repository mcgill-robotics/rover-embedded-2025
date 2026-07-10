"""
CANable (slcan firmware) CAN receiver for Windows
Receives classic CAN frames from STM32 FDCAN2 at 500 kbps

Usage:
    pip install python-can
    python can_receiver.py --port COM3

Change --port to match your Device Manager COM port.
"""

import can
import argparse
import struct
import time


# ── Configuration ────────────────────────────────────────────────────────────

BAUD_RATE   = 500_000   # Must match STM32 NominalBaudRate (500 kbps)
SLCAN_BAUD  = 115200    # Serial port baud rate for slcan firmware


# ── CAN ID parsing ────────────────────────────────────────────────────────────
# Mirrors your ParsedCANID struct on the STM32 side.
# Adjust the bit layout below if your sendCANResponse() packs fields differently.

def parse_can_id(can_id: int) -> dict:
    """Break the 11-bit standard CAN ID into your custom fields."""
    return {
        "raw":           can_id,
        "messageSender": (can_id >> 10) & 0x1,
        "motorType":     (can_id >> 8)  & 0x3,
        "motorConfig":   (can_id >> 6)  & 0x3,
        "commandType":   (can_id >> 5)  & 0x1,
        "readSpec":      (can_id >> 4)  & 0x1,
        "runSpec":       (can_id >> 3)  & 0x1,
        "motorID":       (can_id >> 0)  & 0x7,
    }


def parse_payload(data: bytes) -> float | None:
    """
    Attempt to decode the payload as a little-endian float (4 bytes).
    Your sendCANResponse() sends a float — adjust if your format differs.
    """
    if len(data) >= 4:
        return struct.unpack_from("<f", data, 0)[0]
    return None


# ── Main ──────────────────────────────────────────────────────────────────────

def main(port: str):
    print(f"Connecting to CANable on {port} at {BAUD_RATE} bps...")

    bus = can.Bus(
        interface="slcan",
        channel=port,
        bitrate=BAUD_RATE,
        ttyBaudrate=SLCAN_BAUD,
    )

    print(f"Listening for CAN frames — Ctrl+C to stop\n")
    print(f"{'Time':>10}  {'ID':>5}  {'DLC':>3}  {'Data':<24}  {'Float':>10}  Fields")
    print("-" * 90)

    try:
        while True:
            msg = bus.recv(timeout=1.0)

            if msg is None:
                print("  [waiting...]")
                continue

            # Raw hex data
            hex_data = " ".join(f"{b:02X}" for b in msg.data)

            # Try to decode float payload
            value = parse_payload(msg.data)
            float_str = f"{value:.4f}" if value is not None else "N/A"

            # Parse your custom CAN ID fields
            fields = parse_can_id(msg.arbitration_id)
            field_str = (
                f"sender={fields['messageSender']} "
                f"motorType={fields['motorType']} "
                f"motorID={fields['motorID']} "
                f"cmd={fields['commandType']}"
            )

            print(
                f"{msg.timestamp:>10.3f}  "
                f"0x{msg.arbitration_id:03X}  "
                f"{msg.dlc:>3}  "
                f"{hex_data:<24}  "
                f"{float_str:>10}  "
                f"{field_str}"
            )

    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        bus.shutdown()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="CANable slcan receiver")
    parser.add_argument(
        "--port", default="COM10",
        help="Windows COM port for CANable (default: COM3)"
    )
    args = parser.parse_args()
    main(args.port)