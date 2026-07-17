import time
import can

from esc_can.commander import CANCommander
from esc_can.protocol import (
    decode_can_id,
    unpack_single_float,
    ReadSpec,
    Sender,
    ARB_BITRATE,
    DATA_BITRATE,
)

# -----------------------------
# Your bus function
# -----------------------------
def open_bus(port: str) -> can.BusABC:
    return can.interface.Bus(
        interface="slcan",
        channel=f"{port}@115200",
        bitrate=ARB_BITRATE,
        data_bitrate=DATA_BITRATE,
        fd=True,
        ignore_config=True,
    )


DEVICE_ID = 1
PORT = "COM4"   # or "/dev/ttyACM0"


def wait_for_response(bus, expected_spec, timeout=1.0):
    """Generic wait for a READ response with given spec."""
    start = time.time()

    while time.time() - start < timeout:
        msg = bus.recv(timeout=0.05)
        if msg is None:
            continue

        parsed = decode_can_id(msg.arbitration_id)

        if (
            parsed.sender == Sender.SLAVE
            and parsed.spec == expected_spec
        ):
            value = unpack_single_float(msg.data)
            return value

    return None


def main():
    bus = open_bus(PORT)
    cmd = CANCommander(bus=bus, is_fd=True)

    # -----------------------------
    # 1. Ping
    # -----------------------------
    print("🔵 Sending PING...")
    cmd.ping(device_id=DEVICE_ID)

    value = wait_for_response(bus, ReadSpec.PING)

    if value is None:
        print("❌ No ping response")
        return

    print(f"📥 Ping response: {value}")

    if abs(value - 69.0) > 1e-3:
        print("❌ Ping incorrect")
        return

    print("✅ Ping verified")

    # -----------------------------
    # 2. Send position
    # -----------------------------
    target_deg = 45.0
    print(f"🎯 Sending position: {target_deg} deg")
    cmd.set_position(device_id=DEVICE_ID, degrees=target_deg)

    time.sleep(0.2)  # small delay to let motion start

    # -----------------------------
    # 3. Read position for 10 sec
    # -----------------------------
    print("📡 Streaming position for 10 seconds...")

    start_time = time.time()

    while time.time() - start_time < 10.0:
        # Request position
        cmd.read_position(device_id=DEVICE_ID)

        # Wait for response
        pos = wait_for_response(bus, ReadSpec.POSITION, timeout=0.1)

        if pos is not None:
            print(f"📍 Position: {pos:.3f} deg")
        else:
            print("⚠️ No position response")

        time.sleep(0.05)  # ~20 Hz polling rate


if __name__ == "__main__":
    main()