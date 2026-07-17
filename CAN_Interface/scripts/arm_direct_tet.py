import argparse
import time

import can

from esc_can.protocol import (
    ARB_BITRATE,
    DATA_BITRATE,
    MotorType,
    ReadSpec,
    RunSpec,
    build_single_read_frame,
    build_single_run_frame,
    unpack_single_float,
)

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


def flush_rx(bus: can.BusABC):
    """Drain any stale frames sitting in the receive buffer."""
    while bus.recv(timeout=0.01) is not None:
        pass


def send_frame(bus: can.BusABC, arb_id: int, data: bytes):
    msg = can.Message(
        arbitration_id=arb_id,
        data=data,
        is_fd=True,
        bitrate_switch=True,
    )
    bus.send(msg)


def read_field(bus: can.BusABC, device_id: int, spec: ReadSpec, motor_type: MotorType):
    """
    Send a single read request and wait for the matching ESC response.
    Returns (value, round_trip_ms) or (None, None) on timeout.
    """
    arb_id, data = build_single_read_frame(
        device_id=device_id,
        read_spec=spec,
        motor_type=motor_type,
    )
    expected_resp_id = arb_id | (1 << 10)  # sender bit becomes SLAVE

    flush_rx(bus)
    send_frame(bus, arb_id, data)

    t_send = time.perf_counter()

    deadline = time.time() + TIMEOUT_S
    while time.time() < deadline:
        rx = bus.recv(timeout=0.25)
        if rx is None:
            continue
        if rx.arbitration_id == expected_resp_id:
            t_recv = time.perf_counter()
            rtt_ms = (t_recv - t_send) * 1000.0
            return unpack_single_float(rx.data), rtt_ms

    return None, None


def send_run_command(
    bus: can.BusABC,
    device_id: int,
    run_spec: RunSpec,
    value: float,
    motor_type: MotorType,
):
    """
    Send a RUN command. This does not wait for a response because RUN commands
    generally do not produce a direct reply frame.
    """
    arb_id, data = build_single_run_frame(
        device_id=device_id,
        run_spec=run_spec,
        value=value,
        motor_type=motor_type,
    )

    flush_rx(bus)
    send_frame(bus, arb_id, data)
    return arb_id


def motor_type_from_str(name: str) -> MotorType:
    name = name.strip().upper()
    if name == "STEERING":
        return MotorType.STEERING
    if name == "DRIVE":
        return MotorType.DRIVE
    raise ValueError(f"Unknown motor type: {name}")


def do_read_all(bus: can.BusABC, device_id: int, motor_type: MotorType):
    read_fields = [
        (ReadSpec.PING, "Ping", 69.0),
        (ReadSpec.TEMPERATURE, "Temperature (°C)", None),
        (ReadSpec.VOLTAGE, "VBUS Voltage (V)", None),
        (ReadSpec.POSITION, "Position (deg)", None),
        (ReadSpec.CURRENT, "Current (A)", None),
        (ReadSpec.CURRENT_STATE, "Motor State", None),
        (ReadSpec.CONTROL_MODE, "Control Mode", None),
    ]

    print(f"{'=' * 55}")
    print(f"  Reading all fields from ESC {device_id}")
    print(f"{'=' * 55}")

    for spec, label, expected in read_fields:
        arb_id, _ = build_single_read_frame(
            device_id=device_id,
            read_spec=spec,
            motor_type=motor_type,
        )
        print(f"\n  TX  0x{arb_id:03X}  [{label}]")

        value, rtt_ms = read_field(bus, device_id, spec, motor_type)

        if value is None:
            print("  RX  TIMEOUT — no response")
            continue

        resp_id = arb_id | (1 << 10)
        print(f"  RX  0x{resp_id:03X}  → {label} = {value}    ({rtt_ms:.2f} ms)")

        if expected is not None:
            if value == expected:
                print(f"       ✓ matches expected {expected}")
            else:
                print(f"       ✗ expected {expected}, got {value}")

        time.sleep(0.05)


def main() -> None:
    parser = argparse.ArgumentParser(description="Standalone CAN FD ESC test tool")
    parser.add_argument("--port", default="COM4", help="CANable serial port (default: COM4)")
    parser.add_argument("--id", type=int, default=1, help="Target ESC device ID 0–15 (default: 1)")
    parser.add_argument(
        "--motor",
        default="STEERING",
        choices=["DRIVE", "STEERING"],
        help="Motor type for CAN ID encoding (default: STEERING)",
    )

    subparsers = parser.add_subparsers(dest="command", required=True)

    subparsers.add_parser("read-all", help="Read all implemented fields")

    p_read = subparsers.add_parser("read", help="Read one field")
    p_read.add_argument(
        "spec",
        choices=[name for name in ReadSpec.__members__.keys()],
        help="ReadSpec name",
    )

    p_run = subparsers.add_parser("run", help="Send one RUN command")
    p_run.add_argument(
        "spec",
        choices=[name for name in RunSpec.__members__.keys()],
        help="RunSpec name",
    )
    p_run.add_argument(
        "--value",
        type=float,
        default=0.0,
        help="Float payload value (degrees for POSITION, rad/s for SPEED, usually 0 for STOP/ACKNOWLEDGE_FAULTS/CALIBRATION)",
    )

    p_pos = subparsers.add_parser("position", help="Send RUN POSITION")
    p_pos.add_argument("degrees", type=float, help="Target position in degrees")

    p_vel = subparsers.add_parser("velocity", help="Send RUN SPEED")
    p_vel.add_argument("rad_per_sec", type=float, help="Target velocity in rad/s")

    subparsers.add_parser("stop", help="Send RUN STOP")
    subparsers.add_parser("ack", help="Send ACKNOWLEDGE_FAULTS")
    subparsers.add_parser("calib", help="Send CALIBRATION")

    args = parser.parse_args()

    device_id = args.id
    motor_type = motor_type_from_str(args.motor)

    print(f"Opening CAN bus on {args.port} …")
    bus = open_bus(args.port)

    try:
        if args.command == "read-all":
            do_read_all(bus, device_id, motor_type)

        elif args.command == "read":
            spec = ReadSpec[args.spec]
            arb_id, _ = build_single_read_frame(
                device_id=device_id,
                read_spec=spec,
                motor_type=motor_type,
            )
            print(f"TX  0x{arb_id:03X}  [READ {spec.name}]")

            value, rtt_ms = read_field(bus, device_id, spec, motor_type)
            if value is None:
                print("RX  TIMEOUT — no response")
            else:
                print(f"RX  0x{(arb_id | (1 << 10)):03X}  → {spec.name} = {value}    ({rtt_ms:.2f} ms)")

        elif args.command == "run":
            spec = RunSpec[args.spec]
            arb_id = send_run_command(
                bus=bus,
                device_id=device_id,
                run_spec=spec,
                value=args.value,
                motor_type=motor_type,
            )
            print(f"TX  0x{arb_id:03X}  [RUN {spec.name}]  value={args.value}")

        elif args.command == "position":
            arb_id = send_run_command(
                bus=bus,
                device_id=device_id,
                run_spec=RunSpec.POSITION,
                value=args.degrees,
                motor_type=motor_type,
            )
            print(f"TX  0x{arb_id:03X}  [RUN POSITION]  degrees={args.degrees}")

        elif args.command == "velocity":
            arb_id = send_run_command(
                bus=bus,
                device_id=device_id,
                run_spec=RunSpec.SPEED,
                value=args.rad_per_sec,
                motor_type=motor_type,
            )
            print(f"TX  0x{arb_id:03X}  [RUN SPEED]  rad_per_sec={args.rad_per_sec}")

        elif args.command == "stop":
            arb_id = send_run_command(
                bus=bus,
                device_id=device_id,
                run_spec=RunSpec.STOP,
                value=0.0,
                motor_type=motor_type,
            )
            print(f"TX  0x{arb_id:03X}  [RUN STOP]")

        elif args.command == "ack":
            arb_id = send_run_command(
                bus=bus,
                device_id=device_id,
                run_spec=RunSpec.ACKNOWLEDGE_FAULTS,
                value=0.0,
                motor_type=motor_type,
            )
            print(f"TX  0x{arb_id:03X}  [RUN ACKNOWLEDGE_FAULTS]")

        elif args.command == "calib":
            arb_id = send_run_command(
                bus=bus,
                device_id=device_id,
                run_spec=RunSpec.CALIBRATION,
                value=0.0,
                motor_type=motor_type,
            )
            print(f"TX  0x{arb_id:03X}  [RUN CALIBRATION]")

    finally:
        bus.shutdown()


if __name__ == "__main__":
    main()