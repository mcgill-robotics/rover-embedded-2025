import serial
from enum import IntEnum
import time
import struct

START_BYTE = 0xAA
FEEDBACK_LEN = 17

class CommandID(IntEnum):
    SETPOINT = 0x01
    FEEDBACK = 0x02
    ECHO = 0x03
    HOME = 0x04
    ERROR = 0x05

class ErrorCode(IntEnum):
    NO_ERROR = 0x00
    UNKNOWN_COMMAND = 0x80
    INVALID_CRC = 0x06
    BUFFER_OVERFLOW = 0x07

    def __str__(self):
        return {
            ErrorCode.UNKNOWN_COMMAND: "Unknown command",
            ErrorCode.INVALID_CRC: "Invalid CRC",
            ErrorCode.BUFFER_OVERFLOW: "Buffer overflow",
            ErrorCode.NO_ERROR: "No error"
        }.get(self, "Unknown error")

def crc8(data: bytes) -> int:
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc << 1) ^ 0x07 if (crc & 0x80) else (crc << 1)
            crc &= 0xFF
    return crc

def build_packet(cmd: int, payload: bytes = b'') -> bytes:
    pkt = bytearray()
    pkt.append(START_BYTE)
    pkt.append(cmd)
    pkt.append(len(payload))
    pkt.extend(payload)
    pkt.append(crc8(pkt[1:]))  # CRC over cmd + len + payload
    return pkt

def wait_for_response(ser: serial.Serial, timeout=1.0) -> bytes:
    deadline = time.time() + timeout
    rx_buffer = bytearray()

    while time.time() < deadline:
        if ser.in_waiting:
            rx_buffer += ser.read(ser.in_waiting)
        time.sleep(0.001)

    return rx_buffer  # Raw dump for debugging

class BrushedBoard:
    def __init__(self, port: str, baud: int = 115200, timeout: float = 0.1):
        self.ser = serial.Serial(port, baudrate=baud, timeout=timeout)
        self.ser.reset_input_buffer()  # Clear any stale data

    def close(self):
        self.ser.close()

    def send_echo(self, data: bytes) -> bytes:
        packet = build_packet(CommandID.ECHO, data)
        self.ser.write(packet)
        resp = wait_for_response(self.ser)
        if not resp or resp[1] != CommandID.ECHO:
            raise RuntimeError("No echo or wrong command in response")
        return resp[3:-1]

    def request_feedback(self) -> dict:
        packet = build_packet(CommandID.FEEDBACK)
        print(packet.hex())
        self.ser.write(packet)
        resp = wait_for_response(self.ser)
        print("Raw bytes:", list(resp))
        if not resp or resp[1] != CommandID.FEEDBACK:
            raise RuntimeError("Invalid feedback response")

        payload = resp[3:-1]
        motor_position = list(struct.unpack('<2f', payload[0:8]))
        motor_current = list(struct.unpack('<2f', payload[8:16]))
        error_code = payload[16]

        return {
            'position': motor_position,
            'current': motor_current,
            'error': ErrorCode(error_code)
        }

    def send_setpoint(self, setpoint_0: float, setpoint_1: float, ee_command: int = 0):
        """Send two float setpoints and an EE command (uint8)"""
        payload = struct.pack('<2fB', setpoint_0, setpoint_1, ee_command)
        packet = build_packet(CommandID.SETPOINT, payload)
        self.ser.write(packet)

    def send_home(self, payload: bytes = b''):
        packet = build_packet(CommandID.HOME, payload)
        self.ser.write(packet)

    def report_error(self, error_code: int):
        packet = build_packet(CommandID.ERROR, bytes([error_code]))
        self.ser.write(packet)

# Example usage
if __name__ == "__main__":
    board = BrushedBoard("/dev/ttyACM2")  # Or 'COM4' on Windows
    try:
        # echo = board.send_echo(b"test")
        # print("Echoed:", echo)

        fb = board.request_feedback()
        print(f"Position WP: {fb['position'][0]} || Position WR: {fb['position'][1]}")
        print(f"Current WP: {fb['current'][0]} || Current WR: {fb['current'][1]}")
        print(f"Error Code: {fb['error']}")

        board.send_setpoint(10.0, 20.0, ee_command=0)

    finally:
        board.close()
