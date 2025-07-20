import serial
from enum import IntEnum
import time

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
        match self:
            case ErrorCode.UNKNOWN_COMMAND:
                return "Unknown command"
            case ErrorCode.INVALID_CRC:
                return "Invalid CRC"
            case ErrorCode.BUFFER_OVERFLOW:
                return "Buffer overflow"
            case ErrorCode.NO_ERROR:
                return "No error"
            case _:
                return "Unknown error"

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
    pkt.append(crc8(pkt[1:]))  # CRC over cmd, len, payload
    return pkt

def wait_for_response(ser: serial.Serial, timeout=1.0) -> bytes:
    deadline = time.time() + timeout
    buf = bytearray()

    while time.time() < deadline:
        if ser.in_waiting:
            buf += ser.read(ser.in_waiting)
            if len(buf) >= 3 and buf[0] == START_BYTE:
                total_len = 3 + buf[2] + 1
                if len(buf) >= total_len:
                    return buf[:total_len]
        time.sleep(0.001)
    return b''

class BrushedBoard:
    def __init__(self, port: str, baud: int = 115200, timeout: float = 0.1):
        self.ser = serial.Serial(port, baudrate=baud, timeout=timeout)

    def close(self):
        self.ser.close()

    def send_echo(self, data: bytes) -> bytes:
        """Send data and expect exact echo back."""
        packet = build_packet( CommandID.ECHO, data)
        self.ser.write(packet)
        resp = wait_for_response(self.ser)
        if not resp or resp[1] != CommandID.ECHO:
            raise RuntimeError("No echo or wrong command in response")
        return resp[3:-1]

    def request_feedback(self) -> dict:
        """Request feedback from the board. Returns dict with position, current, and error code."""
        packet = build_packet(CommandID.FEEDBACK)
        self.ser.write(packet)
        resp = wait_for_response(self.ser)
        print(resp.hex())
        if not resp or resp[1] != CommandID.FEEDBACK or resp[2] != FEEDBACK_LEN:
            raise RuntimeError("Invalid feedback response")

        payload = resp[3:-1]
        motor_position: list[float] = []
        for i in range(0, 8, 4):
            motor_position.append(float.from_bytes(payload[i:i+4], 'little', signed=True))
        motor_current: list[float] = []
        for i in range(8, 16, 4):
            motor_current.append(float.from_bytes(payload[i:i+4], 'little', signed=True))
        error_code = payload[16]

        return {
            'position': motor_position,
            'current': motor_current,
            'error': ErrorCode(error_code)
        }

    def send_setpoint(self, setpoint_bytes: bytes):
        """Send control setpoints to the board (format TBD in firmware)."""
        # TODO: finalize payload format in C
        packet = build_packet(CommandID.SETPOINT, setpoint_bytes)
        self.ser.write(packet)
        # No response expected

    def send_home(self, payload: bytes = b''):
        """Send homing command with optional payload (if any)."""
        # TODO: implement homing logic in firmware
        packet = build_packet(CommandID.HOME, payload)
        self.ser.write(packet)

    def report_error(self, error_code: int):
        """Manually send an error packet for testing/debug."""
        packet = build_packet(CommandID.ERROR, bytes([error_code]))
        self.ser.write(packet)

# Example usage
if __name__ == "__main__":
    board = BrushedBoard("/dev/ttyACM1")  # Or 'COM4' on Windows
    try:
        echo = board.send_echo(b"test")
        print("Echoed:", echo)

        fb = board.request_feedback()
        print(f"Position WP: {fb['position'][0]} || Position WR: {fb['position'][1]}")
        print(f"Current WP: {fb['current'][0]} || Current WR: {fb['current'][1]}")
        print(f"Error Code: {fb['error']}")

        board.send_setpoint(b'\x00\x10\x00\x20')  # Placeholder

    finally:
        board.close()
