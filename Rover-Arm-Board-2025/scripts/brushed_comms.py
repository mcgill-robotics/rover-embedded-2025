import serial
import time

START_BYTE = 0xAA
CMD_SETPOINT = 0x01
CMD_FEEDBACK = 0x02
CMD_ECHO = 0x03
CMD_HOME = 0x04
CMD_ERROR = 0x05

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
        packet = build_packet(CMD_ECHO, data)
        self.ser.write(packet)
        resp = wait_for_response(self.ser)
        if not resp or resp[1] != CMD_ECHO:
            raise RuntimeError("No echo or wrong command in response")
        return resp[3:-1]

    def request_feedback(self) -> dict:
        """Request feedback from the board. Returns dict with position, current, and error code."""
        packet = build_packet(CMD_FEEDBACK)
        self.ser.write(packet)
        resp = wait_for_response(self.ser)
        if not resp or resp[1] != CMD_FEEDBACK or resp[2] != 9:
            raise RuntimeError("Invalid feedback response")

        payload = resp[3:-1]
        motor_position = payload[0:4]
        motor_current = payload[4:8]
        error_code = payload[8]

        return {
            'position': int.from_bytes(motor_position, 'little'),
            'current': int.from_bytes(motor_current, 'little'),
            'error': error_code
        }

    def send_setpoint(self, setpoint_bytes: bytes):
        """Send control setpoints to the board (format TBD in firmware)."""
        # TODO: finalize payload format in C
        packet = build_packet(CMD_SETPOINT, setpoint_bytes)
        self.ser.write(packet)
        # No response expected

    def send_home(self, payload: bytes = b''):
        """Send homing command with optional payload (if any)."""
        # TODO: implement homing logic in firmware
        packet = build_packet(CMD_HOME, payload)
        self.ser.write(packet)

    def report_error(self, error_code: int):
        """Manually send an error packet for testing/debug."""
        packet = build_packet(CMD_ERROR, bytes([error_code]))
        self.ser.write(packet)

# Example usage
if __name__ == "__main__":
    board = BrushedBoard("/dev/ttyACM0")  # Or 'COM4' on Windows
    try:
        echo = board.send_echo(b"test")
        print("Echoed:", echo)

        fb = board.request_feedback()
        print("Feedback:", fb)

        board.send_setpoint(b'\x00\x10\x00\x20')  # Placeholder

    finally:
        board.close()
