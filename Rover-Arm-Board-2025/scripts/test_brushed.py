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
            if crc & 0x80:
                crc = (crc << 1) ^ 0x07
            else:
                crc <<= 1
            crc &= 0xFF
    return crc

def build_echo_packet(payload: bytes) -> bytes:
    packet = bytearray()
    packet.append(START_BYTE)
    packet.append(CMD_ECHO)                 # Command directly after start byte
    packet.append(len(payload))             # Payload length
    packet.extend(payload)
    crc = crc8(packet[1:])                  # CRC of command + len + payload
    packet.append(crc)
    return packet

def wait_for_response(ser: serial.Serial, timeout=2.0) -> bytes:
    deadline = time.time() + timeout
    buf = bytearray()

    while time.time() < deadline:
        if ser.in_waiting > 0:
            buf += ser.read(ser.in_waiting)

            if len(buf) >= 3 and buf[0] == START_BYTE:
                total_len = 3 + buf[2] + 1  # header + payload + crc
                if len(buf) >= total_len:
                    return buf[:total_len]
        time.sleep(0.001)
    return b''

def main():
    port = '/dev/ttyACM1'  # Change as needed
    ser = serial.Serial(port, 115200, timeout=0.1)

    payload = b'Hello, brushed board!'
    packet = build_echo_packet(payload)

    print(f"Sending echo packet: {packet.hex()}")
    ser.write(packet)

    response = wait_for_response(ser)
    print(f"Received response: {response.hex() if response else 'None'}")

    if not response:
        print("❌ No response received")
        return

    if response[1] != CMD_ECHO:
        print("❌ Unexpected command in response")
        return

    echoed_data = response[3:-1]  # strip start, cmd, len, and crc
    if echoed_data != payload:
        print(f"❌ Echo mismatch!\nSent: {payload}\nGot : {echoed_data}")
    else:
        print("✅ Echo successful!")

    ser.close()

if __name__ == "__main__":
    main()
