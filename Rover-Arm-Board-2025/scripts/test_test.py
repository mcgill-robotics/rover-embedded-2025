import serial
import time

def calculate_crc(data: bytes) -> int:
    crc = 0x00
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x07) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc

def send_feedback_command(port_name="COM3", baudrate=115200):
    header = 0xAA
    cmd = 0x01       # Feedback command
    length = 0x00    # No payload

    # Construct message (no payload)
    data = bytes([header, cmd, length])
    crc = calculate_crc(data)
    message = data + bytes([crc])

    with serial.Serial(port_name, baudrate=baudrate, timeout=1) as ser:
        print(f"Sending: {[hex(b) for b in message]}")
        ser.write(message)

        time.sleep(0.05)  # adjust delay as needed

        response = ser.read(64)
        print(f"Received {len(response)} bytes: {[hex(b) for b in response]}")

        if len(response) >= 4:
            r_header = response[0]
            r_cmd = response[1]
            r_len = response[2]
            r_payload = response[3:3 + r_len]
            r_crc = response[3 + r_len] if len(response) > 3 + r_len else None

            print(f"Header: 0x{r_header:02X}")
            print(f"Command: 0x{r_cmd:02X}")
            print(f"Length: {r_len}")
            print(f"Payload: {[hex(b) for b in r_payload]}")
            print(f"CRC: 0x{r_crc:02X}" if r_crc is not None else "CRC: missing")

            # Validate CRC
            computed_crc = calculate_crc(response[:3 + r_len])
            if r_crc == computed_crc:
                print("CRC is valid ✅")
            else:
                print(f"CRC mismatch ❌ (Expected 0x{computed_crc:02X})")

if __name__ == "__main__":
    send_feedback_command("/dev/ttyACM1")  # <-- Update to match your port
