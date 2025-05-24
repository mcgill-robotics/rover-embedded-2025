import serial
import threading
import time

SERIAL_PORT = 'COM3'  # Change to connected port name
BAUD_RATE = 9600

try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    time.sleep(2)
except serial.SerialException as e:
    print(f"Connection error: {e}")
    exit(1)

def read_serial():
    while True:
        try:
            if ser.in_waiting:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if '[' in line and ']' in line:
                    print(line)
        except Exception:
            continue

threading.Thread(target=read_serial, daemon=True).start()

def send_command(cmd: str):
    ser.write((cmd.strip() + '\n').encode('utf-8'))

# for servo input
try:
    print("Enter commands (e.g., a10, w5, d15, s5). Type 'exit' to quit.")
    while True:
        user_input = input("> ").lower()
        if user_input == "exit":
            break
        elif len(user_input) >= 2 and user_input[0] in ['a', 'w', 's', 'd'] and user_input[1:].isdigit():
            send_command(user_input)
        else:
            print("Invalid input. Format: a10, w5, s15, d5")
except KeyboardInterrupt:
    pass
finally:
    ser.close()
    print("Serial connection closed.")
