# to run this code, type in the terminal: python status_display.py
# Make sure you change this value correctly

import serial
import time

PORT = "COM8"     # Change this to match your port
BAUD_RATE = 9600

def main():
    try:
        with serial.Serial(PORT, BAUD_RATE, timeout=2) as ser:
            print(f"Connected to {PORT} at {BAUD_RATE} baud.")
            
            # Give Teensy time to reset
            time.sleep(2)
            
            while True:
                if ser.in_waiting:
                    line = ser.readline().decode("utf-8", errors="ignore").strip()
                    print(line)

    except serial.SerialException as e:
        print(f"Serial error: {e}")
    except KeyboardInterrupt:
        print("Stopped by user.")

if __name__ == "__main__":
    main()

