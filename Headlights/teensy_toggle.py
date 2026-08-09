# To kill System2, write in terminal:   python teensy_toggle.py kill
# To unkill System2, write in terminal: python teensy_toggle.py unkill
# To toggle the headlights on, type in the terminal:  python teensy_toggle.py lights_on 
# To toggle the headlights off, type in the terminal: python teensy_toggle.py lights_off
# Make sure you modify your port properly 

import serial
import time
import sys

PORT = "COM6"  # Change this to the correct port
BAUDRATE = 9600

# Argument check
if len(sys.argv) != 2:
    print("Usage: python teensy_toggle.py [kill|unkill|lights_on|lights_off]")
    sys.exit(1)

cmd = sys.argv[1].lower()
if cmd not in ["kill", "unkill", "lights_on", "lights_off"]:
    print("Invalid argument. Use 'kill', 'unkill', 'lights_on' or 'lights_off'.")
    sys.exit(1)

# Map command to byte
cmd_map = {
    "kill": b'Kill_H',
    "unkill": b'Kill_L'
    "lights_on": b'Light_H',
    "lights_off": b'Light_L',
}

try:
    with serial.Serial(PORT, BAUDRATE, timeout=1) as ser:
        time.sleep(2)  # Wait for Teensy to initialize
        ser.write(cmd_map[cmd])
        print(f"Sent command '{cmd}' to Teensy on {PORT}")
except serial.SerialException as e:
    print(f"Serial error: {e}")
