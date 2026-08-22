# This will be the python script to run for current sensing 

import serial
import time

# Replace with your Teensy's port
PORT = 'COM3'           # Windows
# PORT = '/dev/ttyACM0' # Linux

BAUDRATE = 9600         # Match your Teensy sketch's Serial.begin(9600);

# Open serial connection
ser = serial.Serial(PORT, BAUDRATE, timeout=1)

time.sleep(2)  # Allow time for Teensy to reset on serial connection

# Send a command
command = "POWER_ON\n"  # Ensure your Teensy is coded to read this
ser.write(command.encode('utf-8'))
print(f"Sent: {command.strip()}")

# Optionally, read a response
response = ser.readline().decode('utf-8').strip()
print(f"Received: {response}")

# Close when done
ser.close()
