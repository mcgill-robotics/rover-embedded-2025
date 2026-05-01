import json
import time

import serial

pending_pings = {}
latencies = []
ping_id = 0

try:
    interface = serial.Serial("/dev/ttyACM1", 115200, timeout=1)
    interface.reset_input_buffer()
except serial.SerialException as e:
    print(f"Error opening serial port: {e}")
    exit(1)

time.sleep(1)


while True:
	line = interface.readline().decode().strip(" \n")

	if len(line) == 0:
		print("Data timeout")
		continue
	try:
		message = json.loads(line)
	except json.JSONDecodeError:
		print(f"Error decoding JSON: {line}")
		continue

	topic = message.get("topic")
	if topic == "uart0":
		value = message.get("message")
		print(f"Received {value} for {topic}")
		interface.write(f'{{"topic":"uart0","message":"{value}"}}\n'.encode())
            