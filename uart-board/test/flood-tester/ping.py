import json
import time

import serial

latencies = []

try:
    interface = serial.Serial("/dev/ttyACM1", 115200, timeout=1)
    interface.reset_input_buffer()
except serial.SerialException as e:
    print(f"Error opening serial port: {e}")
    exit(1)

time.sleep(1)

try:
    interface.write('{{"topic":"uart0","message":"ping"}}\n'.encode())
    start_time = time.time()

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
            if value == "ping":
                end_time = time.time()
                latency = end_time - start_time
                latencies.append(latency)
                print(f"Received ping, latency: {latency:.6f} seconds")
                interface.write('{{"topic":"uart0","message":"ping"}}\n'.encode())
                start_time = time.time()
                continue

except KeyboardInterrupt:
    interface.close()
    if latencies:
        average_latency = sum(latencies) / len(latencies)
        print(
            f"\nAverage latency over {len(latencies)} pings: {average_latency:.6f} seconds"
        )
    else:
        print("\nNo pings received.")
    exit(0)
