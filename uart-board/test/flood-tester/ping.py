import json
import time

import serial

pending_pings = {}
ping_id = 0

try:
    interface = serial.Serial("/dev/ttyACM1", 115200, timeout=1)
    interface.reset_input_buffer()
except serial.SerialException as e:
    print(f"Error opening serial port: {e}")
    exit(1)

time.sleep(1)

try:
    interface.write(f'{{"topic":"uart0","message":"{ping_id}"}}\n'.encode())
    start_time = time.time()
    pending_pings[ping_id] = start_time

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
            end_time = time.time()
            latency = end_time - pending_pings[value]
            del pending_pings[value]
            latencies.append(latency)
            print(f"Received ping, latency: {latency:.6f} seconds")
            ping_id += 1
            interface.write(f'{{"topic":"uart0","message":"{ping_id}"}}\n'.encode())
            start_time = time.time()
            pending_pings[ping_id] = start_time
                
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
