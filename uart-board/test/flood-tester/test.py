import json

import serial

uarts = ["uart0", "uart1", "uart2", "uart3", "uart4", "uart5"]
values = [0, 0, 0, 0, 0, 0]

errors = [0, 0, 0, 0, 0, 0]
failures = 0

try:
    interface = serial.Serial("/dev/ttyACM1", 115200, timeout=1)
    interface.reset_input_buffer()
except serial.SerialException as e:
    print(f"Error opening serial port: {e}")
    exit(1)

try:
    while True:
        line = interface.readline().decode().strip(" \n")
        try:
            message = json.loads(line)
        except json.JSONDecodeError:
            print(f"Error decoding JSON: {line}")
            failures += 1
            continue

        topic = message.get("topic")
        if topic in uarts:
            index = uarts.index(topic)
            value = int(message.get("message"))
            if values[index] != value:
                print(f"Error: Expected {values[index]} but got {value} for {topic}")
                errors[index] += 1
                values[index] = value + 1  # Resync to the next expected value
            else:
                print(f"Received {value} for {topic}")
                values[index] += 1
except KeyboardInterrupt:
    print(f"Final results: {failures} failures")
    for i in range(len(uarts)):
        print(f"{uarts[i]}: {values[i]} messages, {errors[i]} dropped")
    interface.close()
    exit(0)
