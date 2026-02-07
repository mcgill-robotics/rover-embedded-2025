import json

import serial

uarts = ["uart0", "uart1", "uart2", "uart3", "uart4", "uart5"]
values = [0, 0, 0, 0, 0, 0]
errors = [0, 0, 0, 0, 0, 0]
failures = 0

try:
    interface = serial.Serial("/dev/ttyACM1", 115200)
except serial.SerialException as e:
    print(f"Error opening serial port: {e}")
    exit(1)

try:
    while True:
        line = interface.readline().decode().strip(" \n")
        try:
            json = json.loads(line)
        except json.JSONDecodeError:
            print(f"Error decoding JSON: {line}")
            failures += 1
            continue
        topic = json.get("topic")
        if topic in uarts:
            index = uarts.index(topic)
            if values[index] != int(json.get("message")):
                print(
                    f"Error: Expected {values[index]} but got {json.get('message')} for {topic}"
                )
                errors[index] += 1
            values[index] += 1
except KeyboardInterrupt:
    print("Final results:")
    for i in range(len(uarts)):
        print(f"{uarts[i]}: {values[i]} messages, {errors[i]} dropped")
    interface.close()
    exit(0)
