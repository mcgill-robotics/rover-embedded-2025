import time
import cobs
import serial

# Change this to the lits of wanted UARTs to test
config = [0]#, 1, 2, 3, 4, 5]

uarts = ["diag0"]#,"uart0", "uart1", "uart2", "uart3", "uart4", "uart5"]
values = [0, 0, 0, 0, 0, 0]

try:
    interface = serial.Serial("/dev/ttyACM1", 115200)
except serial.SerialException as e:
    print(f"Error opening serial port: {e}")
    exit(1)

try:
    while True:
        for i in config:
            interface.write(
                f'{{"topic":"{uarts[i]}","message":"{values[i]}"}}\n'.encode()
            )
            print(f'{{"topic":"{uarts[i]}","message":"{values[i]}"}}')
            values[i] += 1
#        time.sleep(0.1)
except KeyboardInterrupt:
    print("\nStopping flood tester.")
    interface.close()
    exit(0)
