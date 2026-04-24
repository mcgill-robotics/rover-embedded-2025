import time
import cobs
import serial
import msgpack
import io

# Change this to the lits of wanted UARTs to test
# config = [0]#, 1, 2, 3, 4, 5]

uarts = ["diag0", "uart0", "uart1", "uart2", "uart3", "uart4", "uart5"]
values = [0, 0, 0, 0, 0, 0]

try:
    interface = serial.Serial("/dev/ttyACM1", 115200000)
except serial.SerialException as e:
    print(f"Error opening serial port: {e}")
    exit(1)
current_uart = 0
print("Starting")
counter = 0
try:
    first_write = True
    while True:
        if interface.writable():
            if first_write:
                written = interface.write(bytes([0]))
                if written:
                    first_write= False
                continue

            
            data = msgpack.dumps(f"Hello from {uarts[current_uart]} {counter}")
            data = msgpack.dumps({"topic":uarts[current_uart], "data":data})
            data = bytearray(data)
            data.append("\b".encode('ascii')[0])
            data = cobs.encode(bytes(data), 0)
            # print(data)
            interface.write(data)
            interface.flush()
            # print(f"writing to {uarts[current_uart]}")
            current_uart = (current_uart+1)%len(uarts)
            counter+=1
        else:
            print("not writable")
                

except KeyboardInterrupt:
    print("\nStopping flood tester.")
    interface.close()
    exit(0)
