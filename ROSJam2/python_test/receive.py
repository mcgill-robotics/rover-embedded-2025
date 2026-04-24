import time
import cobs
import serial
import msgpack
import io

# Change this to the lits of wanted UARTs to test
config = [0]#, 1, 2, 3, 4, 5]

uarts = ["diag0"]#,"uart0", "uart1", "uart2", "uart3", "uart4", "uart5"]
values = [0, 0, 0, 0, 0, 0]

try:
    interface = serial.Serial("/dev/ttyACM2", 115200)
except serial.SerialException as e:
    print(f"Error opening serial port: {e}")
    exit(1)

counter = 0
try:
    stream = bytearray()
    while True:
        if interface.readable():
            read_bytes:bytes = interface.read()
            # print(read_bytes)
            if read_bytes:
                stream.extend(read_bytes)
            # print("before")
            # print(stream)
            data, read = cobs.decode(stream, 0)
            # print("after")
            # print(stream)
            # print(data)
            # print(read)
            if read > 0 and len(data) > 0:
                parsed_data = msgpack.load(io.BytesIO(data[:-1])) # strip newline
                # print(parsed_data)
                parsed_data["data"] = msgpack.load(io.BytesIO(parsed_data["data"]))
                print(parsed_data)
                if counter != int(parsed_data["data"].removeprefix("Hello uart0 ")):
                    print("Mismatched count")
                    exit(1)
                counter+=1
            # print(stream)
                

except KeyboardInterrupt:
    print("\nStopping flood tester.")
    interface.close()
    exit(0)
