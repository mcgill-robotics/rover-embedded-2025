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
    interface = serial.Serial("/dev/ttyACM1", 115200)
except serial.SerialException as e:
    print(f"Error opening serial port: {e}")
    exit(1)

counter = 0
firstMismatch = True
throughput = 0
try:
    stream = bytearray()
    data_rate_time_start = time.perf_counter()
    total_read_per_period = 0
    while True:
        if interface.readable():
            read_bytes:bytes = interface.read(max(1, min(1024, interface.in_waiting)))
            total_read_per_period+=len(read_bytes)
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
                print(parsed_data)
                print(f"{throughput/125000} Mbps, {total_read_per_period} bytes total, {elapsed}s")
                parsed_data["data"] = msgpack.load(io.BytesIO(parsed_data["data"]))
                # print(parsed_data)
                
                if counter != int(parsed_data["data"].split()[-1]):
                    if not firstMismatch:
                        print("Mismatched count")
                        counter = int(parsed_data["data"].split()[-1])
                        # exit(1)
                    else:
                        counter = int(parsed_data["data"].split()[-1])
                        firstMismatch = False
                counter+=1
            # print(stream)
        data_rate_end_time = time.perf_counter()
        elapsed = data_rate_end_time-data_rate_time_start
        if elapsed >= 1:
            throughput = total_read_per_period/elapsed
            total_read_per_period = 0
            data_rate_time_start = time.perf_counter()


                

except KeyboardInterrupt:
    print("\nStopping flood tester.")
    interface.close()
    exit(0)
