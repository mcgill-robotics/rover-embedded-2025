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

send = True
times = []
first_write = True
try:
    stream = bytearray()
    while True:
        if interface.writable():
            if first_write:
                written = interface.write(bytes([0]))
                if written:
                    first_write= False
                continue

            current_time = time.perf_counter()
            data = msgpack.dumps(f"{current_time}")#{uarts[current_uart]}")
            
            data = msgpack.dumps({"topic":"diag0", "data":data})
            data = bytearray(data)
            
            data.append("\n".encode('ascii')[0])
            
            data = cobs.encode(bytes(data), 0)
            

            interface.write(data)
            interface.flush()
            send = False

        while not send:
            if interface.readable() and not send:
                read_bytes:bytes = interface.read()
                if read_bytes:
                    stream.extend(read_bytes)
                data, read = cobs.decode(stream, 0)
                if read > 0:
                    end_time = time.perf_counter()
                    parsed_data = msgpack.load(io.BytesIO(data[:-1])) # strip newline
                    # print(parsed_data["topic"])
                    if parsed_data["topic"] == "diag0":
                        parsed_data["data"] = msgpack.load(io.BytesIO(parsed_data["data"]))
                        try:
                            start_time = float(parsed_data["data"])
                            rtt = end_time-start_time
                            print(f"rtt: {rtt}s")
                            times.append(rtt)
                        except ValueError:
                            pass
                        
                        send = True
                

except KeyboardInterrupt:
    print(f"Average rtt: {sum(times)/len(times)}s")
    print("\nStopping latency tester.")
    interface.close()
    exit(0)
