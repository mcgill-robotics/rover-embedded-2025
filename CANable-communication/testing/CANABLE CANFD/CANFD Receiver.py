# RECIEVER
import can

ARB = 500_000
DATA = 2_000_000

bus = can.interface.Bus(
    interface="slcan",
    channel="COM4@115200", # make sure to adjust to your receiver COM port
    bitrate=ARB,
    data_bitrate=DATA,
    fd=True,
    ignore_config=True,
)

print("Listening for CAN FD frames...")
while True:
    m = bus.recv(timeout=1.0)
    if m:
        print("RX:", m, " data:", list(m.data))
