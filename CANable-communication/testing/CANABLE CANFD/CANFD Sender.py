# SENDER
import can, time

ARB = 500_000      # arbitration bitrate
DATA = 2_000_000   # data-phase bitrate

bus = can.interface.Bus(
    interface="slcan",
    channel="COM3@115200", # make sure to adjust to your sender COM port
    bitrate=ARB,
    data_bitrate=DATA,
    fd=True,
    ignore_config=True,
)

msg = can.Message(arbitration_id=0x123, data=bytes([6,7,6,7,6,7,6,7]), dlc=64, is_fd=True) # for the data i only put 8 bytes, but dlc is 64 (rest will be padded with zeros)
while True:
    bus.send(msg)
    print("TX:", msg)
    time.sleep(0.5)
