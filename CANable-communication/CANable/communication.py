import can

# Set up a CAN bus instance
bus = can.interface.Bus(interface='slcan', channel='COM9', bitrate=500000, )

# Create a message to send
msg = can.Message(arbitration_id=0x123, data=[0x11, 0x22, 0x33, 0x44], is_extended_id=False, )

try:
    bus.send(msg)
    print("Message sent!")
except can.CanError:
    print("Message NOT sent!")
