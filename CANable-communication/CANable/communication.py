import can
import time
import serial
from enum import Enum


#TODO: need robust
#TODO: change global vars manually
current_interface = 'slcan'
current_channel = 'COM9'
current_bitrate = 500000

class IDNumber(Enum):
    """ID numbers for the different CAN messages"""
    #max 11 bits long for ID
    BLDCDriveSpeed = 0b1
    BLDCDrivePos = 0b10
    BLDCDriveTorque = 0b100

    #TODO: Brushed are only for steering so naming might change
    BrushedDriveSpeed = 0b1000
    BrushedDrivePos = 0b10000
    BrushedDriveTorque = 0b100000


class CANMessage:
    def __init__(self, senderID: str, DLC: int, message): 
        """Creates a CAN message
        param:      senderID: any of the list of IDNumber
                    DLC     : from 0 to 8 bytes of sent info
                    message : is no longer than DLC and contains the information
                
        returns:    None
        
        """
        self.senderID = IDNumber(senderID)
        self.DLC = DLC
        #TODO: add a check to verify that the message is the right size
        #TODO: make sure that the message is sent in hexadecimal or what
        self.message = message

    def to_can_msg(self):
        """Returns the message to send"""
        self.CANMessage = can.Message(arbitration_id=self.senderID, data=self.message, is_extended_id=False, dlc=self.DLC)
        return self.CANMessage


class CANStation:
    def __init__(self, interface, channel, bitrate: int, ):
        """Creates a CAN station that receives or sends CAN messages
        param:      interface   : see canable.io 
                    channel     : on Windows, look at the port used
                    bitrate     : make sure it is consistent on both end of the CAN communication
                
        returns:    None
        """
        self.interface = interface
        self.channel = channel
        self.bitrate = bitrate
        self.bus = None
        self.setup_CANbus()

    def setup_CANbus(self):
        """"Setup CAN bus instance"""
        try:
            self.bus = can.interface.Bus(interface=self.interface, channel=self.channel, bitrate=self.bitrate, ignore_config=True, )
            print(f"CANStation initialized on channel {self.channel} with bitrate {self.bitrate}.")
        except can.CanError as e:
            print(f"Could not setup bus instance on channel {self.channel} with bitrate {self.bitrate} due to {e}.")
    
    def send_msg(self, message: CANMessage):
        """Sends message
        param   :   message   : a CANMessage
                    
        returns :   0, if the message is sent
                    -1, if not sent
        """
        if not self.bus:
            print("CAN bus not initialized. Cannot send message.")
            return -1
        
        try: 
            self.bus.send(msg)
            print(f"Message sent")
            return 0
        except can.CanError as e:
            print(f"Message at repetition {i} not sent due to {e}")
            return -1

    def send_msg_repeat(self, message: CANMessage, repetition: int, interval: int):
        """Sends message n times every m seconds
        param   :   message   : a CANMessage
                    repetition: number of times the message is sent
                    interval  : time in seconds in between messages
        returns :   0, if the message is sent
                    -1, if not sent
        """
        if not self.bus:
            print("CAN bus not initialized. Cannot send message.")
            return -1

        msg = message.to_can_msg()

        for i in range(0,repetition):
            try: 
                self.bus.send(msg)
                print(f"Message sent")
                time.sleep(interval)
            except can.CanError as e:
                print(f"Message at repetition {i} not sent due to {e}")
                return -1
        return 1
    
    def receive_message(self, timeout=1.0):
        """Receives message with a timeout
        param   :   timeout : in seconds
                    
        returns :   the message, if the message is received within timeout
                    -1, if not sent within timeout
        """
        if not self.bus:
            print("CAN bus not initialized. Cannot receive messages.")
            return -1

        try:
            msg = self.bus.recv(timeout)
            if msg:
                received_msg = CANMessage(
                    arbitration_id=msg.arbitration_id,
                    data=msg.data,
                    is_extended_id=msg.is_extended_id
                )
                print(f"Message received: {received_msg}")
                return received_msg
            else:
                print("No message received within timeout period.")
                return -1
        except Exception as e:
            print(f"Failed to receive message: {e}")
            return -1
        

 

if __name__ == "__main__":





while (1):
    # Create a message to send
    try:
        msg = can.Message(arbitration_id=0x103, data=[0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88], is_extended_id=False, dlc=8)
    except can.CanError:
        print("Could not create message")

    try:
        bus.send(msg)
        print("Message sent!")
    except can.CanError:
        print("Message NOT sent!")
    
    time.sleep(1)

#NOTE: RXDATA AND DATA ARE JUST THE DATA SECTION OF THE CAN MESSAGE AND THUS DLC ONLY DICTATES THIS PART!!!!!!!!!!
