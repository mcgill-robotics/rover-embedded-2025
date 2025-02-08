import can
import time
import serial
from enum import Enum


# TODO: need robust
# TODO: add to doc that: add threads to be able to tx rx simultaneously - no simultaenously
# if many instructions from wheels - CAN should take care of that

class IDNumber(Enum):
    """ID numbers for the different CAN messages"""
    #max 11 bits long for ID (since CAN only takes an 11 bit identifier) 

    ## DRIVE MOTORS ##
    # Desired Drive Motor Speed sent by JETSON
    BLDCDesiredSpeedFL = 0b1        #Front Left
    BLDCDesiredSpeedFR = 0b10       #Front Right
    BLDCDesiredSpeedBL = 0b100      #Back Left
    BLDCDesiredSpeedBR = 0b1000     #Back Right
    # Current Drive Motor Speed sent by motors
    BLDCCurrentSpeedFL = 0b10000    #Front Left
    BLDCCurrentSpeedFR = 0b100000   #Front Right
    BLDCCurrentSpeedBL = 0b1000000  #Back Left
    BLDCCurrentSpeedBR = 0b10000000 #Back Right

    ## STEERING MOTORS ##
    # Desired Steering Motor Position sent by JETSON
    SteeringDesiredPosFL = 0b11         #Front Left
    SteeringDesiredPosFR = 0b110        #Front Right
    SteeringDesiredPosBL = 0b1100       #Back Left
    SteeringDesiredPosBR = 0b11000      #Back Right
    # Current Steering Motor Position sent by motors
    SteeringCurrentPosFl = 0b110000     #Front Left
    SteeringCurrentPosFR = 0b1100000    #Front Right
    SteeringCurrentPosBl = 0b11000000   #Back Left
    SteeringCurrentPosBR = 0b110000000  #Back Right

    TESTSTM = 0x201 #TODO: for testing with stm32, erase later

class CANMessage:
    def __init__(self, senderID, DLC: int, message: list[int]): 
        """Creates a CAN message
        param:      senderID: any of the list of IDNumber
                    DLC     : from 0 to 8 bytes of sent info
                    message : is no longer than DLC and contains the information
                
        returns:    None
        
        """
        if type(senderID) == str:
            self.senderID = IDNumber[senderID]
        elif type(senderID) == int:
            self.senderID = IDNumber(senderID)
        self.DLC = DLC
        #TODO: make sure that the message is sent in hexadecimal or what
        self.message = message

        # Verifies the size of the message corresponds to DLC size
        # If DLC is bigger than message size, CANBus will pad the rest with 0x00
        if len(self.message) > self.DLC:
            raise ValueError(f"Given message of size {len(self.message)} is bigger than configured DLC size {self.DLC}")

    def to_can_msg(self):
        """Returns the message to send"""
        self.CANMessage = can.Message(arbitration_id=self.senderID.value, data=self.message, is_extended_id=False, dlc=self.DLC)
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
    
    def send_msg(self, message: CANMessage, test = False):
        """Sends message
        param   :   message   : a CANMessage
                    test      : a boolean for whether we are testing the speed at which the messages are sent and received (TTL time) in ns
                    
        returns :   0, if the message is sent
                    -1, if not sent
        """

        if not self.bus:
            print("CAN bus not initialized. Cannot send message.")
            return -1
        
        msg = message.to_can_msg()

        try: 
            #TODO: for testing, can remove
            if test:
                self.init_time = time.time_ns()

            self.bus.send(msg)
            print(f"Message sent")
            return 0
        except can.CanError as e:
            print(f"Message not sent due to {e}")
            return -1

    def send_msg_repeat(self, message: CANMessage, repetition: int, interval: int, test = False):
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
    
    def receive_message(self, timeout=1.0, test=False):
        """Receives a CAN message with a timeout"""
        if not self.bus:
            print("CAN bus not initialized. Cannot receive messages.")
            return -1

        try:
            msg = self.bus.recv(timeout)
            if msg:
                # Allow unknown IDs by storing them as raw integers
                sender_id = msg.arbitration_id if msg.arbitration_id in [e.value for e in IDNumber] else msg.arbitration_id
                received_msg = CANMessage(senderID=sender_id, message=list(msg.data), DLC=msg.dlc)

                print(f"Message received: ID={hex(sender_id)}, Data={list(received_msg.message)}")
                return received_msg
            else:
                print("No message received within timeout period.")
                return -1
        except Exception as e:
            print(f"Failed to receive message: {e}")
            return -1

    def send_test_message(self):
        """Sends a test message to ESC and waits for a response"""
        if not self.bus:
            print("CAN bus not initialized. Cannot send test message.")
            return
        
        message = can.Message(arbitration_id=0x200, data=[0x01], is_extended_id=False, dlc=1)
        try:
            self.bus.send(message)
            print("Sent test message to ESC")
        except can.CanError as e:
            print(f"CAN Message send failed: {e}")
        
        try:
            response = self.receive_message(timeout=5.0)  # Increase timeout
            if response != -1 and response.senderID == IDNumber(0x201):  # Fixed response check
                print(f"Received valid response from ESC: {list(response.message)}")
            else:
                print("No valid response received from ESC.")
        finally:
            self.close_bus()  # Ensure CAN bus is properly closed


    def close_bus(self):
        """Properly shuts down the CAN bus"""
        if self.bus:
            try:
                self.bus.shutdown()
                print("CAN bus properly closed.")
            except can.CanOperationError:
                print("Warning: Could not properly shut down CAN bus.")
            except Exception as e:
                print(f"Unexpected error while shutting down CAN bus: {e}")


# def main():
#     bus = can.Bus(channel='COM6', interface='slcan', bitrate=500000)
#     send_test_message(bus)

    # Code elow works for listening


    # # Initialize the CAN interface
    # bus = can.Bus(channel='COM6', interface='slcan', bitrate=500000)

    # print("Listening for CAN messages...")
    # while True:
    #     message = bus.recv(timeout=0.15)  # Wait up to 1 second for a message
    #     if message:
    #         print(f"Received message: ID={hex(message.arbitration_id)}, Data={list(message.data)}")
    #     else:
    #         print("No message received.")

# if __name__ == "__main__":
#     main()

if __name__ == "__main__":
    # Initialize the CAN interface
    my_station = CANStation('slcan', 'COM9', 500000)

    # Send a test message to the ESC
    my_station.send_test_message()

 

# if __name__ == "__main__":
#     my_station = CANStation('slcan', 'COM6', 500000)
#     my_msg = CANMessage('TESTJAWDAT_ESC', 2, [0x64,0xA])#, 0x5, 0x5, 0x5, 0x5, 0x5, 0x5])\


#     my_station.send_msg_repeat(my_msg, 100000, 1)
#     my_station.receive_message(20)


#     init_time = time.time_ns()
#     count = 0
#     while (time.time_ns() - init_time <= 1e9): #for a second
#         my_station.send_msg(my_msg, 1)
#         my_station.receive_message(10, 1)
#         count = count + 1

        






# while (1):
#     # Create a message to send
#     try:
#         msg = can.Message(arbitration_id=0x103, data=[0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88], is_extended_id=False, dlc=8)
#     except can.CanError:
#         print("Could not create message")

#     try:
#         bus.send(msg)
#         print("Message sent!")
#     except can.CanError:
#         print("Message NOT sent!")
    
#     time.sleep(1)

#NOTE: RXDATA AND DATA ARE JUST THE DATA SECTION OF THE CAN MESSAGE AND THUS DLC ONLY DICTATES THIS PART!!!!!!!!!!
