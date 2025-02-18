import can
import time
import serial
from enum import Enum

## FIRST FOUR BYTES OF THE SENT/DATA FROM MSE TO LSE. RXDATA AND TXDATA HAVE A MAX OF 8 BYTES ##
## THOSE 4 BYTES SPECIFY THE TYPE OF COMMAND SENT TO THE MOTOR/ RECEIVED FROM THE MOTORS ##
class CommandType(Enum):
    """First byte position in the sent data for CAN messages"""
    RUN = 0x00
    READ = 0x01
    FAULT = 0x02

class SpecType(Enum):
    """Second byte position in the sent data for CAN messages"""
    SPEED = 0x00,
    POSITION = 0x01
    VOLTAGE = 0x02
    CURRENT = 0x03
    GET_CURRENT_FAULTS = 0x04
    GET_ALL_FAULTS = 0x05
    ACKNOWLEDGE_FAULTS = 0x06
    GET_CURRENT_STATE = 0x07
    STOP_MOTOR = 0xFF

class DirectionType(Enum):
    """Third byte position in the sent data for CAN messages"""
    FORWARD_CW = 0x00
    BACKWARD_CCW = 0x01

class ErrorType(Enum):
    """Fourth and last byte position in the sent data for CAN messages"""
    UNRECOGNIZABLE_REQUEST = 0x00 # the user has used something other than the 3 possible options
    UNREACHABLE_SPECIFICATION = 0x01 # the user is asking for something it cannot given the request given to the ESC
    CANNOT_START_MOTOR = 0x02
    CANNOT_STOP_MOTOR = 0x03

## ID OF THE CANBUS (HAS NOTHING TO DO WITH THE DATA BYTES ABOVE) ##
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


"""your task is to essentially create the outline of a function, 
which allows me to alter the first 4 bytes of the message which is being sent, 
and fit the rest of the message with a float. 
(If you have time, you can have a function to implement some of these functions, 
like one for running motor and stopping, one for reading and one for faults, 
and then that is plenty since we can copy and paste this format for different types of commands from then on
"""
def create_message(senderID: IDNumber, command: CommandType, specification: SpecType = 0x10, direction: DirectionType = 0x10, error: ErrorType = 0x10, message: float = 0, ):
    """This function specifies the 8 data sent/received via CANbus
    
    Params:
    - command: one of the commands in the CommandType Enumeration
    - specification: one of the specs in the SpecType Enumeration
        - Default value: 0x10 since the FAULT command does not require any specifications
    - direction: one of the two directions in the DirecitonType
        - Defaut value: 0x10 since some commands do not need a direction)
    - error: one of the errors in ErrorType Enumeration
        - Default value: 0x10 since only the FAULT command requires this data byte
    - message: float value that represents the speed, postion, voltage and current values received or sent
        - Default value: 0 since some commands do not need a float
    
    Returns:
    - CANMessage: the message in CAN format
    - -1        : if any error occurs      
    
    """


    # TODO: reutrn error for combinations that dont work

    full_message = [command.value, specification.value, direction.value, error.value, message]

    try:
        message = CANMessage(senderID = senderID.value, DLC = 8, message=full_message)
    except can.CanError as e:
        print(f"Could not create message due to: {e}")
        return -1
    
    return full_message


class CANMessage:
    def __init__(self, senderID, DLC: int, message = list[int]): 
        """Creates a CAN message
        param:      senderID: any of the list of IDNumber
                    DLC     : from 0 to 8 bytes of sent info
                    message : is no longer than DLC and contains the information
                    message : data bytes of the message
                
        returns:    None
        
        """
        if type(senderID) == str:
            self.senderID = IDNumber[senderID]
        elif type(senderID) == int:
            self.senderID = IDNumber(senderID)
        self.DLC = DLC
        
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

## LIST OF FUNCTIONS ##
def run_speed(senderID, speed, station: CANStation):
    try:
        my_msg = CANMessage(senderID, 8, 'RUN', 'SPEED', [speed])
        station.send_msg(my_msg)
    except can.CanError as e:
        print(f"Command not successfully ran due to {e}")

    

# def run_position(senderID, speed, station: CANStation):
#     return create_can_message(CommandType.RUN, CommandSpec.POSITION, additional_data)

# def read_speed(senderID, speed, station: CANStation):
#     return create_can_message(CommandType.READ, CommandSpec.SPEED, additional_data)

# def read_position(senderID, speed, station: CANStation):
#     return create_can_message(CommandType.READ, CommandSpec.POSITION, additional_data)

# def read_voltage(senderID, speed, station: CANStation):
#     return create_can_message(CommandType.READ, CommandSpec.VOLTAGE, additional_data)

# def read_current(senderID, speed, station: CANStation):
#     return create_can_message(CommandType.READ, CommandSpec.CURRENT, additional_data)

# def read_temperature(senderID, speed, station: CANStation):
#     return create_can_message(CommandType.READ, CommandSpec.TEMPERATURE, additional_data)

# def fault_speed(senderID, speed, station: CANStation):
#     return create_can_message(CommandType.FAULT, CommandSpec.SPEED, additional_data)

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
