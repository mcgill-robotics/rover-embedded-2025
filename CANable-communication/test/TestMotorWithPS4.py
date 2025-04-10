import time
import pygame
from communication import (
    CANStation, 
    ESCInterface, 
    FORWARD_CW, 
    BACKWARD_CCW,
    RUN_REQUEST,
    SPEED,
    STOP_MOTOR,
    READ_REQUEST,
    VOLTAGE,
    CURRENT,
    GET_CURRENT_STATE
)

########################################
# Configuration for your CAN bus setup #
########################################
INTERFACE = "slcan"     # e.g. "slcan", "socketcan", or "pcan"
CHANNEL   = "COM12"     # e.g. "can0" on Linux, or "COM12" on Windows
BITRATE   = 500000      # Must match your bus bit rate

########################################
# Control Variables
########################################
max_speed = 2500.0              # Maximum speed (your chosen units)
dead_zone = 70.0                # Dead zone threshold for joystick speed
MESSAGE_DELAY = 0.2         # Delay (in seconds) between each CAN message loop

########################################
# Main Script
########################################
def main():
    # Initialize pygame for joystick input
    pygame.init()
    pygame.joystick.init()

    # Wait until the user connects a PS4 controller
    joystick = None
    while joystick is None:
        count = pygame.joystick.get_count()
        if count > 0:
            joystick = pygame.joystick.Joystick(0)
            joystick.init()
            print(f"Detected joystick: {joystick.get_name()}")
        else:
            print("No joystick detected. Please connect a PS4 controller.")
            time.sleep(2)
            pygame.joystick.quit()
            pygame.joystick.init()

    # Initialize the CAN station and the ESC interface
    station = CANStation(interface=INTERFACE, channel=CHANNEL, bitrate=BITRATE)
    esc = ESCInterface(station)

    print("Press Ctrl+C to quit.\n")
    
    try:
        while True:
            # Pump the pygame event loop so we can read joystick values
            pygame.event.pump()

            # Only consider Y-axes:
            # Left stick Y for motors 0x001 and 0x002
            left_y_axis = joystick.get_axis(3)
            # Right stick Y for motors 0x003 and 0x004
            right_y_axis = joystick.get_axis(1)

            # Convert joystick axes to speed commands (invert so stick "up" is positive)
            left_speed_command = -left_y_axis * max_speed
            right_speed_command = -right_y_axis * max_speed

            # ---------------------
            #    DEAD ZONE LOGIC
            # ---------------------
            if abs(left_speed_command) < dead_zone:
                left_speed_command = 0.0
            if abs(right_speed_command) < dead_zone:
                right_speed_command = 0.0

            # ---------------------
            #    DIRECTION LOGIC
            # ---------------------
            # For left motors (IDs 0x001 and 0x002) based on provided straight run logic:
            #   - Motor 0x001: forward = 0, reverse = 1
            #   - Motor 0x002: forward = 1, reverse = 0
            if left_speed_command > 0:
                dir_0x001 = 0
                dir_0x002 = 1
            elif left_speed_command < 0:
                dir_0x001 = 1
                dir_0x002 = 0
            else:
                # When speed is zero, choose default "forward" directions
                dir_0x001 = 0
                dir_0x002 = 1

            # For right motors (IDs 0x003 and 0x004):
            #   - Both motors: forward = 1, reverse = 0
            if right_speed_command > 0:
                dir_0x003 = 1
                dir_0x004 = 1
            elif right_speed_command < 0:
                dir_0x003 = 0
                dir_0x004 = 0
            else:
                dir_0x003 = 1
                dir_0x004 = 1

            # ---------------------
            #   SEND TO MOTORS
            # ---------------------
            abs_left_speed = abs(left_speed_command)
            abs_right_speed = abs(right_speed_command)

            # Left motors
            esc.run_speed(abs_left_speed, dir_0x001, node_id=0x001)
            station.recv_msg(timeout=0.02)
            esc.run_speed(abs_left_speed, dir_0x002, node_id=0x002)
            station.recv_msg(timeout=0.02)

            # Right motors
            esc.run_speed(abs_right_speed, dir_0x003, node_id=0x003)
            station.recv_msg(timeout=0.02)
            esc.run_speed(abs_right_speed, dir_0x004, node_id=0x004)
            station.recv_msg(timeout=0.02)

            # Wait for the defined delay before the next loop iteration
            time.sleep(MESSAGE_DELAY)

    except KeyboardInterrupt:
        print("Stopping motors and exiting...")
        for motor_id in [0x001, 0x002, 0x003, 0x004]:
            esc.stop_motor(node_id=motor_id)
            station.recv_msg(timeout=0.02)
    finally:
        station.close()
        pygame.quit()


if __name__ == "__main__":
    main()
