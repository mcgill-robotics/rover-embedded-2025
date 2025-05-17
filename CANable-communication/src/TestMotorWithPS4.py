import time
import pygame
from driveCANCommunication import CANStation, ESCInterface, DriveInterface, NodeID

# ==== Configuration ====
INTERFACE = "slcan"
CHANNEL   = "COM12"
BITRATE   = 500000

MAX_SPEED = 1500.0
MAX_RAMP_DELTA = 200.0  # adjust as needed


DEAD_ZONE = 100
MESSAGE_DELAY = 0.1   # Seconds between motor commands
TEST_DELAY = 0.5        # Seconds between test prints

# ==== Helper Functions ====

def get_ps4_controller():
    """ Wait for and initialize a PS4 controller. """
    pygame.init()
    pygame.joystick.init()

    joystick = None
    while joystick is None:
        if pygame.joystick.get_count() > 0:
            joystick = pygame.joystick.Joystick(0)
            joystick.init()
            print(f"Detected controller: {joystick.get_name()}")
        else:
            print("Waiting for PS4 controller...")
            time.sleep(2)
            pygame.joystick.quit()
            pygame.joystick.init()

    return joystick

def apply_dead_zone(value, threshold):
    """ Zero out values within the dead zone threshold. """
    return 0.0 if abs(value) < threshold else value

# ==== Main Motor Drive Loop ====
def run_ps4_drive_loop():
    joystick = get_ps4_controller()

    station = CANStation(interface=INTERFACE, channel=CHANNEL, bitrate=BITRATE)
    esc = ESCInterface(station)
    drive = DriveInterface(esc)

    print("PS4 motor drive active. Press Ctrl+C to quit.\n")

    # Initialize smoothed values
    prev_left_cmd = 0.0
    prev_right_cmd = 0.0

    # Max change in speed per iteration (in RPM)


    try:
        while True:
            pygame.event.pump()

            # === Check for X Button Press ===
            if joystick.get_button(0):  # X button
                print("X button pressed! Acknowledging all motor faults...")
                for node in [NodeID.RF_DRIVE, NodeID.RB_DRIVE, NodeID.LB_DRIVE, NodeID.LF_DRIVE]:
                    drive.acknowledge_motor_fault(node)
                    time.sleep(0.05)


            # === Check for Circle Button Press ===
            if joystick.get_button(1):  # Circle button
                print("Circle button pressed! Stopping all motors.")
                drive.broadcast_multi_motor_stop()

            # Read joystick values
            target_left = -joystick.get_axis(1) * MAX_SPEED
            target_right = -joystick.get_axis(3) * MAX_SPEED

            # Apply dead zone
            target_left = apply_dead_zone(target_left, DEAD_ZONE)
            target_right = apply_dead_zone(target_right, DEAD_ZONE)


            left_cmd = target_left
            right_cmd = target_right

            # Save for next iteration
            prev_left_cmd = left_cmd
            prev_right_cmd = right_cmd

            # Print current smoothed values
            print(f"Left Y Setpoint: {left_cmd:.2f} | Right Y Setpoint: {right_cmd:.2f}")

            # Drive mapping: [RF, LF, LB, RB]
            speeds = [right_cmd, left_cmd, left_cmd, right_cmd]
            drive.broadcast_multi_motor_speeds(speeds)

            time.sleep(MESSAGE_DELAY)

    except KeyboardInterrupt:
        print("Stopping motors and exiting...")
        drive.broadcast_multi_motor_stop()
        time.sleep(0.2)
    finally:
        station.close()
        pygame.quit()


# ==== Controller Test Mode ====

def test_ps4_controller():
    joystick = get_ps4_controller()
    print("Starting PS4 joystick test (press Ctrl+C to exit)\n")

    try:
        while True:
            pygame.event.pump()

            # Get left/right stick Y axes
            left_y = -joystick.get_axis(1) * MAX_SPEED
            right_y = -joystick.get_axis(3) * MAX_SPEED

            # Apply dead zones
            left_speed = apply_dead_zone(left_y, DEAD_ZONE)
            right_speed = apply_dead_zone(right_y, DEAD_ZONE)

            print(f"Left = {left_speed:.1f} | Right = {right_speed:.1f}")
            time.sleep(TEST_DELAY)

    except KeyboardInterrupt:
        print("\nTest ended by user.")
    finally:
        pygame.quit()


def test_x_button_press():
    joystick = get_ps4_controller()
    print("Monitoring X button press (press Ctrl+C to exit)...")

    try:
        while True:
            pygame.event.pump()

            if joystick.get_button(0):  # Button 0 is typically the "X" button
                print("X button was pressed!")

            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\nStopped button test.")
    finally:
        pygame.quit()

# ==== Entry Point ====

if __name__ == "__main__":
    # Uncomment one of the following to test or run motors
    run_ps4_drive_loop()
    # test_ps4_controller()
    # test_x_button_press()
