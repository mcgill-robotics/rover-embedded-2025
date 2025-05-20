import time
import pygame
from driveCANCommunication import CANStation, ESCInterface, DriveInterface, NodeID

# ==== Configuration ====
INTERFACE      = "slcan"
CHANNEL        = "COM12"
BITRATE        = 500000

MAX_SPEED      = 2000.0
MAX_RAMP_DELTA = 200.0  # adjust as needed

DEAD_ZONE   = 100
MESSAGE_DELAY = 0.1   # Seconds between motor commands
TEST_DELAY    = 0.5   # Seconds between test prints

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

def get_all_motor_status(drive, station):
    """
    Query each motor for faults and print a ✓ if no fault, ✗ if fault present.
    """
    motor_names = ['RF Drive', 'RB Drive', 'LB Drive', 'LF Drive']
    nodes       = [
        NodeID.RF_DRIVE,
        NodeID.RB_DRIVE,
        NodeID.LB_DRIVE,
        NodeID.LF_DRIVE
    ]

    print("\nMotor Fault Status:")
    for name, node in zip(motor_names, nodes):
        drive.read_all_faults(node)
        ok = (station.recv_msg(0.02) == 0)
        symbol = '✓' if ok else '✗'
        print(f"  {symbol}  {name}")
    print()  # blank line

# ==== Main Motor Drive Loop ====

def run_ps4_drive_loop():
    joystick = get_ps4_controller()

    station = CANStation(interface=INTERFACE, channel=CHANNEL, bitrate=BITRATE)
    esc     = ESCInterface(station)
    drive   = DriveInterface(esc)

    print("PS4 motor drive active. Press Ctrl+C to quit.\n")

    # track previous square-button state so we only act on transitions
    prev_square = False

    try:
        while True:
            pygame.event.pump()

            # === X Button: acknowledge faults ===
            if joystick.get_button(0):  # X button
                print("X button pressed! Acknowledging all motor faults...")
                for node in [NodeID.RF_DRIVE, NodeID.RB_DRIVE, NodeID.LB_DRIVE, NodeID.LF_DRIVE]:
                    drive.acknowledge_motor_fault(node)
                    time.sleep(0.05)

            # === Circle Button: stop all motors ===
            if joystick.get_button(1):  # Circle button
                print("Circle button pressed! Stopping all motors.")
                drive.broadcast_multi_motor_stop()

            # === Square Button: print status of each motor ===
            square = joystick.get_button(2)  # Square button
            if square and not prev_square:
                print("Square button pressed! Fetching motor status...")
                get_all_motor_status(drive, station)
            prev_square = square

            # === Read analog sticks and drive ===
            target_left  = -joystick.get_axis(1) * MAX_SPEED
            target_right = -joystick.get_axis(3) * MAX_SPEED

            target_left  = apply_dead_zone(target_left, DEAD_ZONE)
            target_right = apply_dead_zone(target_right, DEAD_ZONE)

            left_cmd  = -target_left
            right_cmd = target_right

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

# ==== Controller Test Modes ====
# (unchanged: test_ps4_controller, test_x_button_press)

if __name__ == "__main__":
    run_ps4_drive_loop()
    # test_ps4_controller()
    # test_x_button_press()
