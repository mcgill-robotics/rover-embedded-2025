import time
import struct
import pygame
from armCANCommunicationV2 import CANStation, ArmESCInterface, ArmNodeID

frequency_hz = 50
step_time = 1.0 / frequency_hz
desired_dps = 5.0
JOYSTICK_SCALE = desired_dps / frequency_hz
DEADZONE = 0.1

station = CANStation(interface="slcan", channel="COM7", bitrate=500000)
arm = ArmESCInterface(station)

pygame.init()
pygame.joystick.init()
joystick = pygame.joystick.Joystick(0)
joystick.init()

# Confirm which axis is right stick Y
print("Move your joysticks. Watching for axis values:")
for i in range(joystick.get_numaxes()):
    print(f"Axis {i}: {joystick.get_axis(i):.3f}")
print("Now starting control loop...")

elbow_pos = arm.read_position(ArmNodeID.ELBOW) or 0.0

try:
    while True:
        pygame.event.pump()

        right_y = -joystick.get_axis(3)  # UPDATE THIS INDEX BASED ON TEST
        if abs(right_y) < DEADZONE:
            right_y = 0.0

        elbow_pos += right_y * JOYSTICK_SCALE
        print(f"Joystick: {right_y:.2f} | Elbow Pos: {elbow_pos:.2f}")

        arm.run_follower(ArmNodeID.ELBOW, elbow_pos)
        time.sleep(step_time)

except KeyboardInterrupt:
    print("Exiting gracefully...")

finally:
    station.close()
    joystick.quit()
    pygame.quit()



