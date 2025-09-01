import time
import struct
import pygame
from armCANCommunicationV2 import CANStation, ArmESCInterface, ArmNodeID

frequency_hz = 50
step_time = 0.025#1.0 / frequency_hz
desired_dps = 5.0
JOYSTICK_SCALE = desired_dps / frequency_hz
waist_speed_dps = 100.0
WAIST_DELTA = waist_speed_dps / frequency_hz
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
shoulder_pos = arm.read_position(ArmNodeID.SHOULDER) or 0.0
waist_pos = arm.read_position(ArmNodeID.WAIST) or 0.0

try:
    while True:
        pygame.event.pump()

        right_y = -joystick.get_axis(3)  # UPDATE THIS INDEX BASED ON TEST (right joystick)
        if abs(right_y) < DEADZONE:
            right_y = 0.0
        left_y = -joystick.get_axis(1)  # UPDATE THIS INDEX BASED ON TEST (left joystick)
        if abs(left_y) < DEADZONE:
            left_y = 0.0

        right_press = joystick.get_button(4)  # (right bumper)
        left_press = joystick.get_button(5)  # (left bumper)
        waist_delta = 0
        if left_press and not right_press:
            waist_delta = -WAIST_DELTA
        elif right_press and not left_press:
            waist_delta = WAIST_DELTA

        elbow_pos += right_y * JOYSTICK_SCALE
        print(f"Joystick: {right_y:.2f} | Elbow Pos: {elbow_pos:.2f}")
        shoulder_pos += left_y * JOYSTICK_SCALE
        print(f"Joystick: {left_y:.2f} | Waist Pos: {shoulder_pos:.2f}")
        waist_pos += right_press * WAIST_DELTA
        print(f"Joystick: {right_press:.2f} | Waist Pos: {waist_pos:.2f}")

        arm.run_follower(ArmNodeID.ELBOW, elbow_pos)
        time.sleep(step_time)
        arm.run_follower(ArmNodeID.WAIST, shoulder_pos)
        time.sleep(step_time)
        # arm.run_follower(ArmNodeID.WAIST, waist_pos)



except KeyboardInterrupt:
    print("Exiting gracefully...")

finally:
    station.close()
    joystick.quit()
    pygame.quit()