#!/usr/bin/env pybricks-micropython
"""
Example LEGO® MINDSTORMS® EV3 Robot Educator Color Sensor Down Program
----------------------------------------------------------------------

This program requires LEGO® EV3 MicroPython v2.0.
Download: https://education.lego.com/en-us/support/mindstorms-ev3/python-for-ev3

Building instructions can be found at:
https://education.lego.com/en-us/support/mindstorms-ev3/building-instructions#robot
"""
import logging
import datetime

from pybricks.ev3devices import Motor, ColorSensor
from pybricks.parameters import Port
from pybricks.tools import wait, StopWatch
from pybricks.robotics import DriveBase
from PID import PID

log = logging.getLogger('FLL')

# Initialize the motors.
left_motor = Motor(Port.B)
right_motor = Motor(Port.A)

# Initialize the color sensor.
line_sensor = ColorSensor(Port.S3)

# Initialize the drive base.
robot = DriveBase(left_motor, right_motor, wheel_diameter=55.5, axle_track=124)

while True:
    log.info(datetime.datetime.now())

# # Calculate the light threshold. Choose values based on your measurements.
# BLACK = 5
# WHITE = 67
# threshold = (BLACK + WHITE) / 2
# 
# # Set the drive speed at 100 millimeters per second.
# DRIVE_SPEED = 150
# 
# # Set the gain of the proportional line controller. This means that for every
# # percentage point of light deviating from the threshold, we set the turn
# # rate of the drivebase to 1.2 degrees per second.
# 
# # For example, if the light value deviates from the threshold by 10, the robot
# # steers at 10*1.2 = 12 degrees per second.
# 
# # Get on Left line
# TARGET_LOOP_PERIOD = 10  # ms
# pid = PID(12.0, 0.0, 0.00, setpoint=0)
# def follow_line_until(callback):
# 
#     single_loop_timer = StopWatch()
#     control_loop_timer = StopWatch()
#     control_loop_count = 0
#     # Start following the line endlessly.
#     while True:
#         single_loop_timer.reset()
#         if control_loop_count == 0:
#             average_control_loop_period = TARGET_LOOP_PERIOD / 1000.
#             control_loop_timer.reset()
#         else:
#             average_control_loop_period = (control_loop_timer.time() / 1000 / control_loop_count)
#         control_loop_count += 1
# 
#         # Calculate the turn rate.
#         error = line_sensor.reflection() - threshold
#         control = pid(error)
#         turn_rate = control
#         if abs(control) > 350:
#             speed = 0
#         else:
#             speed = 0
#     
#         # Set the drive base speed and turn rate.
#         log.info('Turn Rate: %s Speed: %s Control: %s Error: %s' % (turn_rate, speed, control, error))
# 
#         robot.drive(speed, turn_rate)
#     
#         # First Change lines
#         if callback():
#             return
#     
#         # You can wait for a short time or do other things in this loop.
#         wait(TARGET_LOOP_PERIOD - single_loop_timer.time())
# # Effective range of speed 0-400
# # Effective range of turn 0-400
# # Error ranges from -30 - 30
# 
# 
# # Reset the distance calculator
# robot.reset()
# follow_line_until(lambda _ = None: robot.distance() > 100)
# # wait(100)
# # Calibration of light reflection sensor
# # while True:
# #     log.info(line_sensor.reflection())