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

from pybricks.ev3devices import Motor, ColorSensor
from pybricks.parameters import Port
from pybricks.tools import wait
from pybricks.robotics import DriveBase

log = logging.getLogger('FLL')

# Initialize the motors.
left_motor = Motor(Port.B)
right_motor = Motor(Port.A)

# Initialize the color sensor.
line_sensor = ColorSensor(Port.S3)

# Initialize the drive base.
robot = DriveBase(left_motor, right_motor, wheel_diameter=55.5, axle_track=124)

# Calculate the light threshold. Choose values based on your measurements.
BLACK = 9
WHITE = 85
threshold = (BLACK + WHITE) / 2

# Set the drive speed at 100 millimeters per second.
DRIVE_SPEED = 50

# Set the gain of the proportional line controller. This means that for every
# percentage point of light deviating from the threshold, we set the turn
# rate of the drivebase to 1.2 degrees per second.

# For example, if the light value deviates from the threshold by 10, the robot
# steers at 10*1.2 = 12 degrees per second.
PROPORTIONAL_GAIN = .21

# Get on Left line
def follow_line_until(callback):

    # Start following the line endlessly.
    while True:
        # Calculate the deviation from the threshold.
        deviation = line_sensor.reflection() - threshold
    
        # Calculate the turn rate.
        turn_rate = -PROPORTIONAL_GAIN * deviation
    
        # Set the drive base speed and turn rate.
        robot.drive(DRIVE_SPEED, turn_rate)
    
        # First Change lines
        if callback():
            return
    
        # Log the drive distance
        log.info('Distance: %s' % robot.distance())
    
        # You can wait for a short time or do other things in this loop.
        wait(10)

# while robot.distance() > -115:
#     robot.drive(-DRIVE_SPEED, 0)

# Reset the distance calculator
robot.reset()
first_change_line = False

follow_line_until(lambda _ = None: robot.distance() < -900)
robot.turn(-40)
robot.reset()
follow_line_until(lambda _ = None: robot.distance() < -390)
while robot.distance() > -40:
    robot.drive(-DRIVE_SPEED, 0)