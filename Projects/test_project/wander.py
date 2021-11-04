#!/usr/bin/env pybricks-micropython
"""
"""
import logging

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor, GyroSensor, InfraredSensor
from pybricks.parameters import Port
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch

log = logging.getLogger('FLL')

# Initialize the EV3 Brick.
ev3 = EV3Brick()

# Initialize the Infrared Sensor. It is used to detect
# obstacles as the robot drives around.
infrared_sensor = InfraredSensor(Port.S3)

# Initialize the motors.
left_motor = Motor(Port.A)
right_motor = Motor(Port.B)

# Initialize the motor connected to the arms.
arm_motor = Motor(Port.C)

# Initialize the Color Sensor. It is used to detect the colors that command
# which way the robot should move.
color_sensor = ColorSensor(Port.S2)

# Initialize the gyro sensor. It is used to provide feedback for balancing the
# robot.
gyro_sensor = GyroSensor(Port.S1)

# Initialize the drive base.
robot = DriveBase(left_motor, right_motor, wheel_diameter=55.5, axle_track=124)

# Let us know we are ready to start
ev3.speaker.set_speech_options(voice='f1')
ev3.speaker.play_file('meow.wav')

# The following loop makes the robot drive forward until it detects an
# obstacle. Then it backs up and turns around. It keeps on doing this
# until you stop the program.
while True:
    # Begin driving forward at 200 millimeters per second.
    robot.drive(200, 0)

    # Wait until an obstacle is detected. This is done by repeatedly
    # doing nothing (waiting for 10 milliseconds) while the measured
    # distance is still greater than 300 mm.
    while infrared_sensor.distance() > 15:
        wait(10)

    # Drive backward for 300 millimeters.
    robot.straight(-300)

    # Turn around by 120 degrees
    robot.turn(120)
    log.info('Gyro Angle: %s' % gyro_sensor.angle())
    log.info('Color Sensor: %s' % str(color_sensor.rgb()))
    ev3.speaker.say('where did I leave my human')