#!/usr/bin/env pybricks-micropython

#-------------------
#
# Gripper Robot examples
# 
# 
#-------------------

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile


# This program requires LEGO EV3 MicroPython v2.0 or higher.

gripper_state = 0

# Create your objects here.
ev3 = EV3Brick()

grip_motor = Motor(Port.A)

left_motor = Motor(Port.B)
right_motor = Motor(Port.C)

# The wheel diameter of Gripper is about 45 mm.
WHEEL_DIAMETER = 45

# The axle track is the distance between the centers of each of the
# wheels.  This is about 150 mm for Gripper.
AXLE_TRACK = 150

# The Driving Base is comprised of 2 motors.  There is a wheel on each
# motor.  The wheel diameter and axle track values are used to make the
# motors move at the correct speed when you give a drive command.
robot = DriveBase(left_motor, right_motor, WHEEL_DIAMETER, AXLE_TRACK)

infrared_sensor = InfraredSensor(Port.S4)

# Beep - Init ready

ev3.speaker.beep()

# Program --------------------------- 


def gripper_reset():
    global _gripper_state

    _gripper_state = 0
    grip_motor.run_until_stalled(-1000, then=Stop.COAST, duty_limit=50)


def gripper_arm():
    global _gripper_state

    if _gripper_state == 0:
        gripper_reset()

    speed = 1000

    if _gripper_state >= 2:
        speed = - speed
        _gripper_state = 1
    else:
        _gripper_state = 2

    grip_motor.run_until_stalled(speed, then=Stop.COAST, duty_limit=50)


def stop_drive_on_hit(dist=25):
    checking = True

    while checking:
        checking = infrared_sensor.distance() > 25
        wait(10)

    robot.drive(0, 0)


def main():

    gripper_reset()
    ev3.speaker.say("I'm Ready")

    ev3.speaker.beep()

    robot.drive(200, 0)

    stop_drive_on_hit()

    ev3.speaker.play_file(SoundFile.AIRBRAKE)
    gripper_arm()

    robot.turn(-360)

    robot.drive(200, 0)
    stop_drive_on_hit()

    gripper_arm()
    ev3.speaker.play_file(SoundFile.AIR_RELEASE)

    robot.stop()
    ev3.speaker.say("Program done")

# ---------------------------


main()
