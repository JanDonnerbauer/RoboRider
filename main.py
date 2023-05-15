#!/usr/bin/env pybricks-micropython

from pybricks.tools import wait
from pybricks.robotics import DriveBase

from pybricks.ev3devices import Motor, ColorSensor, InfraredSensor, UltrasonicSensor
from pybricks.nxtdevices import ColorSensor as ColorSensorNXT
from pybricks.hubs import EV3Brick
from pybricks.robotics import DriveBase
from Constants import *
from Robot import Robot

# starts the robot

robot = Robot()
robot.start()



