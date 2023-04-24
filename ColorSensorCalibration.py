#!/usr/bin/env pybricks-micropython

from pybricks.ev3devices import InfraredSensor
from pybricks.parameters import Port
from pybricks.hubs import EV3Brick
from pybricks.tools import wait

ev3 = EV3Brick()

colorSen = InfraredSensor(Port.S3)

while True:
    reflection = colorSen.distance()
    ev3.screen.print(reflection)
    wait(100)
