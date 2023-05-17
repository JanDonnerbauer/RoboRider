#!/usr/bin/env pybricks-micropython

# from pybricks.nxtdevices import ColorSensor
from pybricks.ev3devices import InfraredSensor
from pybricks.parameters import Port
from pybricks.hubs import EV3Brick
from pybricks.tools import wait

ev3 = EV3Brick()

colorSen = InfraredSensor(Port.S3)
# ultra = InfraredSensor(Port.z2)

while True:
    ev3.screen.print(ev3.buttons.pressed())
