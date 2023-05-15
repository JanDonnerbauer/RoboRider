#!/usr/bin/env pybricks-micropython

# from pybricks.nxtdevices import ColorSensor
from pybricks.ev3devices import InfraredSensor, ColorSensor
from pybricks.parameters import Port
from pybricks.hubs import EV3Brick
from pybricks.tools import wait

ev3 = EV3Brick()

colorSen = ColorSensor(Port.S4)
# ultra = InfraredSensor(Port.S2)

while True:
    reflection = colorSen.reflection()
    # distance = ultra.distance()
    ev3.screen.print(reflection)
    wait(100)
