#!/usr/bin/env pybricks-micropython

"""
This program requires LEGO® EV3 MicroPython v2.0 or higher.
"""

from pybricks.ev3devices import Motor, ColorSensor, InfraredSensor, UltrasonicSensor
from pybricks.nxtdevices import ColorSensor as ColorSensorNXT
from pybricks.parameters import Port
from pybricks.hubs import EV3Brick
from pybricks.parameters import Color
from pybricks.tools import wait
from pybricks.robotics import DriveBase

# Initialize ports.
motorL = Port.A
motorR = Port.D
colorSenEv3 = Port.S4
colorSenNXT = Port.S1
infraredSen = Port.S3
ultrasonicSen = Port.S2
wheel_diameter_ = 55.5
towerDistance = 30
towerSpeedDistance = 70
tableEnd = 150
turn_angle = 18
seventydegrees = 63
DRIVE_SPEED = 100

ev3 = EV3Brick()
left_motor = Motor(motorL)
right_motor = Motor(motorR)
line_sensor = ColorSensor(colorSenEv3)
tower_sensor = ColorSensorNXT(colorSenNXT)
infrared = InfraredSensor(infraredSen)
ultrasonic = UltrasonicSensor(ultrasonicSen)
robot = DriveBase(left_motor, right_motor, wheel_diameter=wheel_diameter_, axle_track=104)

# Calculate the light threshold. CALIBRATION NEEDED!
BLACK = 7
WHITE = 47
threshold = (BLACK + WHITE) / 2
PROPORTIONAL_GAIN = 4

# Start following the line endlessly.
while True:
    # if state = 'Table end':
    # if state = 'Obstacle':
    # if state = 'tower':
    # if state = 'other object':
    # if state = 'last tower':
    # if state = 'line following':
    if infrared.distance() >= 20:
        robot.drive(0, 0)
        ev3.screen.print(ultrasonic.distance())
        ev3.speaker.beep()
        wait(50000)
    if ultrasonic.distance() <= towerDistance:
        ev3.screen.print(tower_sensor.color(), tower_sensor.color() == 'Color.BLUE' or tower_sensor.color() == 'Color.RED')
        side = 1
        if tower_sensor.color() == Color.BLUE:
            side  = 1
        elif tower_sensor.color() == Color.RED:
            side = -1
        
        robot.straight(-70)

        #make a slight turn right or left for 1.2 seconds
        robot.turn(-100*side)

        #looking for line
        while line_sensor.reflection() > BLACK+2:
            robot.drive(DRIVE_SPEED, turn_angle*side)
        

        #going straight for 0.2 seonds
        robot.straight(70)

        while line_sensor.reflection() > BLACK+2:
            robot.turn(-10*side)
        deviation = line_sensor.reflection() - threshold

    # Calculate the turn rate.
        turn_rate = PROPORTIONAL_GAIN * deviation

    # Set the drive base speed and turn rate.
        robot.drive(DRIVE_SPEED, threshold)

    elif ultrasonic.distance() <= towerSpeedDistance:
        DRIVE_SPEED = 50
        deviation = line_sensor.reflection() - threshold

        # Calculate the turn rate.
        turn_rate = PROPORTIONAL_GAIN * deviation

        # Set the drive base speed and turn rate.
        robot.drive(DRIVE_SPEED, turn_rate)

        # You can wait for a short time or do other things in this loop.
        wait(10)
    else:
        
        DRIVE_SPEED = 100
    #Check if the robot drove through the gate
    # if tower_sensor.color == Color.RED or tower_sensor.color == Color.BLUE:
    #     ev3.speaker.beep()

    # Calculate the deviation from the threshold.
    deviation = line_sensor.reflection() - threshold

    # Calculate the turn rate.
    turn_rate = PROPORTIONAL_GAIN * deviation

    # Set the drive base speed and turn rate.
    robot.drive(DRIVE_SPEED, turn_rate)

    # You can wait for a short time or do other things in this loop.
    wait(10)