#!/usr/bin/env pybricks-micropython

"""
This program requires LEGO® EV3 MicroPython v2.0 or higher.
"""

from pybricks.ev3devices import Motor, ColorSensor, InfraredSensor, UltrasonicSensor
from pybricks.nxtdevices import ColorSensor as ColorSensorNXT
from pybricks.parameters import Port
from pybricks.hubs import EV3Brick
from pybricks.tools import wait
from pybricks.robotics import DriveBase

# Initialize ports.
motorL = Port.C
motorR = Port.D
colorSenEv3 = Port.S4
# colorSenNXT = Port.S3
infraredSen = Port.S3
ultrasonicSen = Port.S1
wheel_diameter_ = 55.5
towerDistance = 130
tableEnd = 150

ev3 = EV3Brick()

# Initialize the motors.
left_motor = Motor(motorL)
right_motor = Motor(motorR)

# Initialize the color sensors.
line_sensor = ColorSensor(colorSenEv3)
# tower_sensor = ColorSensorNXT(colorSenNXT)

# Initialize the distance sensors.
infrared = InfraredSensor(infraredSen)
ultrasonic = UltrasonicSensor(ultrasonicSen)

# Initialize the drive base.
robot = DriveBase(left_motor, right_motor, wheel_diameter=wheel_diameter_, axle_track=104)

# Calculate the light threshold. CALIBRATION NEEDED!
BLACK = 9
WHITE = 35
threshold = (BLACK + WHITE) / 2

# Set the drive speed at 100 millimeters per second.
DRIVE_SPEED = 100

# Set the gain of the proportional line controller. This means that for every
# percentage point of light deviating from the threshold, we set the turn
# rate of the drivebase to 1.2 degrees per second.

# For example, if the light value deviates from the threshold by 10, the robot
# steers at 10*1.2 = 12 degrees per second.
PROPORTIONAL_GAIN = 7

# Start following the lie endlessly.
while True:
    # Check if the robot is not near table end
    if  infrared.distance() >= 22:
        robot.drive(0, 0)
        ev3.screen.print(ultrasonic.distance())
        ev3.speaker.beep()
        wait(50000)
    elif ultrasonic.distance() <= towerDistance:
        while line_sensor.reflection() > 13:
            robot.drive(10, 180)
            wait(150)
            robot.drive(DRIVE_SPEED, 20)
    else:
        DRIVE_SPEED = 150
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