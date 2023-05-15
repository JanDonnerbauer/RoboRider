#!/usr/bin/env pybricks-micropython

from pybricks.parameters import Port, Color

# Initialize ports.
MOTOR_L_PORT        = Port.A
MOTOR_R_PORT        = Port.D
COLOR_SEN_PORT      = Port.S4
COLOR_SEN_NXT_PORT  = Port.S1
INFRARED_PORT       = Port.S3
ULTRASONIC_PORT     = Port.S2

# Calculate the light threshold. CALIBRATION NEEDED!
BLACK               = 8
WHITE               = 50
LINE_THRESHOLD      = (BLACK + WHITE) / 2

# Speed and turn constants
DRIVE_SPEED         = 100
DRIVE_SPEED_SLOW         = DRIVE_SPEED/2
RIGHT_TURN_CONSTANT = 70
LEFT_TURN_CONSTANT  = -70
TURN_ANGLE          = 40

# Others
WHEEL_DIAMETER      = 55.5
AXLE_TRACK          = 104
OBSTACLE_DISTANCE   = 70
OBSTACLE_DISTANCE_SMALL = 35
TABLE_END           = 22
COLOR_RIGHT         = Color.RED
COLOR_LEFT          = Color.BLUE

# Set the gain of the proportional line controller. This means that for every
# percentage point of light deviating from the threshold, we set the turn
# rate of the drivebase to 1.2 degrees per second.

# For example, if the light value deviates from the threshold by 10, the robot
# steers at 10*1.2 = 12 degrees per second.
PROPORTIONAL_GAIN   = 1