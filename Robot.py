from Constants import *
from pybricks.tools import wait
from pybricks.robotics import DriveBase

from pybricks.ev3devices import Motor, ColorSensor, InfraredSensor, UltrasonicSensor
from pybricks.nxtdevices import ColorSensor as ColorSensorNXT
from pybricks.hubs import EV3Brick
from pybricks.robotics import DriveBase

class Robot(object):
    def __init__(self):
        self.current_state = 'idle'
        self.ev3 = EV3Brick()

        # Initialize the motors.
        self.left_motor = Motor(MOTOR_L_PORT)
        self.right_motor = Motor(MOTOR_R_PORT)

        # Initialize the color sensors.
        self.line_sensor = ColorSensor(COLOR_SEN_PORT)
        self.tower_sensor = ColorSensorNXT(COLOR_SEN_NXT_PORT)

        # Initialize the distance sensors.
        self.infrared = InfraredSensor(INFRARED_PORT)
        self.ultrasonic = UltrasonicSensor(ULTRASONIC_PORT)

        # Initialize the drive base.
        self.drive_base = DriveBase(self.left_motor, self.right_motor, WHEEL_DIAMETER, AXLE_TRACK)
    
    def start(self):
        self.current_state = 'follow_line'
        self.run()

    # starts robot
    def run(self):
        if self.current_state == 'follow_line':
            self.follow_line()
        elif self.current_state == 'idle':
            self.idle()

        # wait for a short time
        # TODO necessary?
        wait(10)

        # executes run again (working loop)
        self.run()

    # follows the line until obstacle reached or end of line
    def follow_line(self):
        if self.current_state != 'follow_line':
            self.stop()
        
        # calculate the deviation from the threshold.
        deviation = self.line_sensor.reflection() - LINE_THRESHOLD

        # calculate the turn rate.
        turn_rate = PROPORTIONAL_GAIN * deviation

        # set the drive base speed and turn rate.
        self.drive_base.drive(DRIVE_SPEED, turn_rate)

        # checks obstacle detected
        self.check_for_obstacle()

        # checks end of line
        self.check_end_of_line()

        # checks for end of table
        self.check_end_of_table()

    def check_for_obstacle(self):
        if self.ultrasonic.distance() <= OBSTACLE_DISTANCE:
            color = self.check_color()
            if color == COLOR_RIGHT:
                self.turn_left()
            elif color == COLOR_LEFT:
                self.turn_right()
            else:
                self.current_state = 'idle'
                self.check_for_obstacle()

        elif self.ultrasonic.distance() <= OBSTACLE_DISTANCE_SMALL:
            self.set_drive_speed(DRIVE_SPEED_SLOW)

    
    def check_color(self):
        return tower_sensor.color()
        print("check color")

    def check_end_of_line(self):
        # TODO
        # print("turns left")
        pass
    
    def check_end_of_table(self):
        if self.infrared.distance() >= TABLE_END:
            self.current_state = State.IDLE
            # debug output
            # self.ev3.screen.print(self.ultrasonic.distance())
            # self.ev3.speaker.beep()
    
    def turn(self, side):

        self.drive_base.straight(-OBSTACLE_DISTANCE)

        #turn left on 100 degrees
        self.drive_base.turn(-100*side)

        #looking for line
        while self.line_sensor.reflection() > BLACK+2:
            self.set_turn_rate(turn_angle*side)
        

        #going straight for 7 cm
        self.drive_base.straight(70)

        while self.line_sensor.reflection() > BLACK+2:
            robot.turn(-10*side)


    def turn_left(self):
        # TODO
        self.turn(-1)

    def turn_right(self):
        # TODO
        self.turn(1)

    def idle(self):
        # TODO check for color again
        self.drive_base.stop()

    def reset_drive_speed(self):
        self.set_drive_speed(DRIVE_SPEED)
        
    def set_drive_speed(self, drive_speed):
        self.drive_base.drive(drive_speed, self.drive_base.state().turn_rate)

    def set_turn_rate(self, turn_rate):
        self.drive_base.drive(self.drive_base.state().speed, turn_rate)