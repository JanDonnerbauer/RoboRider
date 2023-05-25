#!/usr/bin/env pybricks-micropython

from Constants import *
from pybricks.tools import wait
from pybricks.robotics import DriveBase
import time

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

        self.WHITE = 0
        self.BLACK = 0

        # Initialize the drive base.
        self.drive_base = DriveBase(self.left_motor, self.right_motor, WHEEL_DIAMETER, AXLE_TRACK)

        # Count time after we lost a line
        self.end_of_line_counter = 0
    
    def start(self):
        self.ev3.screen.print('Press middle ')
        self.ev3.screen.print('button on black line')
        while self.ev3.buttons.pressed() == []:
            pass
        self.calibration()
        while True:	
            while self.ev3.buttons.pressed() == []:	
                pass	
            if self.check_start_position() == True:	
                self.current_state = 'follow_line'	
                self.ev3.screen.clear()	
                self.ev3.speaker.say('LETS GO')	
                self.ev3.screen.print('LETS GO!')	
                self.run()	
                break	
            else:	
                self.ev3.screen.print('Invalid position')
            
    # starts robot
    def run(self):
        while True:
            if self.current_state == 'follow_line':
                self.follow_line()
            elif self.current_state == 'idle':
                self.idle()

            # wait for a short time
            wait(10)

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
        print(self.infrared.distance())

    def check_for_obstacle(self):
        color = self.check_color()
        if self.ultrasonic.distance() <= OBSTACLE_DISTANCE_SMALL or color != None:
            color = self.check_color()
            if color == COLOR_RIGHT:
                self.turn_left()
            elif color == COLOR_LEFT:
                self.turn_right()
            else:
                color = self.check_color()
                while (color != COLOR_LEFT and color != COLOR_RIGHT and color != None) or self.ultrasonic.distance() <= OBSTACLE_DISTANCE_SMALL:
                    color = self.check_color()
                    self.current_state = 'idle'
                    self.idle()
                    print(color)
                self.current_state = 'follow_line'
                self.reset_drive_speed()
                self.check_for_obstacle()
                

        elif self.ultrasonic.distance() <= OBSTACLE_DISTANCE and self.current_state != 'idle':
            self.set_drive_speed(DRIVE_SPEED_SLOW)

    
    def check_color(self):
        return self.tower_sensor.color()

    def check_end_of_line(self):
        print('end of line ', self.end_of_line_counter)
        if self.line_sensor.reflection() in range(self.WHITE-8, self.WHITE+10):
            if self.end_of_line_counter <= 30:
                self.end_of_line_counter += 1
            else:
                self.end_of_line_counter = 0
                for i in range(0,7):
                    # making turns for 10 degrees until line is found
                    if self.line_sensor.reflection() <= self.BLACK+3:
                        return
                    self.drive_base.turn(-11)

                for i in range(0,11):
                    # making turns for 10 degrees until line is found
                    if self.line_sensor.reflection() <= self.BLACK+3:
                        return
                    self.drive_base.turn(-10*-1)
                self.current_state = 'idle'
                self.idle()
        else:
            self.end_of_line_counter = 0


     
    def check_end_of_table(self):
        if self.infrared.distance() >= TABLE_END:
            self.current_state = 'idle'
            self.idle()
            print('end of table')
            # debug output
            # self.ev3.screen.print(self.ultrasonic.distance())
            # self.ev3.speaker.beep()
    
    # def turn(self, side):

    #     self.drive_base.straight(-OBSTACLE_DISTANCE)
    #     # turn left on 110 degrees
    #     self.drive_base.turn(-110*side)

    #     # looking for line
    #     while self.line_sensor.reflection() > self.BLACK+2:
    #         self.check_end_of_table()
    #         while self.ultrasonic.distance() <= OBSTACLE_DISTANCE:
    #             self.idle()
    #         self.current_state = 'follow_line'
    #         self.set_param_drive(DRIVE_SPEED+50, TURN_ANGLE*side)


    #     # going straight for 7 cm
    #     self.drive_base.straight(70)

    #     while self.line_sensor.reflection() > self.BLACK+5:
    #         # making turns for 10 degrees until line is found
    #         self.drive_base.turn(-10*side)

    def turn(self, side):


        # looking for line
        while self.line_sensor.reflection() > self.BLACK+2:
            self.check_end_of_table()
            while self.ultrasonic.distance() <= OBSTACLE_DISTANCE:
                self.idle()
            self.current_state = 'follow_line'
            self.set_param_drive(DRIVE_SPEED+50, TURN_ANGLE*side)


        # going straight for 7 cm
        self.drive_base.straight(70)

        while self.line_sensor.reflection() > self.BLACK+5:
            # making turns for 10 degrees until line is found
            self.drive_base.turn(-10*side)


    def turn_left(self):
        self.drive_base.drive(-DRIVE_SPEED, -200)
        wait(550)
        # making turn left, switching sides, by multiplying values by -1
        self.turn(1)

    def turn_right(self):
        
        self.drive_base.drive(-DRIVE_SPEED, 100)
        wait(550)
        # making turn right, coefficient is 1 -> no changes
        self.turn(-1)

    def idle(self):
        # TODO check for color again
        self.drive_base.stop()

    def reset_drive_speed(self):
        self.set_drive_speed(DRIVE_SPEED)
        
    def set_drive_speed(self, drive_speed):
        deviation = self.line_sensor.reflection() - LINE_THRESHOLD

        # calculate the turn rate.
        turn_rate = PROPORTIONAL_GAIN * deviation

        # set the drive base speed and turn rate.
        self.drive_base.drive(drive_speed, turn_rate)

    def set_turn_rate(self, turn_rate):
        self.drive_base.drive(DRIVE_SPEED, turn_rate)

    def set_param_drive(self, drive_speed, turn_rate):
        self.drive_base.drive(drive_speed, turn_rate)

    def check_start_position(self):
        if self.line_sensor.reflection() not in range(self.BLACK-5, self.BLACK +5):
            self.current_state = 'idle'
            self.ev3.screen.print('Invalid position')
            self.idle()
            return False
        else:
            return True

    def calibration(self):
        while self.ev3.buttons.pressed() == []:
            pass
        self.BLACK = self.line_sensor.reflection()
        self.ev3.screen.clear()
        self.ev3.screen.print('black value = ', self.BLACK)
        wait(1000)
        self.drive_base.turn(30)
        wait(200)
        self.WHITE = self.line_sensor.reflection()
        self.ev3.screen.clear()
        self.ev3.screen.print('white value = ', self.WHITE)
        wait(1000)
        self.drive_base.turn(-30)
        self.ev3.screen.clear()
        self.ev3.screen.print('Press middle ')
        self.ev3.screen.print('button to start')
        