
import RPi.GPIO as gpio
import time
import numpy as np
import serial
import cv2
import imutils
import picamera
from picamera.array import PiRGBArray
from picamera import PiCamera
# For Email method
import os
from datetime import datetime
import smtplib
from email.mime.multipart import MIMEMultipart
from email.mime.text import MIMEText
from email.mime.image import MIMEImage
from tkinter.messagebox import NO

import imaplib


class Robot:

    def __init__(self, monitor_encoders=False,
                 monitor_imu=False,
                 debug_mode=False,
#                  start_x=0.3048,
#                  start_y=0.3048,
                 start_x=0.2096,
                 start_y=0.2096,
                 monitor_pose=False,
                 ser=serial.Serial('/dev/ttyUSB0', 9600)):

        # Save Encoder data to text files
        self.monitor_encoders = monitor_encoders

        self.debug_mode = debug_mode

        self.monitor_pose = monitor_pose

        # Motor pins
        self.motor_frequency = 50
        self.motor_dut_cycle = 45  # Controls speed
        self.turn_dc = 45  # Controls turn speed
        self.lb_motor_pin = 31
        self.lf_motor_pin = 33
        self.rb_motor_pin = 35
        self.rf_motor_pin = 37

        # Encoder pins
        self.left_encoder_pin = 7
        self.right_encoder_pin = 12

        # Gripper pins
        self.servo_frequency = 50
        self.open_servo_duty_cycle = 3
        self.close_servo_duty_cycle = 10.25
        self.servo_pin = 36

        # Distance sensor pins
        self.trig = 16
        self.echo = 18

        # PWM signals
        self.lpwmrev = 0
        self.rpwmrev = 0
        self.lpwm = 0
        self.rpwm = 0
        self.gpwm = 0

        self.InitGpio()

        # Identify serial connection on RPI for IMU
        # self.ser = serial.Serial('/dev/ttyUSB0', 9600)
        self.ser = ser
        self.monitor_imu = monitor_imu
        self.imu_angle = 0
        self.imu_margin = 3

        # Localization
        self.start_x = start_x  # Meters
        self.start_y = start_y  # Meters
        self.cur_x = self.start_x
        self.cur_y = self.start_y
        #self.goal_x = 2.7432  # Meters 9ft
        self.goal_x = 2.5
        self.goal_y = 0.45872  # Meters 1.5 ft
#         self.goal_x = 2.43
#         self.goal_y = 0.6096
        #self.goal_x = 1.50  # Meters 9ft
        #self.goal_y = 0.2  # Meters 1.5 ft
        self.pos_history = [(self.start_x, self.start_y)]

        # PID terms
        self.dt = 0.1  # Time step
        self.prev_err = 0
        self.integral = 0
        self.min_val = 0
        self.max_val = 2

        self.kp = 0.5
        self.ki = 0.0
        self.kd = 0.0

        # Robot properties
        self.gear_ratio = 120 / 1  # 1:120
        self.wheel_diameter = 0.065  # Meters
        self.wheel_base = 0.1397  # Meters (5.5")
        self.tics_per_rev = 20  # Number of encoder tics per 1 wheel revolution
        self.drive_constant = self.tics_per_rev \
            / (2 * np.pi * (self.wheel_diameter / 2))
        self.turning_perimeter = 2 * np.pi * (self.wheel_base)

    def InitGpio(self):
        """Assign RPI pins and initialize pwm signals
        """

        gpio.setmode(gpio.BOARD)

        # Left Motor pins
        gpio.setup(self.lb_motor_pin, gpio.OUT)  # IN1
        gpio.setup(self.lf_motor_pin, gpio.OUT)  # IN2

        # Control both left wheels
        # self.lpwm = gpio.PWM(self.lb_motor_pin, self.motor_frequency)

        # Right Motor pins
        gpio.setup(self.rb_motor_pin, gpio.OUT)  # IN3
        gpio.setup(self.rf_motor_pin, gpio.OUT)  # IN4

        # Control both right wheels
        # self.rpwm = gpio.PWM(self.rf_motor_pin, self.motor_frequency)

        # Encoder pins
        gpio.setup(self.left_encoder_pin, gpio.IN,
                   pull_up_down=gpio.PUD_UP)  # (Left) Setup pin 7 encoder
        gpio.setup(self.right_encoder_pin, gpio.IN,
                   pull_up_down=gpio.PUD_UP)  # (Right) Setup pin 12 encoder

        # Servo pins
        gpio.setup(self.servo_pin, gpio.OUT)
        self.gpwm = gpio.PWM(self.servo_pin, self.servo_frequency)
        self.gpwm.start(self.open_servo_duty_cycle)

        # Distance sensor pins
        gpio.setup(self.trig, gpio.OUT)
        gpio.setup(self.echo, gpio.IN)
        gpio.output(self.trig, False)

    def GameOver(self):
        """Set all pins to low
        """

        # Motor pins
        gpio.output(self.lb_motor_pin, False)
        gpio.output(self.lf_motor_pin, False)
        gpio.output(self.rb_motor_pin, False)
        gpio.output(self.rf_motor_pin, False)

        # Servo pins
        gpio.output(self.servo_pin, False)

        # Distance sensor pins
        gpio.output(self.trig, False)

        # self.lpwm.stop()
        # self.rpwm.stop()
        # self.gpwm.stop()

    def MonitorEncoders(self, action, stateBR, stateFL):
        # Save encoder states to txt file
        br = open(f'{action}_BRencoderstates.txt', 'a')
        fl = open(f'{action}_FLencoderstates.txt', 'a')
        # Save encoder states to txt files
        broutstring = str(stateBR) + '\n'
        floutstring = str(stateFL) + '\n'
        br.write(broutstring)
        fl.write(floutstring)

    def MonitorIMU(self, updated_angle):
        # Save encoder states to txt file
        file = open(f'imu_data.txt', 'a')
        # Save encoder states to txt files
        outstring = str(updated_angle) + '\n'
        file.write(outstring)

    def Distance(self):
        """Generate pulse signal to measure distance of objects in front of
        Baron Bot

        Returns:
            float: Distance of objects detected by distance sensor in [cm]
        """

        # Ensure output has no value
        gpio.output(self.trig, False)
        time.sleep(0.01)

        # Generate trigger pulse
        gpio.output(self.trig, True)
        time.sleep(0.00001)
        gpio.output(self.trig, False)

        # Generate echo time signal
        while gpio.input(self.echo) == 0:
            pulse_start = time.time()

        while gpio.input(self.echo) == 1:
            pulse_end = time.time()

        pulse_duration = pulse_end - pulse_start

        # Convert time to distance in [cm] using speed of sound
        distance = pulse_duration * 17150
        distance = round(distance, 2)

        print(f'Distance Sensor Reading: {distance}')
        return distance

    # def ReadIMU(self, count):
    #     """Read IMU data and return the value as a float

    #     Args:
    #         count (int): Number of times IMU reading has been read

    #     Returns:
    #         float: x-axis orientation value
    #           int: Number of times IMU reading has been read
    #     """

    #     count += 1

    #     # Read serial stream
    #     line = self.ser.readline()

    #     # Avoid first n-lines of serial info
    #     if count > 10:

    #         # Strip serial stream of extra characters
    #         line = line.rstrip().lstrip()

    #         line = str(line)
    #         line = line.strip("'")
    #         line = line.strip("b'")

    #         # Return float
    #         line = float(line)
    #         # try:
    #         #     line = float(line)
    #         #     return line, count
    #         # except ValueError:
    #         #     line = line.rstrip('\\x00')
    #         #     line = float(line)
    #         #     return line, count

    #     else:
    #         line = 0

    #     return line, count

    def ReadIMU(self):
        """Read IMU data and return the value as a float

        Args:
            count (int): Number of times IMU reading has been read

        Returns:
            float: x-axis orientation value
              int: Number of times IMU reading has been read
        """

        count = 0
        while True:
            if ser.in_waiting > 0:
                count += 1
                # Read serial stream
                line = ser.readline()

                # Avoid first n-lines of serial information
                if count > 0:
                    # Strip serial stream of extra characters
                    line = line.strip()
                    line = line.decode('utf-8')
                    line = line.strip("X: ")

                    try:
                        line = float(line)
                        return line, count


                    except ValueError:
                        pass

    def BufferIMU(self):
        """Buffer the IMU
        """

        count = 0

        while True:
            if ser.in_waiting > 0:
                count += 1
                # Read serial stream
                line = ser.readline()

                # Avoid first n-lines of serial information
                if count > 0:
                    # Strip serial stream of extra characters
                    line = line.strip()
                    line = line.decode('utf-8')
                    line = line.strip("X: ")

                    try:
                        line = float(line)
                        return line
                    except ValueError:
                        pass
                    break

    def ImgDistance(self, width):
        """Calculate the distance from the camera to a detected object using
        the width of its bounding box.

        Args:
            width (int): Width of the detected object's bounding box in pixels

        Returns:
            float: Distance from the camera to the detected object in meters
        """

        known_width = 0.0381  # Width of known bounding box and known_dist Meters
        box_width = width  # In pixels
        known_pix_width = 20  # pixels
        known_dist = 1  # Meter
        focal_length = (known_pix_width * known_dist) / known_width  # Meters

        distance = (known_width * focal_length) / box_width
        

        #distance = 2*((known_width * focal_length) / box_width)
        distance += 0.0762  # Meters
        print(f'Image Distance in meters: {distance}', distance)
        return distance

    def CloseGripper(self):
        """Fully close gripper
        """

        self.gpwm.ChangeDutyCycle(self.close_servo_duty_cycle)
        time.sleep(1.5)

    def OpenGripper(self):
        """Fully open gripper
        """

        self.gpwm.ChangeDutyCycle(self.open_servo_duty_cycle)
        time.sleep(1.5)

    def StopDriving(self):
        self.lpwm.ChangeDutyCycle(0)
        self.rpwm.ChangeDutyCycle(0)
        self.lpwmrev.ChangeDutyCycle(0)
        self.rpwmrev.ChangeDutyCycle(0)

        gpio.output(self.lb_motor_pin, False)
        gpio.output(self.lf_motor_pin, False)

        gpio.output(self.rb_motor_pin, False)
        gpio.output(self.rf_motor_pin, False)

    def Forward(self, distance):
        """Move robot forward for 'distance' meters

        Args:
            distance (int): Distance in meters for robot to drive forward
        """

        # Total encoder tics to drive the desired distance
        encoder_tics = int(self.drive_constant * distance)
        proportional_gain = 21

        # straight_contition = False

        # # Get Initial IMU angle reading
        # init_angle = self.imu_angle

        # if 360 - self.imu_margin < init_angle or \
        #         init_angle < self.imu_margin:
        #     straight_contition = True
        #     # init_angle -= 360

        # if init_angle > 360 - self.imu_margin:
        #     init_angle -= 360

        # Left wheel
        gpio.output(self.lb_motor_pin, True)
        gpio.output(self.lf_motor_pin, False)
        #self.lpwm.start(self.motor_dut_cycle)
        

        # Right wheel
        gpio.output(self.rb_motor_pin, False)
        gpio.output(self.rf_motor_pin, True)
        #self.rpwm.start(self.motor_dut_cycle)
        self.lpwm.ChangeDutyCycle(self.motor_dut_cycle)
        self.rpwm.ChangeDutyCycle(self.motor_dut_cycle)

        counterBR = np.uint64(0)
        counterFL = np.uint64(0)

        buttonBR = int(0)
        buttonFL = int(0)

        print(f'Duty Cycle: {self.motor_dut_cycle}')

        # count = 0
        while True:

            # distance = self.Distance()

            # if distance < 10.16:  # cm
            #     self.Reverse(0.25)
            #     self.OpenGripper()
            #     break

            # if self.ser.in_waiting > 0:
            #     updated_angle, count = self.ReadIMU(count)
            #     self.rpwm.ChangeDutyCycle(self.motor_dut_cycle)
            #     self.lpwm.ChangeDutyCycle(self.motor_dut_cycle)

            #     if straight_contition and updated_angle > 180:
            #         updated_angle -= 360

            stateBR = gpio.input(self.right_encoder_pin)
            stateFL = gpio.input(self.left_encoder_pin)

            # Save encoder states to txt files
            if self.monitor_encoders is True:
               self.MonitorEncoders('Forward', stateBR, stateFL)

            # if self.monitor_imu is True:
            #     self.MonitorIMU(updated_angle)

            if int(stateBR) != int(buttonBR):
                buttonBR = int(stateBR)
                counterBR += 1

            if int(stateFL) != int(buttonFL):
                buttonFL = int(stateFL)
                counterFL += 1


            # # PID tunning
            # low_thresh = init_angle - self.imu_margin
            # high_thresh = init_angle + self.imu_margin

            # if not straight_contition:
            #     if low_thresh < 0:
            #         low_thresh += 360

            #     if high_thresh > 359:
            #         high_thresh -= 360
            # # print(f'Goal: {encoder_tics} R: {counterBR} L: {counterFL}\
            # #  Angle: {updated_angle} Dutycycle: {self.motor_dut_cycle}')

            # if self.debug_mode:
            #     print(f'Angle: {updated_angle} Initial: {init_angle}\
            #         Dutycycle: {self.motor_dut_cycle}')

            # if updated_angle < low_thresh:

            #     # Double speed to match encoder counts
            #     speed_update = min(self.motor_dut_cycle * 2, 100)
            #     self.lpwm.ChangeDutyCycle(speed_update)

            # if updated_angle > high_thresh:

            #     # Double speed to match encoder counts
            #     speed_update = min(self.motor_dut_cycle * 2, 100)
            #     self.rpwm.ChangeDutyCycle(speed_update)

            # if low_thresh < updated_angle < high_thresh:

            #     self.rpwm.ChangeDutyCycle(self.motor_dut_cycle)
            #     self.lpwm.ChangeDutyCycle(self.motor_dut_cycle)
            #print('counterBR: ', counterBR,  'counterFL: ', counterFL)
            error = counterBR - counterFL       


            speed_update_l = self.motor_dut_cycle + error * proportional_gain
        

            speed_update_r = self.motor_dut_cycle + error * proportional_gain
            # PID tunning
            
                # Double speed to match encoder counts
            speed_update_l = max(10, min(100, speed_update_l))
            

            
                # Double speed to match encoder counts
            speed_update_r = max(10, min(100, speed_update_r))
            try:
                
                  self.lpwm.ChangeDutyCycle(speed_update_l)
                  self.rpwm.ChangeDutyCycle(speed_update_r)
            except:
                pass

            if self.debug_mode:
                print(f'Goal: {encoder_tics} R: {counterBR} L: {counterFL}')

            # Break when both encoder counts reached the desired total
            if counterBR >= encoder_tics and counterFL >= encoder_tics:
                self.StopDriving()
                self.CollectPos(distance)
                # updated_angle, count = self.ReadIMU(count)
                # if straight_contition:
                #     self.imu_angle = updated_angle + 360
                # else:
                #     self.imu_angle = updated_angle
                break

    def Reverse(self, distance):
        """Move robot in reverse for 'distance' meters

        Args:
            distance (int): Distance in meters for robot to drive in reverse
        """

        # Total encoder tics to drive the desired distance
        encoder_tics = int(self.drive_constant * distance)

        # # Get Initial IMU angle reading
        # init_angle = 0

        # gpio.output(self.lb_motor_pin, True)
        # gpio.output(self.lf_motor_pin, False)
        # # self.lpwm.start(self.motor_dut_cycle)

        # # Right wheel
        # gpio.output(self.rb_motor_pin, False)
        # gpio.output(self.rf_motor_pin, True)

        # Left wheel
        # gpio.output(self.lb_motor_pin, False)
        # gpio.output(self.lf_motor_pin, True)
        # self.lpwm.ChangeDutyCycle(self.motor_dut_cycle)
        self.lpwmrev.ChangeDutyCycle(self.motor_dut_cycle)

        # Right wheel
        # gpio.output(self.rb_motor_pin, True)
        # gpio.output(self.rf_motor_pin, False)
        # self.rpwm.ChangeDutyCycle(self.motor_dut_cycle)
        self.rpwmrev.ChangeDutyCycle(self.motor_dut_cycle)

        counterBR = np.uint64(0)
        counterFL = np.uint64(0)

        buttonBR = int(0)
        buttonFL = int(0)

        print(f'Duty Cycle: {self.motor_dut_cycle}')

        count = 0
        while True:

            # if self.ser.in_waiting > 0:
            #     updated_angle, count = self.ReadIMU(count)

            # if 300 < updated_angle < 360:
            #     updated_angle -= 360

            stateBR = gpio.input(self.right_encoder_pin)
            stateFL = gpio.input(self.left_encoder_pin)

            # Save encoder states to txt files
            if self.monitor_encoders is True:
                self.MonitorEncoders('Reverse', stateBR, stateFL)

            # if self.monitor_imu is True:
            #     self.MonitorIMU(updated_angle)

            if int(stateBR) != int(buttonBR):
                buttonBR = int(stateBR)
                counterBR += 1

            if int(stateFL) != int(buttonFL):
                buttonFL = int(stateFL)
                counterFL += 1

            if counterBR >= encoder_tics:
                # self.rpwm.stop()
                self.rpwmrev.ChangeDutyCycle(0)

            if counterFL >= encoder_tics:
                # self.lpwm.stop()
                self.lpwmrev.ChangeDutyCycle(0)

            # PID tunning
            # imu_margin = 0.5
            # low_thresh = init_angle - imu_margin
            # high_thresh = init_angle + imu_margin

            # if self.debug_mode:
            #     print(f'Goal: {encoder_tics} R: {counterBR} L: {counterFL}\
            #        Angle: {updated_angle} Dutycycle: {self.motor_dut_cycle}')

            # if updated_angle < low_thresh:

            #     # Double speed to match encoder counts
            #     speed_update = min(self.motor_dut_cycle * 2, 100)
            #     self.rpwm.ChangeDutyCycle(speed_update)

            # if updated_angle > high_thresh:

            #     # Double speed to match encoder counts
            #     speed_update = min(self.motor_dut_cycle * 2, 100)
            #     self.lpwm.ChangeDutyCycle(speed_update)

            # if low_thresh < updated_angle < high_thresh:

            #     self.rpwm.ChangeDutyCycle(self.motor_dut_cycle)
            #     self.lpwm.ChangeDutyCycle(self.motor_dut_cycle)

            # PID tunning
            if counterBR > counterFL:
                # Double speed to match encoder counts
                speed_update = min(self.motor_dut_cycle * 2, 100)
                self.lpwmrev.ChangeDutyCycle(speed_update)

            if counterFL > counterBR:
                # Double speed to match encoder counts
                speed_update = min(self.motor_dut_cycle * 2, 100)
                self.rpwmrev.ChangeDutyCycle(speed_update)

            if counterBR == counterFL:
                self.lpwmrev.ChangeDutyCycle(self.motor_dut_cycle)
                self.rpwmrev.ChangeDutyCycle(self.motor_dut_cycle)

            if self.debug_mode:
                print(f'Goal: {encoder_tics} R: {counterBR} L: {counterFL}')

            # Break when both encoder counts reached the desired total
            if counterBR >= encoder_tics and counterFL >= encoder_tics:
                # self.rpwm.stop()
                # self.lpwm.stop()
                self.StopDriving()
                print('Stopped Driving!')
                self.CollectPos(-distance)
                print('Pose Collected!')
                break

    def LeftPiv(self, angle):
        """Pivot robot to the left for 'tf' seconds

        Args:
            angle (int): Angle in degrees for robot to turn left
        """

        # Get Initial IMU angle reading
        init_angle = self.imu_angle

        if init_angle < 0:
            init_angle += 360

        goal_angle = init_angle - angle

        flip = False
        cross = False
        if goal_angle > init_angle:
            cross = True

        if goal_angle < 0:
            goal_angle += 360

        if init_angle - self.imu_margin < goal_angle < init_angle + self.imu_margin:
            pass
        elif (init_angle <= 2 or init_angle >= 358) and \
                (goal_angle <= 3 or goal_angle >= 357):
            pass
        else:
            # # Left wheel
            # gpio.output(self.lb_motor_pin, False)
            # gpio.output(self.lf_motor_pin, True)
            # self.lpwm.start(0)

            # # Right wheel
            # gpio.output(self.rb_motor_pin, False)
            # gpio.output(self.rf_motor_pin, True)
            # self.rpwm.start(0)

            counterBR = np.uint64(0)
            counterFL = np.uint64(0)

            buttonBR = int(0)
            buttonFL = int(0)

            count = 0
            while True:

                updated_angle, count = self.ReadIMU()

                # Left wheel
                # gpio.output(self.lb_motor_pin, False)
                # gpio.output(self.lf_motor_pin, True)
                # # self.lpwm.start(0)

                # # Right wheel
                # gpio.output(self.rb_motor_pin, False)
                # gpio.output(self.rf_motor_pin, True)
                # self.rpwm.start(0)

                self.rpwm.ChangeDutyCycle(self.turn_dc)
                self.lpwmrev.ChangeDutyCycle(self.turn_dc)

                stateBR = gpio.input(self.right_encoder_pin)
                stateFL = gpio.input(self.left_encoder_pin)

                # Save encoder states to txt files
                if self.monitor_encoders is True:
                    self.MonitorEncoders('LeftPiv', stateBR, stateFL)

                if self.monitor_imu is True:
                    self.MonitorIMU(updated_angle)

                if int(stateBR) != int(buttonBR):
                    buttonBR = int(stateBR)
                    counterBR += 1

                if int(stateFL) != int(buttonFL):
                    buttonFL = int(stateFL)
                    counterFL += 1

                if self.debug_mode:
                    print(f'Goal: {goal_angle} Angle: {updated_angle}\
                    Initial: {init_angle} Dutycycle: {self.turn_dc}')

                # PID tunning
                low_thresh = goal_angle - self.imu_margin
                high_thresh = goal_angle + self.imu_margin

                # Break when the current angle is within a threshold of the
                # goal angle
                if low_thresh < updated_angle < high_thresh:

                    # # Left wheel
                    # gpio.output(self.lb_motor_pin, False)
                    # gpio.output(self.lf_motor_pin, False)
                    # self.lpwm.stop()

                    # # Right wheel
                    # gpio.output(self.rb_motor_pin, False)
                    # gpio.output(self.rf_motor_pin, False)
                    # self.rpwm.stop()
                    self.StopDriving()

                    self.imu_angle = updated_angle
                    break

    def RightPiv(self, angle):
        """Pivot robot to the right for 'tf' seconds

        Args:
            angle (int): Angle in degrees for robot to turn right
        """

        # Get Initial IMU angle reading
        init_angle = self.imu_angle

        if init_angle < 0:
            init_angle += 360

        goal_angle = init_angle + angle

        flip = False
        cross = False
        if goal_angle < init_angle:
            cross = True

        if goal_angle > 360:
            goal_angle -= 360

        if init_angle - self.imu_margin < goal_angle < init_angle + self.imu_margin:
            pass
        elif (init_angle <= 2 or init_angle >= 358) and \
                (goal_angle <= 3 or goal_angle >= 357):
            pass
        else:

            # # Left wheel
            # gpio.output(self.lb_motor_pin, True)
            # gpio.output(self.lf_motor_pin, False)
            # self.lpwm.start(0)

            # # # Right wheel
            # gpio.output(self.rb_motor_pin, True)
            # gpio.output(self.rf_motor_pin, False)
            # self.rpwm.start(0)

            counterBR = np.uint64(0)
            counterFL = np.uint64(0)

            buttonBR = int(0)
            buttonFL = int(0)

            count = 0
            while True:

                updated_angle, count = self.ReadIMU()

                # # Left wheel
                # gpio.output(self.lb_motor_pin, True)
                # gpio.output(self.lf_motor_pin, False)
                # # self.lpwm.start(0)

                # # # Right wheel
                # gpio.output(self.rb_motor_pin, True)
                # gpio.output(self.rf_motor_pin, False)
                # self.rpwm.start(0)

                self.rpwmrev.ChangeDutyCycle(self.turn_dc)
                self.lpwm.ChangeDutyCycle(self.turn_dc)

                stateBR = gpio.input(self.right_encoder_pin)
                stateFL = gpio.input(self.left_encoder_pin)

                # Save encoder states to txt files
                if self.monitor_encoders is True:
                    self.MonitorEncoders('RightPiv', stateBR, stateFL)

                if self.monitor_imu is True:
                    self.MonitorIMU(updated_angle)

                if int(stateBR) != int(buttonBR):
                    buttonBR = int(stateBR)
                    counterBR += 1

                if int(stateFL) != int(buttonFL):
                    buttonFL = int(stateFL)
                    counterFL += 1

                # PID tunning
                if self.debug_mode:
                    print(f'Goal: {goal_angle} Angle: {updated_angle}\
                    Initial: {init_angle} Dutycycle: {self.motor_dut_cycle}')

                low_thresh = goal_angle - self.imu_margin
                high_thresh = goal_angle + self.imu_margin

                # Break when the current angle is within a threshold of the
                # goal angle
                if low_thresh < updated_angle < high_thresh:

                    # # Left wheel
                    # gpio.output(self.lb_motor_pin, False)
                    # gpio.output(self.lf_motor_pin, False)
                    # self.lpwm.stop()

                    # # Right wheel
                    # gpio.output(self.rb_motor_pin, False)
                    # gpio.output(self.rf_motor_pin, False)
                    # self.rpwm.stop()
                    self.StopDriving()

                    self.imu_angle = updated_angle
                    break

    def PID(self, target, present):
        """A function which computes the PID controller output value. 'target'
        is used to store the setpoint and 'present' is used to store the
        current value
        Steps to calculate output :
        1) err is the difference between the target and the present value
        2) The proportional term is Kp times the error
        3) The error is multiplied with the time step dt and added to the
        integral variable
        4) The integral term is Ki times the integral variable
        5) The derivate term is Kd times the difference in present error and
        previous error divided by the time step
        6) Total output is the bounded (withing min and max) sum of the
        proportional, integral, and derivate term

        Args:
            target (float): Desired final velocity
            present (float): Current velocity

        Returns:
            float: Final value calculated by PID controller
        """

        err = target - present

        p_term = self.kp * err

        self.integral = self.integral + (err * self.dt)

        i_term = self.ki * self.integral

        d_term = self.kd * (err - self.prev_err) / self.dt

        output = p_term + i_term + d_term

        self.prev_err = err

        output = max(self.min_val, min(self.max_val, output))

        return output

    def ColorRange(self, color='green'):

        if color == 'red':
            # low_red = np.array([0, 80, 88])
            # high_red = np.array([42, 255, 255])

            # 6:40pm
            low_red = np.array([160,110,60])
            high_red = np.array([255,255,255])

            return low_red, high_red

        elif color == 'green':
            # low_green = np.array([60, 93, 70])
            # high_green = np.array([91, 255, 168])

            # 6:40pm
            low_green = np.array([35, 45, 20])
            high_green = np.array([95, 255, 255])
            return low_green, high_green

        elif color == 'blue':
            # low_blue = np.array([100, 79, 68])
            # high_blue = np.array([157, 255, 168])

            # 6:40pm
            low_blue = np.array([90, 90, 50])  # Lower hue, saturation, and value thresholds for blue
            high_blue  = np.array([155, 255, 245])

            return low_blue, high_blue

    def DistFromCenter(self, x_pos):
        """Determine angle to turn to align with center of the detected object.

        Args:
            x_pos (int): pixel x position in the frame corresponding to the
            center of the detected object

        Returns:
            str: Direction to turn
            float: Angle to turn in degrees
        """

        midline = 640 / 2

        pix_per_deg = 38.88 / 640

        pos = x_pos - midline

        if pos > 0:
            angle = pix_per_deg * pos
            # self.RightPiv(angle)
            direction = 'Right'
        else:
            angle = pix_per_deg * abs(pos)
            # self.LeftPiv(angle)
            direction = 'Left'

        if self.debug_mode:
            print(f'Angle to Turn {direction}: {angle}')

        return direction, angle

    def FindBlock(self, color='green'):
        print(f'Finding Block {color}')

        x_center = 0
        distance = 0
        w = 0
        image = self.capture()
        image = cv2.flip(image,-1)
        #Converting to HSV
        hsv_image = cv2.cvtColor(image,cv2.COLOR_BGR2HSV)

#         cap = cv2.VideoCapture(0)
        # print('Cap')
#         ret, frame = cap.read()
        # print('Captured frame')

#         if ret:

#             frame = cv2.flip(frame, 0)
#             frame = cv2.flip(frame, 1)
#             hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        low_thresh, high_thresh = self.ColorRange(color)
        mask = cv2.inRange(hsv_image, low_thresh, high_thresh)

            # Perform Closing (morphology) to filter noise
        kernel = np.ones((15, 15), np.uint8)
        mask = cv2.dilate(mask, kernel)
        mask = cv2.erode(mask, kernel)

        # Find the contours
        cnts = cv2.findContours(mask,
                                cv2.RETR_TREE,
                                cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)

        if len(cnts) > 0:
            # find the biggest countour (c) by the area
            c = max(cnts, key=cv2.contourArea)

            rect = cv2.boundingRect(c)
            

            x, y, w, h = rect
            #print('w', w)
            

            if w > 5:
                print(f'Found {color} block')
                x_center = int(x + w / 2)
                y_center = int(y + h / 2)

                center = (x_center, y_center)
                print('x,y', center)

                cv2.rectangle(image, (x, y), (x + w, y + h),
                              color=(0, 255, 255), thickness=2)

                # self.DistFromCenter(x_center)
                distance = self.ImgDistance(w)

                # self.Forward(distance)
                # self.CloseGripper()

                cv2.circle(image,
                           center,
                           1,
                           color=(0, 0, 255),
                           thickness=4)
            else:
                x_center = 0
                distance = 0
                w = 0

        if w == 0:
            # Close the window / Release webcam
#             cap.release()

            # De-allocate any associated memory usage
            cv2.destroyAllWindows()

            print(f'Scanning for {color} block')
            time.sleep(4)
            self.LeftPiv(45)
            print('Turned 45')
            self.FindBlock(color)

        if self.debug_mode:
            print(f'Bounding Box Width: {w}')
            print(f'Distance: {distance}')
            cv2.imshow('frame', frame)
            cv2.waitKey(0)
        # Close the window / Release webcam
#         cap.release()

        # De-allocate any associated memory usage
        cv2.destroyAllWindows()
        return x_center, distance, w
        
    def capture(self):
        cam= cv2.VideoCapture(0)
        fps = cam.get(cv2.CAP_PROP_FPS)
        timestamps = [cam.get(cv2.CAP_PROP_POS_MSEC)]
    #    calc_timestamps = [0.0]
    #    frame_width = int(cam.get(3))
    #    frame_height = int(cam.get(4))
    #    size = (frame_width, frame_height)
        ret, image = cam.read()
        cam.release()
        return image  
    def SendEmail(self):
        """Take a picture using the RaspberryPi camera and send it in an email.
        """

        # Define time stamp and record an image
        pic_time = datetime.now().strftime('%Y%m%d%H%M%S')
        cmd = f'raspistill -w 640 -h 480 -vf -hf -o {pic_time}.jpg'
        os.system(cmd)

        # Email information
        smtpUser = 'banagirisaketh@gmail.com'
        smtpPass = 'cjsxldphfgeklrfm'

        # Destination email information
        # toAdd = 'scortes3@umd.edu'
        toAdd = ['sakethnarayan@gmail.com']#['ENPM809TS19@gmail.com']#,
                #  'ENPM809TS19@gmail.com',
                #  'skotasai@umd.edu']
        fromAdd = smtpUser
        subject = f'Image recorded at {pic_time}'
        msg = MIMEMultipart()
        msg['Subject'] = subject
        msg['From'] = fromAdd
        # msg['To'] = toAdd
        msg['To'] = ', '.join(toAdd)
        msg.preamble = f'Image recorded at {pic_time}'

        # Email text
        body = MIMEText(f'Image recorded at {pic_time}')
        msg.attach(body)

        # Attach image
        fp = open(f'{pic_time}.jpg', 'rb')
        img = MIMEImage(fp.read())
        msg.attach(img)

        # Send email
        s = smtplib.SMTP('smtp.gmail.com', 587)

        s.ehlo()
        s.starttls()
        s.ehlo()

        s.login(smtpUser, smtpPass)
        s.sendmail(fromAdd, toAdd, msg.as_string())
        s.quit()

        print("Email delivered!")

    def checkEmail(self):
        """Wait for email to be received then return true.
        Checks every 2 seconds, after 120 seconds return False.

        Returns:
            Bool: True if an unread email is found in less than 120 seconds
        """

        mail = imaplib.IMAP4_SSL('imap.gmail.com')
        emailadds = 'banagirisaketh@gmail.com'
        pwd = 'cjsxldphfgeklrfm'
        mail.login(emailadds, pwd)
        mail.list()  # List of folders or lables in gmail

        count = 0

        # while count < 60:
        #     try:
        #         # Connect to inbox
        #         mail.select('inbox')

        #         # Search for an unread email form user's email address
        #         result, data = mail.search(None,
        #                                    f'(UNSEEN FROM "balase22@umd.edu")')
        #         # print(result)
        #         # print(len(data))
        #         # print(f'Data: {data}')

        #         ids = data[0]  # Data is a list
        #         id_list = ids.split()  # ids is a space separated by a string

        #         latest_email_id = id_list[-1]  # Get latest
        #         result, data = mail.fetch(latest_email_id, 'RFC822')

        #         if data is None:
        #             print('Waiting...')

        #         if data is not None:
        #             print('Process Initiated')
        #             return True

        #     except IndexError:
        #         print('err')
        #         time.sleep(2)
        #         if count < 59:
        #             count = count + 1
        #             continue
        #         else:
        #             print("Gameover")
        #             count = 60
        #             return False

    def KeyInput(self, key, value):
        """Operate robot through user input to drive and open/close gripper

        Args:
            event (str): 'w', 's', 'a', 'd', 'o', 'c', 'p' to choose an action
            for the robot
            value (float): Distance to travel/Angle to turn
        """

        print(f'Key: {key}')
        print(f'Distance/Angle: {value}')

        key_press = key

        # Measure distance of objects before following a command
        self.Distance()

        if key_press.lower() == 'w':
            self.Forward(value)
        elif key_press == 's':
            self.Reverse(value)
        elif key_press == 'a':
            self.LeftPiv(value)
        elif key_press == 'd':
            self.RightPiv(value)
        elif key_press == 'o':
            self.OpenGripper()
        elif key_press == 'c':
            self.CloseGripper()
        else:
            print('Invlaid key pressed!!')

    def Teleop(self):
        """Control loop to teleop the robot
        """

        while True:

            key_press = input("Select Action: 'w' - Forward \n \
                's' - Backward \n \
                'a' - Pivot Left \n \
                'd' - Pivot Right \n \
                'o' - Open Gripper \n \
                'c' - Close Gripper \n \
                'p' - Exit Program \n")

            if key_press.lower() == 'p':
                break

            value = input("Distance to travel/Angle to turn: ")

            self.KeyInput(key_press, float(value))

            self.GameOver()

    def Navigate(self):

        # Collect navigation commands
        cmds = []
        while True:
            cmd = input('Type command in the format \
                (direction, distance/angle) "q" to finish: ').split(', ')

            if cmd[0] == 'q' or cmd[1] == 'q':
                break

            cmd[1] = float(cmd[1])
            cmds.append(cmd)

        # Unpack commands and control bot
        for key, val in cmds:
            self.KeyInput(key, val)
            time.sleep(1.5)

    def GLocalize(self):
        """Calculate the distance to drive and angle to turn to get from the
        current position to the goal zone.

        Returns:
            float: Distance to the goal zone
            float: Angle to turn to face goal zone
        """

        v_x = self.goal_x - self.cur_x
        v_y = self.goal_y - self.cur_y
        v_mag = np.sqrt((v_x**2) + (v_y**2))
        print('distance left to goal: ', v_mag)

        # x_prev, y_prev = self.pos_history[-2]

        # u_x = self.cur_x - x_prev
        # u_y = self.cur_y - y_prev

        # u_mag = np.sqrt((u_x**2) + (u_y**2))

        # dot = (u_x * v_x) + (u_y * v_y)
        # coeff = dot / (u_mag * v_mag)
        # angle = np.arccos(coeff)
        # angle = np.rad2deg(angle)

        angle = np.arccos(v_x / v_mag)
        angle = np.rad2deg(angle)

        angle += self.imu_angle

        if angle > 359:
            angle -= 360

        # v_mag -= 0.1  # Subtract from distance to drive

        if self.debug_mode:
            print(f'Pos History: {self.pos_history}')
            print(f'Move to Goal- Turn: {angle} Drive: {v_mag}')
        # time.sleep(10)

        return v_mag, angle

    def CollectPos(self, distance):
        """Determine current position w.r.t. world frame

        Args:
            distance (float): distance traveled forward
        """

        x_prev, y_prev = self.pos_history[-1]
        angle = np.deg2rad(self.imu_angle)
        x_new = x_prev + (distance * np.cos(angle))
        y_new = y_prev + (distance * np.sin(angle))
        print(",x_prev, x_new: ",x_prev, x_new, "y_prev,y_new: ",y_prev, y_new)
        self.pos_history.append((x_new, y_new))
        self.cur_x = x_new
        self.cur_y = y_new
     
        if self.monitor_pose:
            print(f'New Pose (x: {x_new}, y: {y_new})')
            # Save Pose data to txt file
            file = open(f'pos_data.txt', 'a')
            xfile = open(f'xpos_data.txt', 'a')
            yfile = open(f'ypos_data.txt', 'a')
            # Save encoder states to txt files
            outstring = 'Drive ' + str(distance) + ' pose: ' + str(x_new) + ', ' + str(y_new) + ' orientation: ' + str(np.rad2deg(angle)) + '\n'
            print('outstring: ', outstring)
            xoutstring = str(x_new) + '\n'
            youtstring = str(y_new) + '\n'
            file.write(outstring)
            xfile.write(xoutstring)
            yfile.write(youtstring)


def GrandChallenge(robot, color, idx):

    midline = 640 / 2

    while True:
        x_center, distance, box_width = robot.FindBlock(color[idx])
        #print('x-center,distance:',x_center, distance)

        delta = midline - x_center
        #print(f'Delta = {midline} - {x_center} = {delta}', delta)
        robot.CloseGripper()
        

        if abs(delta) > 90:  # Obj center is greater than 90 pixels
            direction, angle = robot.DistFromCenter(x_center)
            #print(f'Angle: {angle}')
            if direction == 'Right':
                robot.RightPiv(angle)
            else:
                robot.LeftPiv(angle)
                

        elif distance > 0.3048:
            #0.3048
            #0.2032 8inches
            robot.Forward(distance/2)          
            
            continue
        elif distance < 0.7:
            robot.OpenGripper()
            robot.Forward(distance/2)
           
         
        
    

        x_center, distance, box_width = robot.FindBlock(color[idx])
        #print('x_center, distance, box_width', x_center, distance, box_width )
        #print('bw',box_width)
        if box_width > 320:
            print('Block gripped!')
            robot.CloseGripper()
            robot.SendEmail()
            break

    goal_dist, goal_ang = robot.GLocalize()
    time.sleep(1)
    robot.LeftPiv(goal_ang)
    robot.Forward(goal_dist)


if __name__ == '__main__':

    ser = serial.Serial('/dev/ttyUSB0', 9600)
    # Flush initial readings
    # time.sleep(5)
    ser.reset_input_buffer()

    robot = Robot(monitor_encoders=False,
                  monitor_imu=False,
                  debug_mode=False,
                  monitor_pose=True,
                  ser=ser)

    lpwm = gpio.PWM(robot.lb_motor_pin,
                    robot.motor_frequency)

    rpwm = gpio.PWM(robot.rf_motor_pin,
                    robot.motor_frequency)

    lpwm.start(0)
    rpwm.start(0)

    lpwmrev = gpio.PWM(robot.lf_motor_pin,
                       robot.motor_frequency)

    rpwmrev = gpio.PWM(robot.rb_motor_pin,
                       robot.motor_frequency)

    lpwmrev.start(0)
    rpwmrev.start(0)

    robot.rpwm = rpwm
    robot.lpwm = lpwm
    robot.rpwmrev = rpwmrev
    robot.lpwmrev = lpwmrev

    robot.BufferIMU()

    repeat = 0
    color = ['red', 'green', 'blue']
    idx = 0

    # time.sleep(4)

    robot.RightPiv(45)

    while repeat < 3:
        GrandChallenge(robot, color, idx)
        robot.OpenGripper()
        robot.Reverse(0.25)
        robot.LeftPiv(90)

       # robot.goal_x += 0.1524 #6INCHES
       # robot.goal_x += 0.1
        

        idx += 1
        if idx == 3:
            repeat += 1
            idx = 0

            #robot.goal_x += 0.6096  # 24 inches
            robot.goal_y += 0.05  # 3 inches

    # robot.Teleop()
    robot.goal_x = 0
    robot.goal_y = 0
    robot.GameOver()
    gpio.cleanup()

    robot.lpwm.stop()
    robot.rpwm.stop()
    robot.gpwm.stop()
    print('Grand Challenge Complete! Congrats on finishing your Robotics Masters Degree!')
    
    
    
    
    
    