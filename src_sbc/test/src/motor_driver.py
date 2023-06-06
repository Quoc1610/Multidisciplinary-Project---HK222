#!/usr/bin/env python
import rospy
import time
import sys
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray
                      
# m1: Right motors
# m2: Left motors
# Max linear speed: 2.13m / 3s -> 0.71 m/s
# R = 0.033; L = 0.147

class MotorDriver(object):

    def __init__(self, wheel_distance=0.147, wheel_diameter=0.066, i_BASE_PWM=50, i_MULTIPLIER_STANDARD=0.1, i_MULTIPLIER_PIVOT=1.0, simple_mode = True):
        """
        M1 = Right Wheel
        M2 = Left Wheel
        :param wheel_distance: Distance Between wheels in meters
        :param wheel_diameter: Diameter of the wheels in meters
        """
        self.BASE_PWM = i_BASE_PWM
        self.MAX_PWM = 100
        
        self.simple_mode = simple_mode

        # Wheel and chasis dimensions
        self._wheel_distance = wheel_distance
        self._wheel_radius = wheel_diameter / 2.0
        self.MULTIPLIER_STANDARD = i_MULTIPLIER_STANDARD
        self.MULTIPLIER_PIVOT = i_MULTIPLIER_PIVOT
        self.command_msg = Int32MultiArray()
        self.command_pub = rospy.Publisher('command', Int32MultiArray, queue_size=10)
        self.r = rospy.Rate(10) # 10hz

    def set_motor(self, A2, A1, B2, B1):
        A1 *= 65535
        A2 *= 65535
        B1 *= 65535
        B2 *= 65535
        self.command_msg = Int32MultiArray(data=[
        int(self.PWM1 * 655.35), A1, A2, 
        B1, B2, int(self.PWM2 * 655.35), 
        -1, -1, -1, -1, -1, -1, -1, -1, -1, -1
        ])

    def forward(self):
        self.set_motor(0, 1, 0, 1)

    def stop(self):
        self.set_motor(0, 0, 0, 0)

    def reverse(self):
        self.set_motor(1, 0, 1, 0)

    # Left wheel forward, right wheel stop => turn right
    # Brake mode test
    def left(self):
        self.set_motor(0, 1, 0, 0)

    def left_reverse(self):
        self.set_motor(1, 0, 0, 0)

    def pivot_left(self):
        self.set_motor(1, 0, 0, 1)

    # Brake mode test
    def right(self):
        self.set_motor(0, 0, 0, 1)

    def right_reverse(self):
        self.set_motor(0, 0, 1, 0)

    def pivot_right(self):
        self.set_motor(0, 1, 1, 0)

    def set_M1M2_speed(self, rpm_speedM1, rpm_speedM2, multiplier):

        self.set_M1_speed(rpm_speedM1, multiplier)
        self.set_M2_speed(rpm_speedM2, multiplier)

    def set_M1_speed(self, rpm_speed, multiplier):

        self.PWM1 = min(int((rpm_speed * multiplier) * self.BASE_PWM), self.MAX_PWM)
        # temp = Int32MultiArray(data=[
        # self.PWM1 * 655.35 , self.command_msg[1], self.command_msg[2], 
        # self.command_msg[3], self.command_msg[4], self.command_msg[5], 
        # -1, -1, -1, -1, -1, -1, -1, -1, -1, -1
        # ])
        # self.command_msg[0] = self.PWM1 * 655.35
        print("M1="+str(self.PWM1))

    def set_M2_speed(self, rpm_speed, multiplier):

        self.PWM2 = min(int(rpm_speed * multiplier * self.BASE_PWM), self.MAX_PWM)
        # temp = Int32MultiArray(data=[
        # self.command_msg[0], self.command_msg[1], self.command_msg[2], 
        # self.command_msg[3], self.command_msg[4], self.PWM2 * 655.35, 
        # -1, -1, -1, -1, -1, -1, -1, -1, -1, -1
        # ])
        # self.command_msg = temp
        # self.command_msg[5] = self.PWM2 * 655.35
        print("M2="+str(self.PWM2))

    def calculate_body_turn_radius(self, linear_speed, angular_speed):
        # Car body turn radius to 
        if angular_speed != 0.0:
            body_turn_radius = linear_speed / angular_speed
        else:
            # Not turning, infinite turn radius
            body_turn_radius = None
        return body_turn_radius

    def calculate_wheel_turn_radius(self, body_turn_radius, angular_speed, wheel):

        if body_turn_radius is not None:
            """
            if angular_speed > 0.0:
                angular_speed_sign = 1
            elif angular_speed < 0.0:
                angular_speed_sign = -1
            else:
                angular_speed_sign = 0.0
            """
            if wheel == "right":
                wheel_sign = 1
            elif wheel == "left":
                wheel_sign = -1
            else:
                assert False, "Wheel Name not supported, left or right only."

            wheel_turn_radius = body_turn_radius + ( wheel_sign * (self._wheel_distance / 2.0))
        else:
            wheel_turn_radius = None

        return wheel_turn_radius

    def calculate_wheel_rpm(self, linear_speed, angular_speed, wheel_turn_radius):
        """
        Omega_wheel = Linear_Speed_Wheel / Wheel_Radius
        Linear_Speed_Wheel = Omega_Turn_Body * Radius_Turn_Wheel
        --> If there is NO Omega_Turn_Body, Linear_Speed_Wheel = Linear_Speed_Body
        :param angular_speed:
        :param wheel_turn_radius:
        :return:
        """
        if wheel_turn_radius is not None:
            # The robot is turning
            wheel_rpm = (angular_speed * wheel_turn_radius) / self._wheel_radius
        else:
            # Its not turning therefore the wheel speed is the same as the body
            wheel_rpm = linear_speed / self._wheel_radius

        return wheel_rpm

    def set_wheel_movement(self, right_wheel_rpm, left_wheel_rpm):

        #print("W1,W2=["+str(right_wheel_rpm)+","+str(left_wheel_rpm)+"]")

        if right_wheel_rpm > 0.0 and left_wheel_rpm > 0.0:
            #print("All forwards")
            self.set_M1M2_speed(abs(right_wheel_rpm), abs(left_wheel_rpm), self.MULTIPLIER_STANDARD)

            if self.simple_mode:
                # We make it turn only on one wheel
                if right_wheel_rpm > left_wheel_rpm:
                    #print("GO FORWARDS RIGHT")
                    self.left()
                if right_wheel_rpm < left_wheel_rpm:
                    #print("GO FORWARDS LEFT")
                    self.right()
                if right_wheel_rpm == left_wheel_rpm:
                    #print("GO FORWARDS")
                    self.forward()
            else:
                #print("GO FORWARDS")
                self.forward()

        elif right_wheel_rpm > 0.0 and left_wheel_rpm == 0.0:
            #print("Right Wheel forwards, left stop")
            self.set_M1M2_speed(abs(right_wheel_rpm), abs(left_wheel_rpm), self.MULTIPLIER_STANDARD)
            self.right()
            
        elif right_wheel_rpm > 0.0 and left_wheel_rpm < 0.0:
            #print("Right Wheel forwards, left backwards --> Pivot left")
            self.set_M1M2_speed(abs(right_wheel_rpm), abs(left_wheel_rpm), self.MULTIPLIER_PIVOT)
            self.pivot_right()
            
        elif right_wheel_rpm == 0.0 and left_wheel_rpm > 0.0:
            #print("Right stop, left forwards")
            self.set_M1M2_speed(abs(right_wheel_rpm), abs(left_wheel_rpm), self.MULTIPLIER_STANDARD)
            self.left()

        elif right_wheel_rpm < 0.0 and left_wheel_rpm > 0.0:
            #print("Right backwards, left forwards --> Pivot right")
            self.set_M1M2_speed(abs(right_wheel_rpm), abs(left_wheel_rpm), self.MULTIPLIER_PIVOT)
            self.pivot_left()
            
        elif right_wheel_rpm < 0.0 and left_wheel_rpm < 0.0:
            #print("All backwards")
            self.set_M1M2_speed(abs(right_wheel_rpm), abs(left_wheel_rpm), self.MULTIPLIER_STANDARD)

            if self.simple_mode:
                # We make it turn only on one wheel
                if abs(right_wheel_rpm) > abs(left_wheel_rpm):
                    #print("GO BACKWARDS RIGHT")
                    self.left_reverse()
                if abs(right_wheel_rpm) < abs(left_wheel_rpm):
                    #print("GO BACKWARDS LEFT")
                    self.right_reverse()
                if right_wheel_rpm == left_wheel_rpm:
                    #print("GO BACKWARDS")
                    self.reverse()
            else:
                self.reverse()

        elif right_wheel_rpm == 0.0 and left_wheel_rpm == 0.0:
            #print("Right stop, left stop")
            self.set_M1M2_speed(abs(right_wheel_rpm), abs(left_wheel_rpm), self.MULTIPLIER_STANDARD)
            self.stop()
        else:
            assert False, "A case wasn't considered==>"+str(right_wheel_rpm)+","+str(left_wheel_rpm)

        self.command_pub.publish(self.command_msg)
        
        
    def set_cmd_vel(self, linear_speed, angular_speed):

        body_turn_radius = self.calculate_body_turn_radius(linear_speed, angular_speed)

        wheel = "right"
        right_wheel_turn_radius = self.calculate_wheel_turn_radius(body_turn_radius,
                                                                   angular_speed,
                                                                   wheel)

        wheel = "left"
        left_wheel_turn_radius = self.calculate_wheel_turn_radius(body_turn_radius,
                                                                  angular_speed,
                                                                  wheel)

        right_wheel_rpm = self.calculate_wheel_rpm(linear_speed, angular_speed, right_wheel_turn_radius)
        left_wheel_rpm = self.calculate_wheel_rpm(linear_speed, angular_speed, left_wheel_turn_radius)


        self.set_wheel_movement(right_wheel_rpm, left_wheel_rpm)
