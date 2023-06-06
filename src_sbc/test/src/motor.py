#!/usr/bin/env python
import numpy as np

# m1: Right motors
# m2: Left motors
# Max linear speed: 2.13m / 3s -> 0.71 m/s
# Max motor RPM: ~ 205 RPM
# R = 3.3 cm = 0.033m
# L = 14.7 cm = 0.147m

def setSpeed(m1, m2):
    m1_in1 = 0
    m1_in2 = 0

    m2_in1 = 0
    m2_in2 = 0

    if m1 == 0:
        m1_in1 = 0
        m1_in2 = 0

    elif m1 < 0:
        m1_in1 = 0
        m1_in2 = 65535

    else:
        m1_in1 = 65535
        m1_in2 = 0

    if m2 == 0:
        m2_in1 = 0
        m2_in2 = 0

    elif m2 < 0:
        m2_in1 = 0
        m2_in2 = 65535

    else:
        m2_in1 = 65535
        m2_in2 = 0

    return Int32MultiArray(data=[
        int(abs(m1)*65535), m1_in1, m1_in2, m2_in1, m2_in2, int(abs(m2)*65535), -1, -1,
        -1, -1, -1, -1, -1, -1, -1, -1
    ])


class robotDriver():
    def __init__(self, r, l):
        self.lin = 0.0
        self.ang = 0.0
        self.r = r
        self.l = l

        self.maxRPM = (60 * 0.71) / (3.14 * self.r * 2)

    def getVeloPair(self):
        a = np.array([[1, 1], [1, -1]])
        b = np.array([self.lin * 2 / self.r, self.ang * self.l / self.r])

        return np.linalg.inv(a).dot(b)

    def convertVeloToPower(self, veloPair):
        res = []

        for i in veloPair:
            wheel_rpm = (i / (2 * 3.14 / 60)) / self.maxRPM

            if self.ang != 0 and wheel_rpm < 0:
                wheel_rpm -= 0.35

            if self.ang != 0 and wheel_rpm > 0:
                wheel_rpm += 0.35

            if abs(wheel_rpm) > 1:
                wheel_rpm /= abs(wheel_rpm)

            res.append(wheel_rpm)

        return res

    def setCarSpeed(self, lin, ang):
        self.lin = lin
        self.ang = ang
        
        pair = self.convertVeloToPower(self.getVeloPair())
        print(f"Power for motors: {pair}")

        return setSpeed(pair[0], pair[1])
        

import rospy
import time
from std_msgs.msg import Int32MultiArray
pub = rospy.Publisher('/command', Int32MultiArray, queue_size=10)
rospy.init_node('your_node_name')
r = rospy.Rate(10) # 10hz

# robot = robotDriver(0.033, 0.147)
# instr = robot.setCarSpeed(0, robot.maxRPM)

motor1 = setSpeed(1, 0)
motor2 = setSpeed(0, 0.5)

foward = setSpeed(0.75, 0.75)

turn_right = setSpeed(0.75, 0)

turn_left = setSpeed(0, 0.75)

rev = setSpeed(-0.5, -0.5)

stop = setSpeed(0, 0)

i = 0.0

while not rospy.is_shutdown():
    time.sleep(1.0)
    # pub.publish(turn_right)
    # time.sleep(3.0)
    pub.publish(motor1)
    time.sleep(1.0)
    # pub.publish(stop)
    # time.sleep(1.0)
    # pub.publish(foward)
    # time.sleep(1.0)
    pub.publish(stop)
    r.sleep()
    break