#!/usr/bin/env python
from robot_control_class import RobotControl
import time
# Counterclockwise 90 deg turn and 1.0 speed: ~1 second
# Clockwise 90 deg turn and 1.0 speed: 0.85

class MoveRobot:
    def __init__(self, motion, clockwise, speed, time):
        self.rc = RobotControl()
        self.motion = motion
        self.clockwise = clockwise
        self.speed = speed
        self.time = time

    def do_square(self):
        # Loop through 4 times to make a square
        for i in range(4):
            self.move_straight()
            self.turn()

    def move_straight(self):
        # Move the robot straight for a given time
        self.rc.move_straight_time(self.motion, self.speed, self.time)

    def turn(self):
        # Turn the robot 90 degrees in the given direction
        self.rc.turn(self.clockwise, self.speed, 7.5)

    def move_out(self):
        while True:
            # Get the laser readings
            laser_readings = self.rc.get_laser_full()
            # Check if there is a wall in front of the robot
            if laser_readings[360] > 100:
                self.rc.move_straight()
            i = 0
            while laser_readings[360] > 1:
                self.rc.move_straight()
                laser_readings = self.rc.get_laser_full()
                i += 1
                print("Current distance to wall: %f" % laser_readings[360])

            # Stop the robot when it reaches the wall
            self.rc.stop_robot()
            print("Wall is at %f meters! Stop the robot!" % laser_readings[360])
            
            self.rc.rotate(-i * 1.25)
            print("Self adjust to the left")

            # Determine which direction to turn based on the laser readings
            if laser_readings[0] > laser_readings[719]:
                # Turn right
                self.rc.rotate(90)
                print("Rotate the robot to the right!")
            else:
                # Turn left
                print("Rotate the robot to the left!")
                self.rc.rotate(-90)


# Create an instance of the MoveRobot class and call the move_out method
robot = MoveRobot('backward', 'clockwise', 1.0, 1)
robot.rc.rotate(90)