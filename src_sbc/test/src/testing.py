#!/usr/bin/env python
import rospy
from time import sleep
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

def callback(msg):
    print(len(msg.ranges))
    front_laser,left,right,behind = msg.ranges[113],msg.ranges[225],msg.ranges[0],msg.ranges[337]
    print(front_laser," ",left," ",right," ",behind)
    cmd = Twist()
    # if front_laser > 0.35:
    #     cmd.linear.x = 1.0
    #     pub.publish(cmd)
    #     rospy.sleep(0.1)
    # elif behind < 1:
    #     cmd.linear.x = 0
    #     pub.publish(cmd)
    # else:
    #     cmd.linear.x = -1.0
    #     pub.publish(cmd)
    #     rospy.sleep(0.1)

    cmd.linear.x = 0.0
    cmd.angular.z = 0
    pub.publish(cmd)
    # rospy.sleep(0.5)

rospy.init_node('laser_reader')
pub = rospy.Publisher('/morpheus_bot/cmd_vel', Twist, queue_size =1)
sub = rospy.Subscriber('/scan',LaserScan,callback)
rospy.spin()