import rospy
from time import sleep
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

def callback(msg):
    print(len(msg.ranges))
    front_laser,l,r = msg.ranges[113],msg.ranges[225],msg.ranges[440]
    print(front_laser," ",l," ",r)
    cmd = Twist()
    # if front_laser > 0.1 and l > 0.02 and r > 0.02:
    #     cmd.linear.x = 0.5
    #     pub.publish(cmd)
    # else:
    #     cmd.linear.x = -0.5
    #     pub.publish(cmd)

    cmd.linear.x = 0.0
    pub.publish(cmd)

rospy.init_node('laser_reader')
pub = rospy.Publisher('/morpheus_bot/cmd_vel', Twist, queue_size =1)
sub = rospy.Subscriber('/scan',LaserScan,callback)
rospy.spin()
