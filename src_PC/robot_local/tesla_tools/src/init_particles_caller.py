#! /usr/bin/env python3

import rospy
from std_srvs.srv import Empty, EmptyRequest

rospy.init_node('service_client')
rospy.wait_for_service("/global_localization")
disperse_particles_client = rospy.ServiceProxy("/global_localization", Empty)
disperse_particles_object = EmptyRequest()
res = disperse_particles_client(disperse_particles_object)
print(res)