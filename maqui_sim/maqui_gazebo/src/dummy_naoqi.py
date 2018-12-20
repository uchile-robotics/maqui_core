#!/usr/bin/env python

import roslib
import rospy
from std_srvs.srv import Empty
class NAOqi:
	def __init__(self):
		self.wake_up_srv = rospy.Service('wake_up', Empty, self.wake_up)

	def wake_up(self,req):
		return 


if __name__ == '__main__':
	rospy.init_node('naoqi_dummy')
	nao = NAOqi()
	rospy.spin()
