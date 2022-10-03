#!/usr/bin/env python

import rospy #importar ros para python
from std_msgs.msg import String, Int32 # importar mensajes de ROS tipo String y tipo Int32
from geometry_msgs.msg import Twist # importar mensajes de ROS tipo geometry / Twist
from sensor_msgs.msg import Joy, LaserScan

class Template(object):
	def __init__(self, args):
		super(Template, self).__init__()
		self.args = args
		self.sub=rospy.Subscriber('/maqui/laser/front',LaserScan,self.callback)
		self.pub = rospy.Publisher('/maqui/cmd_vel', Twist, queue_size=1)

	def callback(self,msg):
		if msg.ranges[4] <= 1:
			print('Warning')
			cmd = Twist()
	        	cmd.angular.z = 0
        	    	cmd.linear.x = 0
        	    	cmd.linear.y = 0
			cmd.linear.z = 0
        	    	self.pub.publish(cmd)


def main():
	rospy.init_node('test') #creacion y registro del nodo!

	obj = Template('args') # Crea un objeto del tipo Template, cuya definicion se encuentra arriba

	#objeto.publicar() #llama al metodo publicar del objeto obj de tipo Template

	rospy.spin() #funcion de ROS que evita que el programa termine -  se debe usar en  Subscribers


if __name__ =='__main__':
	main()
