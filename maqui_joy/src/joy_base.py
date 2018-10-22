#!/usr/bin/env python
# -*- coding: utf-8 -*-
__author__ = 'Matias Pavez'
__email__ = 'matias.pavez.b@gmail.com'

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from maqui_joy import xbox
from maqui_skills import robot_factory

class JoystickBase(object):

    def __init__(self,robot):
        rospy.loginfo('Joystick base init ...')

        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        # control
        self.is_paused = False
        self.robot = robot
        # load configuration
        self.b_pause = rospy.get_param('~b_pause', 'START')
        a_linear_x = rospy.get_param('~a_linear_x', 'LS_VERT')
        a_linear_y = rospy.get_param('~a_linear_y', 'LS_HORZ')
        a_angular = rospy.get_param('~a_angular', 'RS_HORZ')

        b_say_move = rospy.get_param('~b_say_move', 'X')
        b_say_move2 = rospy.get_param('~b_say_move2', 'Y')
        b_say_move3 = rospy.get_param('~b_say_move3', 'B')

        self.max_linear_vel = rospy.get_param('~max_linear_vel', 0.5)
        self.max_angular_vel = rospy.get_param('~max_angular_vel', 0.5)

        key_mapper = xbox.KeyMapper()
        self.b_idx_pause = key_mapper.get_button_id(self.b_pause)
        self.a_idx_linear_x = key_mapper.get_axis_id(a_linear_x)
        self.a_idx_linear_y = key_mapper.get_axis_id(a_linear_y)
        self.a_idx_angular = key_mapper.get_axis_id(a_angular)

        self.b_idx_say1 = key_mapper.get_button_id(b_say_move)
        self.b_idx_say2 = key_mapper.get_button_id(b_say_move2)
        self.b_idx_say3 = key_mapper.get_button_id(b_say_move3)

        # check
        self.assert_params()

        # ready to work
        rospy.Subscriber('joy', Joy, self.callback, queue_size=1)
        rospy.loginfo('Joystick for base is ready')

    def assert_params(self):
        """
        checks whether the user parameters are valid or no.
        """
        assert isinstance(self.b_idx_pause, int)
        assert isinstance(self.a_idx_angular, int)
        assert isinstance(self.a_idx_linear_x, int)
        assert isinstance(self.a_idx_linear_y, int)

    def callback(self, msg):

        # pause
        if msg.buttons[self.b_idx_pause]:

            self.is_paused = not self.is_paused
            if self.is_paused:

                # stop signal
                cmd = Twist()
                self.pub.publish(cmd)

                rospy.logwarn("\nControlling PAUSED!, press the " + self.b_pause + " button to resume it\n")
            else:
                rospy.logwarn("Controlling RESUMED, press " + self.b_pause + " button to pause it")

            # very important sleep!
            # prevents multiple triggers for the same button
            rospy.sleep(1)  # it should be >= 1;
            return

        # work
        if not self.is_paused:
            cmd = Twist()
            cmd.angular.z = self.max_angular_vel * msg.axes[self.a_idx_angular]
            cmd.linear.x = self.max_linear_vel * msg.axes[self.a_idx_linear_x]
            cmd.linear.y = self.max_linear_vel * msg.axes[self.a_idx_linear_y]

            if msg.buttons[self.b_idx_say1]:
                self.robot.say(" Hacedme espacio para avanzar, porfavor")
                rospy.sleep(1)
            elif msg.buttons[self.b_idx_say2]:
                self.robot.say(" Permiso, porfavor")
                rospy.sleep(1)
            elif msg.buttons[self.b_idx_say3]:
                self.robot.say(" Cuidado! robot pasando")
                rospy.sleep(1)

            self.pub.publish(cmd)

if __name__ == '__main__':
    rospy.init_node('joy_base')
    robot = robot_factory.build(['tts'])
    JoystickBase(robot)
    rospy.spin()
