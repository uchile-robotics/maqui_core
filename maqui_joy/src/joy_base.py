#!/usr/bin/env python
# -*- coding: utf-8 -*-
__author__ = 'Matias Pavez'
__email__ = 'matias.pavez.b@gmail.com'

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from naoqi_bridge_msgs.msg import JointAnglesWithSpeed
from maqui_joy import xbox


class JoystickBase(object):

    def __init__(self):
        rospy.loginfo('Joystick base init ...')

        self.pub = rospy.Publisher('base/cmd_vel', Twist, queue_size=1)
        self.head_pub   = rospy.Publisher('/maqui/pose/joint_angles', JointAnglesWithSpeed, queue_size=1)


        # control
        self.is_paused = False

        # load configuration
        self.b_pause = rospy.get_param('~b_pause', 'START')
        a_linear_x = rospy.get_param('~a_linear_x', 'LS_VERT')
        a_linear_y = rospy.get_param('~a_linear_y', 'LS_HORZ')
        a_angular = rospy.get_param('~a_angular', 'RS_HORZ')
        b_increment    = rospy.get_param('~b_increment', 'RB')
        b_decrement    = rospy.get_param('~b_decrement', 'LB')
        a_pitch = rospy.get_param('~a_pitch', 'RS_VERT')
        self.max_linear_vel = rospy.get_param('~max_linear_vel', 0.5)
        self.max_angular_vel = rospy.get_param('~max_angular_vel', 0.5)
        self.max_neck_degrees      = rospy.get_param('~max_neck_degrees', 1.1)

        key_mapper = xbox.KeyMapper()
        self.b_idx_pause = key_mapper.get_button_id(self.b_pause)
        self.a_idx_linear_x = key_mapper.get_axis_id(a_linear_x)
        self.a_idx_linear_y = key_mapper.get_axis_id(a_linear_y)
        self.a_idx_angular = key_mapper.get_axis_id(a_angular)
        self.b_idx_increment = key_mapper.get_button_id(b_increment)
        self.b_idx_decrement = key_mapper.get_button_id(b_decrement)
        self.a_idx_pitch = key_mapper.get_axis_id(a_pitch)

        # check
        self.assert_params()
        self.yaw_angle = 0.0
        self.pitch_angle= 0.0

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
    def move_head(self, yaw,pitch):

        # saturate
        yaw = max(-self.max_neck_degrees, yaw)
        yaw = min(self.max_neck_degrees, yaw) 
        pitch = max(-self.max_neck_degrees, pitch)
        pitch = min(self.max_neck_degrees, pitch) 
        # send
        msg = JointAnglesWithSpeed()
        msg.joint_names = ["HeadYaw","HeadPitch"]
        msg.header.stamp = rospy.Time.now()
        msg.joint_angles = [yaw,pitch]
        msg.speed = 0.1
        #self.head_pub.publish(msg)

        # todo: throttle 0.5s
        rospy.loginfo("Setting neck angle to %f [deg]" % yaw)


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
            self.yaw_angle = self.yaw_angle + 0.05*(msg.buttons[self.b_idx_increment]-msg.buttons[self.b_idx_decrement])
            self.yaw_angle = max(-self.max_neck_degrees, self.yaw_angle)
            self.yaw_angle = min(self.max_neck_degrees, self.yaw_angle) 
            print self.yaw_angle
            self.pitch_angle = self.pitch_angle + 0.05*msg.axes[self.a_idx_pitch]
            self.pitch_angle = max(-self.max_neck_degrees, self.pitch_angle)
            self.pitch_angle = min(self.max_neck_degrees, self.pitch_angle)
            print self.pitch_angle
            self.move_head(self.yaw_angle,self.pitch_angle)
            self.pub.publish(cmd)

if __name__ == '__main__':
    rospy.init_node('joy_base')
    JoystickBase()
    rospy.spin()
