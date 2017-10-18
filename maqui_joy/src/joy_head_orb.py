#!/usr/bin/env python
# -*- coding: utf-8 -*-
__author__ = 'Tim Resink'
__email__ = 'timresink@gmail.com'

import rospy
import sys
import math
from sensor_msgs.msg import Joy
from maqui_joy import xbox

class JoystickHead(object):
  
    def __init__(self):
        rospy.loginfo('Joystick head init ...')

        # connections
        self.head_pub   = rospy.Publisher('head/cmd', Emotion, queue_size=1)


        # load head configuration
        self.b_pause = rospy.get_param('~b_pause', 'START')
        
        b_neck_actuate = rospy.get_param('~b_neck_actuate', 'RS')
        a_neck_sides   = rospy.get_param('~a_neck_sides', 'RS_HORZ')
        a_neck_front   = rospy.get_param('~a_neck_front', 'RS_VERT')
        
        self.max_neck_degrees      = rospy.get_param('~max_neck_degrees', 60)
    
        # convert to ids
        key_mapper = xbox.KeyMapper()
        self.b_idx_pause = key_mapper.get_button_id(self.b_pause)
        self.a_idx_neck_sides   = key_mapper.get_axis_id(a_neck_sides)
        self.a_idx_neck_front   = key_mapper.get_axis_id(a_neck_front)
        self.b_idx_neck_actuate = key_mapper.get_button_id(b_neck_actuate)
       
        # check
        self.assert_params()
        

        # control
        self.is_paused = False


        # ready to work
        rospy.Subscriber('joy', Joy, self.callback, queue_size=1)
        rospy.loginfo('Joystick for head is ready')


    def assert_params(self):
        """
        checks whether the user parameters are valid or no.
        """
        assert isinstance(self.b_idx_pause, int)
        assert isinstance(self.a_idx_neck_sides, int)
        assert isinstance(self.a_idx_neck_front, int)
        assert isinstance(self.b_idx_neck_actuate, int)
   
    def move_head(self, degrees):

        # saturate
        degrees = max(-self.max_neck_degrees, degrees)
        degrees = min(self.max_neck_degrees, degrees) 

        # send
        msg = Emotion()
        msg.Order = "MoveX"
        msg.Action = ""
        msg.X = degrees
        self.head_pub.publish(msg)

        # todo: throttle 0.5s
        rospy.loginfo("Setting neck angle to %f [deg]" % degrees)


    def callback(self, msg):

        # pause
        if msg.buttons[self.b_idx_pause]:

            self.is_paused = not self.is_paused
            if self.is_paused:                
                rospy.logwarn("\nControlling PAUSED!, press the " + self.b_pause + " button to resume it\n")
            else:
                rospy.logwarn("Controlling RESUMED, press " + self.b_pause + " button to pause it")

            # very important sleep!
            # prevents multiple triggers for the same button
            rospy.sleep(1) # it should be >= 1;
            return
        
        # work
        if not self.is_paused:

            # neck
            if msg.buttons[self.b_idx_neck_actuate]:
                neck_side  = msg.axes[self.a_idx_neck_sides]
                neck_front = msg.axes[self.a_idx_neck_front]

                if neck_side*neck_side + neck_front*neck_front > 0.5:
                    angle = math.atan2(neck_side, neck_front)
                    degrees = math.degrees(angle)
                    self.move_head(degrees)

                return

            return


if __name__ == '__main__':
    rospy.init_node('joy_head')
    JoystickHead()
    rospy.spin()
