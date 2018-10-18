#!/usr/bin/env python
# -*- coding: utf-8 -*-
__author__ = 'Matias Pavez'
__email__ = 'matias.pavez.b@gmail.com'

import rospy
import sys
import math
from sensor_msgs.msg import Joy
from maqui_joy import xbox
from maqui_skills import robot_factory

class JoystickHead(object):
  
    def __init__(self, robot):
        rospy.loginfo('Joystick head init ...')

        # connections

        self.maqui = robot
        # load head configuration
        self.b_pause = rospy.get_param('~b_pause', 'START')
        
        b_neck_actuate = rospy.get_param('~b_neck_actuate', 'RS')
        a_neck_sides   = rospy.get_param('~a_neck_sides', 'RS_HORZ')
        a_neck_front   = rospy.get_param('~a_neck_front', 'RS_VERT')
        
        b_up           = rospy.get_param('~b_up', 'UP')
        b_down         = rospy.get_param('~b_down', 'DOWN')
        b_left         = rospy.get_param('~b_left', 'LEFT')
        b_right        = rospy.get_param('~b_right', 'RIGHT')

        b_emotion      = rospy.get_param('~b_emotion_trigger', 'RB')
        b_action1      = rospy.get_param('~b_action1', 'X')
        b_action2      = rospy.get_param('~b_action2', 'Y')
        b_action3      = rospy.get_param('~b_action3', 'B')
        b_home         = rospy.get_param('~b_home', 'A')

        # convert to ids
        key_mapper = xbox.KeyMapper()
        self.b_idx_pause = key_mapper.get_button_id(self.b_pause)
        self.a_idx_neck_sides   = key_mapper.get_axis_id(a_neck_sides)
        self.a_idx_neck_front   = key_mapper.get_axis_id(a_neck_front)
        self.b_idx_neck_actuate = key_mapper.get_button_id(b_neck_actuate)

        self.b_idx_neck_up = key_mapper.get_button_id(b_up)
        self.b_idx_neck_down = key_mapper.get_button_id(b_down)
        self.b_idx_neck_left = key_mapper.get_button_id(b_left)
        self.b_idx_neck_right = key_mapper.get_button_id(b_right)


        self.b_idx_emotion  = key_mapper.get_button_id(b_emotion)
        self.b_idx_action1  = key_mapper.get_button_id(b_action1)
        self.b_idx_action2  = key_mapper.get_button_id(b_action2)
        self.b_idx_action3  = key_mapper.get_button_id(b_action3)
        self.b_idx_home     = key_mapper.get_button_id(b_home)

        # check
        self.assert_params()
        

        # control
        self.is_paused = False

        #neck control
        self.yaw_step = 0.25

        self.pitch_step = 0.25

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


    def move_head(self, yaw, pitch):
        actual_yaw  = self.maqui.neck.get_yaw()
        actual_pitch  = self.maqui.neck.get_pitch()
        
        self.maqui.neck.send_joint_goal(yaw=actual_yaw + yaw * self.yaw_step, pitch = actual_pitch + pitch * self.pitch_step, interval= 0.5, segments=10)
        return 


    def set_emotion(self, emotion_str):
        return 
    def callback(self, msg):

        # pause
        if msg.buttons[self.b_idx_pause]:

            self.is_paused = not self.is_paused
            if self.is_paused:                
                rospy.logwarn("\nControlling PAUSED!, press the " + self.b_pause + " button to resume it\n")
            else:
                rospy.logwarn("Controlling RESUMED, press " + self.b_pause + " button to pause it")

            # very important sleep!
            # prevents multiple triggersfor the same button
            rospy.sleep(1) # it should be >= 1;
            return
        
        # work
        if not self.is_paused:
            # neck
            if msg.buttons[self.b_idx_neck_actuate]:

                neck_side  = msg.axes[self.a_idx_neck_sides]
                neck_front = msg.axes[self.a_idx_neck_front]

                self.move_head(neck_side, neck_front)
                return


            if msg.buttons[self.b_idx_neck_up] :
                self.maqui.neck.move_up()

            elif msg.buttons[self.b_idx_neck_down] :
                self.maqui.neck.move_down()

            elif msg.buttons[self.b_idx_neck_left] :
                self.maqui.neck.move_left()

            elif msg.buttons[self.b_idx_neck_right] :
                self.maqui.neck.move_right()


            if msg.buttons[self.b_idx_emotion]:
                pass
            else:

                if msg.buttons[self.b_idx_home]:
                    self.maqui.neck.home()
                elif msg.buttons[self.b_idx_action1]:
                    self.maqui.neck.nod()
                elif msg.buttons[self.b_idx_action2]:
                    self.maqui.neck.look_person()

                elif msg.buttons[self.b_idx_action3]:
                    self.maqui.neck.nope()

            # # head emotion in/decrement
            # elif msg.buttons[self.b_idx_decrement]:
            #     self.emotion_intensity = int(max(self.min_emotion_intensity, self.emotion_intensity - 1))
            #     rospy.loginfo("Emotion intensity decrement: %d" % self.emotion_intensity)
            # elif msg.buttons[self.b_idx_increment]:
            #     self.emotion_intensity = int(min(self.max_emotion_intensity, self.emotion_intensity + 1))
            #     rospy.loginfo("Emotion intensity increment: %d" % self.emotion_intensity)
            

            # # head emotions: happy, angry, sad, surprise
            # elif msg.buttons[self.b_idx_happy]:
            #     self.set_emotion("happy%d" % self.emotion_intensity)
            
            # elif msg.buttons[self.b_idx_angry]:
            #     self.set_emotion("angry%d" % self.emotion_intensity)
            
            # elif msg.buttons[self.b_idx_sad]:
            #     self.set_emotion("sad%d" % self.emotion_intensity)
            
            # elif msg.buttons[self.b_idx_surprise]:
            #     self.set_emotion("surprise")

            return


if __name__ == '__main__':
    rospy.init_node('joy_head')
    robot = robot_factory.build(["neck", "face"])
    JoystickHead(robot)
    rospy.spin()
