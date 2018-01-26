#!/usr/bin/env python
# -*- coding: utf-8 -*-

__author__ = 'Lukas Pavez'

import rospy
import math
from sensor_msgs.msg import Joy
from maqui_joy import xbox
from maqui_skills import robot_factory


class Joystick(object):
  
    def __init__(self,robot):
        rospy.loginfo('Joystick Joy demo init ...')
        
        self.robot = robot
        self.robot.tts.set_language("Spanish")
        self.robot.tts.set_gestures_mode(1)

        # load tts configuration
        self.tts_phrases = rospy.get_param('~tts_phrases', [])

        self.b_pause = rospy.get_param('~b_pause', 'START')
        b_increment = rospy.get_param('~b_increment', 'UP')
        b_decrement = rospy.get_param('~b_decrement', 'DOWN')
        b_phrase_1  = rospy.get_param('~b_phrase_1', 'LEFT')
        b_phrase_2  = rospy.get_param('~b_phrase_2', 'RIGHT')
        b_head      = rospy.get_param('~b_head', 'A')
        b_tablet_image = rospy.get_param('~b_tablet_image', 'B')
        b_tablet_reset = rospy.get_param('~b_tablet_reset', 'LB')
        b_record = rospy.get_param('~b_record', 'X')
        b_photo = rospy.get_param('~b_photo', 'Y')

        key_mapper = xbox.KeyMapper()
        self.b_idx_pause = key_mapper.get_button_id(self.b_pause)
        self.b_idx_increment = key_mapper.get_button_id(b_increment)
        self.b_idx_decrement = key_mapper.get_button_id(b_decrement)
        self.b_idx_phrase_1  = key_mapper.get_button_id(b_phrase_1)
        self.b_idx_phrase_2  = key_mapper.get_button_id(b_phrase_2)
        self.b_idx_head      = key_mapper.get_button_id(b_head)
        self.b_idx_tablet_image = key_mapper.get_button_id(b_tablet_image)
        self.b_idx_tablet_reset = key_mapper.get_button_id(b_tablet_reset)
        self.b_idx_record = key_mapper.get_button_id(b_record)
        self.b_idx_photo = key_mapper.get_button_id(b_photo)
        
        # check
        self.assert_params()


        # control
        self.is_recording = False
        self.head_on      = False
        self.is_paused    = False
        self.tts_level    = 0
        self.robot.media_camera.set_picture_params(resolution="2560x1920")
        # ready to work
        rospy.Subscriber('joy', Joy, self.callback, queue_size=1)
        rospy.loginfo('Joystick for Joy is ready')


    def assert_params(self):
        """
        checks whether the user parameters are valid or no.
        """
        assert isinstance(self.b_idx_pause, int)
        assert isinstance(self.b_idx_increment, int)
        assert isinstance(self.b_idx_decrement, int)
        assert isinstance(self.b_idx_phrase_1, int)
        assert isinstance(self.b_idx_phrase_2, int)
        assert isinstance(self.b_idx_head, int)
        assert isinstance(self.b_idx_tablet_image, int)
        assert isinstance(self.b_idx_tablet_reset, int)
        assert isinstance(self.b_idx_record, int)
        assert isinstance(self.b_idx_photo, int)

        # check config
        if not len(self.tts_phrases):
            rospy.logwarn("Using Zero phrases for tts!")


    def synthesize(self, text):
        
        try:
            rospy.loginfo("synthesizing text: %s" % text)
            self.robot.tts.say_with_gestures(text)
            rospy.sleep(0.5)
        except Exception, e:
            rospy.logwarn("TTS server failed!")

    def head(self):
        if not self.head_on:
            self.robot.basic_awareness.look_around()
        else:
            self.robot.basic_awareness.stop()
            self.robot.basic_motion.set_posture()
        self.head_on = not self.head_on

    def tablet_image(self):
        rospy.loginfo("Showing amtc logo in tablet")
        self.robot.tablet.show_image("http://198.18.0.1/apps/media/amtc_png.png","#FFFFFF")

    def reset_tablet(self):
        rospy.loginfo("Resetting tablet")
        self.robot.tablet.sleep()
        self.robot.tablet.wakeUp()
        self.robot.tablet._resetTablet()

    def record(self):
        if not self.is_recording:
            rospy.loginfo("Start video recording")
            self.robot.media_camera.start_video_record("Joystick")
        else:
            rospy.loginfo("Stop video recording")
            self.robot.media_camera.stop_video_record()
        self.is_recording = not self.is_recording

    def photo(self):
        rospy.loginfo("Taking photos")
        self.robot.media_camera.take_pictures("Joystick")

    def callback(self, msg):

        # pause
        if msg.buttons[self.b_idx_pause]:

            self.is_paused = not self.is_paused
            if self.is_paused:                
                rospy.logwarn("\nControlling PAUSED!, press start button to resume it\n")
            else:
                rospy.logwarn("Controlling RESUMED, press start button to pause it")

            # very important sleep!
            # prevents multiple triggersfor the same button
            rospy.sleep(1) # it should be >= 1;
            return
        
        # work
        if not self.is_paused:

            # synthesize increment
            if msg.buttons[self.b_idx_increment]:
                n_phrases = len(self.tts_phrases)
                if n_phrases > 0:
                    # print "n_phrases: " + n_phrases

                    levels = int(math.ceil(n_phrases/2.0))
                    # print "levels: " + levels
                    self.tts_level = (self.tts_level+1)%levels
                    # print "tts_level: " + self.tts_level
                    rospy.loginfo("tts level %d/%d" % (self.tts_level + 1, levels))
                    rospy.sleep(0.25)

            # synthesize decrement
            elif msg.buttons[self.b_idx_decrement]:
                n_phrases = len(self.tts_phrases)
                if n_phrases > 0:
                    levels = int(math.ceil(n_phrases/2.0))
                    self.tts_level = max(0, self.tts_level - 1)
                    rospy.loginfo("tts level %d/%d" % (self.tts_level + 1, levels))
                    rospy.sleep(0.25)
            
            # synthesize 1
            elif msg.buttons[self.b_idx_phrase_1]:
                n_phrases = len(self.tts_phrases)
                if n_phrases > 0:
                    phrase_idx = self.tts_level*2
                    text = self.tts_phrases[phrase_idx]
                    self.synthesize(text)

            # synthesize 2
            elif msg.buttons[self.b_idx_phrase_2]:
                n_phrases = len(self.tts_phrases)
                if n_phrases > 0:
                    phrase_idx = (self.tts_level*2 + 1) % n_phrases
                    text = self.tts_phrases[phrase_idx]
                    self.synthesize(text)

            # start head
            elif msg.buttons[self.b_idx_head]:
                self.head()

            elif msg.buttons[self.b_idx_tablet_image]:
                self.tablet_image()

            elif msg.buttons[self.b_idx_tablet_reset]:
                self.reset_tablet()

            elif msg.buttons[self.b_idx_record]:
                self.record()

            elif msg.buttons[self.b_idx_photo]:
                self.photo()

            return

if __name__ == '__main__':
    rospy.init_node('joy')
    robot = robot_factory.build(["tts","basic_awareness","basic_motion","media_camera","tablet"], core=False)
    Joystick(robot)
    rospy.spin()
