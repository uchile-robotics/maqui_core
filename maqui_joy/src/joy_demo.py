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
        self.robot.basic_motion.set_idle()
        self.robot.basic_motion.set_breathing()
        # load tts configuration
        self.tts_phrases = rospy.get_param('~tts_phrases', [])
        self.animations  = rospy.get_param('~animations', [])


        self.b_pause   = rospy.get_param('~b_pause', 'START')
        b_increment    = rospy.get_param('~b_increment', 'UP')
        b_decrement    = rospy.get_param('~b_decrement', 'DOWN')
        b_phrase_1     = rospy.get_param('~b_phrase_1', 'LEFT')
        b_phrase_2     = rospy.get_param('~b_phrase_2', 'RIGHT')        
        b_life         = rospy.get_param('~b_life', 'A')
        b_photo        = rospy.get_param('~b_photo', 'Y')
        
        b_tablet_reset = rospy.get_param('~b_tablet_reset', 'RB')
        b_tablet_logo  = rospy.get_param('~b_tablet_logo', 'B')
        
        b_behavior  = rospy.get_param('~b_behavior', 'LB')
        b_animation = rospy.get_param('~b_b_animation', 'X')
        key_mapper = xbox.KeyMapper()
        
        self.b_idx_pause        = key_mapper.get_button_id(self.b_pause)
        self.b_idx_increment    = key_mapper.get_button_id(b_increment)
        self.b_idx_decrement    = key_mapper.get_button_id(b_decrement)
        self.b_idx_phrase_1     = key_mapper.get_button_id(b_phrase_1)
        self.b_idx_phrase_2     = key_mapper.get_button_id(b_phrase_2)
        self.b_idx_life         = key_mapper.get_button_id(b_life)
        self.b_idx_tablet_logo  = key_mapper.get_button_id(b_tablet_logo)
        self.b_idx_tablet_reset = key_mapper.get_button_id(b_tablet_reset)
        self.b_idx_photo        = key_mapper.get_button_id(b_photo)
        self.b_idx_behavior     = key_mapper.get_button_id(b_behavior)
        self.b_idx_animation     = key_mapper.get_button_id(b_animation)

        # check
        self.assert_params()

        # control
        self.is_recording = False
        self.head_on      = False
        self.is_paused    = False
        self.tts_level    = 0

        self.animation_level = 0

        self.robot.media_camera.set_picture_params(resolution="2560x1920")
        # ready to work
        rospy.Subscriber('joy', Joy, self.callback, queue_size=1)

        self.logo_iterator = 0
        self.logos  = [["amtc_png.png", "#FFFFFF"],["uch.png","#FFFFFF"],["hb.png","#000000"],["fcfm.png","#000000"],["bender.png","#000000"],["uch_peppers.png","#000000"]]

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
        assert isinstance(self.b_idx_life, int)
        assert isinstance(self.b_idx_tablet_logo , int)
        assert isinstance(self.b_idx_tablet_reset, int)
        assert isinstance(self.b_idx_photo, int)
        assert isinstance(self.b_idx_behavior, int)
        assert isinstance(self.b_idx_animation, int)

        # check config
        if not len(self.tts_phrases):
            rospy.logwarn("Using Zero phrases for tts!")

        if not len(self.animations):
            rospy.logwarn("Using Zero animations!")


    def synthesize(self, text):
        
        try:
            rospy.loginfo("synthesizing text: %s" % text)
            self.robot.tts.say_with_gestures(text.replace("gn", "ñ"))
            rospy.sleep(0.5)
        except Exception, e:
            rospy.logwarn("TTS server failed!")


    def tablet_image(self):
        logo = self.logos[self.logo_iterator%len(self.logos)]
        self.logo_iterator+=1
        rospy.loginfo("Showing {0} Logo in tablet".format(logo))
        self.robot.tablet.show_image("http://198.18.0.1/apps/media/img/logos/{0}".format(logo[0]),logo[1])

    def reset_tablet(self):
        rospy.loginfo("Resetting tablet")
        self.robot.tablet.sleep()
        self.robot.tablet.wakeUp()
        self.logo_iterator-=1
        self.robot.tablet._resetTablet()

    def photo(self):
        rospy.loginfo("Taking photos")
        self.robot.tts.say_with_gestures("Tomaré una foto! \\pau=500\\ digan chiiss")
        self.robot.behavior_manager.start_behavior("Stand/Waiting/TakePicture_1")
        rospy.sleep(3)
        pic = self.robot.media_camera.take_picture("Joystick")
        rospy.sleep(1)
        self.robot.tablet.show_image("img/photos/"+pic)
        self.logo_iterator-=1

    def basic_awareness(self):
        if not self.head_on:
            self.robot.basic_awareness.look_around()
            #self.robot.behavior_manager.start_behavior("run_dialog_dev/init")
        else:
            self.robot.basic_awareness.stop()
            #self.robot.behavior_manager.stop_behavior("run_dialog_dev/init")
            self.robot.basic_motion.set_posture()
        self.head_on = not self.head_on

    def play_behavior(self, emotion):
        rospy.loginfo("Showing Emotion %s" % emotion)        
        self.robot.behavior_manager.play_behavior_tag(emotion)

    def play_animation(self, animation):
        rospy.loginfo("Playing animation %s" % animation)
        self.robot.behavior_manager.run_behavior(animation)

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
            if msg.buttons[self.b_idx_behavior]:
                
                if msg.buttons[self.b_idx_life]:
                    self.play_behavior("sad")

                elif msg.buttons[self.b_idx_tablet_logo ]:
                    self.play_behavior("happy")

                elif msg.buttons[self.b_idx_tablet_reset]:
                    self.play_behavior("ashamed")

                elif msg.buttons[self.b_idx_photo]:
                    self.play_behavior("angry")

                elif msg.buttons[self.b_idx_increment]:
                    n_animation = len(self.animations)
                    if n_animation > 0:
                        # print "n_animation: " + n_animation

                        animation_levels = int(math.ceil(n_animation/2.0))
                        # print "animation_levels: " + animation_levels
                        self.animation_level = (self.animation_level+1)%animation_levels
                        # print "animation_leve: " + self.animation_leve
                        rospy.loginfo("animation level %d/%d" % (self.animation_level + 1, animation_levels))
                        rospy.loginfo("LEFT (<-) %s  || RIGHT (->) %s" % (self.animations[2*self.animation_level] , self.animations[(2*self.animation_level+1)%n_animation] ))
                        rospy.sleep(0.25)

                elif msg.buttons[self.b_idx_decrement]:
                    n_animation = len(self.animations)
                    if n_animation > 0:
                        animation_levels = int(math.ceil(n_animation/2.0))
                        self.animation_level = max(0, self.animation_level - 1)
                        rospy.loginfo("animation level %d/%d" % (self.animation_level + 1, animation_levels))
                        rospy.loginfo("LEFT (<-) %s  ||  RIGHT (->) %s" % (self.animations[2*self.animation_level] , self.animations[(2*self.animation_level+1)%n_animation] ))
                        rospy.sleep(0.25)

                elif msg.buttons[self.b_idx_phrase_1]:
                    n_animation = len(self.animations)
                    if n_animation > 0:
                        animation_idx = self.animation_level*2
                        ani = self.animations[animation_idx]

                        self.play_animation(ani)
                        rospy.sleep(0.25)

                # play_animation 2
                elif msg.buttons[self.b_idx_phrase_2]:
                    n_animation = len(self.animations)
                    if n_animation > 0:
                        animation_idx = (self.animation_level*2 + 1) % n_animation
                        ani = self.animations[animation_idx]
                        self.play_animation(ani)
                        rospy.sleep(0.25)

                return
            # synthesize increment
            elif msg.buttons[self.b_idx_increment]:
                n_phrases = len(self.tts_phrases)
                if n_phrases > 0:
                    # print "n_phrases: " + n_phrases

                    levels = int(math.ceil(n_phrases/2.0))
                    # print "levels: " + levels
                    self.tts_level = (self.tts_level+1)%levels
                    # print "tts_level: " + self.tts_level
                    rospy.loginfo("tts level %d/%d" % (self.tts_level + 1, levels))
                    rospy.loginfo("LEFT (<-) %s  || RIGHT (->) %s" % (self.tts_phrases[2*self.tts_level] , self.tts_phrases[(2*self.tts_level+1)%n_phrases] ))
                    rospy.sleep(0.25)
            
            # synthesize decrement
            elif msg.buttons[self.b_idx_decrement]:
                n_phrases = len(self.tts_phrases)
                if n_phrases > 0:
                    levels = int(math.ceil(n_phrases/2.0))
                    self.tts_level = max(0, self.tts_level - 1)
                    rospy.loginfo("tts level %d/%d" % (self.tts_level + 1, levels))
                    rospy.loginfo("LEFT (<-) %s  || RIGHT (->) %s" % (self.tts_phrases[2*self.tts_level] , self.tts_phrases[(2*self.tts_level+1)%n_phrases] ))
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
            elif msg.buttons[self.b_idx_life]:
                self.basic_awareness()

            elif msg.buttons[self.b_idx_tablet_logo ]:
                self.tablet_image()

            elif msg.buttons[self.b_idx_tablet_reset]:
                self.reset_tablet()

            elif msg.buttons[self.b_idx_photo]:
                self.photo()


            return

if __name__ == '__main__':
    rospy.init_node('joy')
    robot = robot_factory.build(["tts","basic_awareness","basic_motion","media_camera","tablet","behavior_manager"], core=False)
    Joystick(robot)
    rospy.spin()
