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
from maqui_skills.core.tts import TTSSkill

import rospy
import actionlib

from std_msgs.msg import( String )
from std_srvs.srv import( Empty, EmptyResponse )

from naoqi_bridge_msgs.msg import(
    SpeechWithFeedbackGoal,
    SpeechWithFeedbackResult,
    SpeechWithFeedbackFeedback,
    SpeechWithFeedbackAction )


class JoystickTTS(object):
  
    def __init__(self,robot):
        rospy.loginfo('Joystick TTS init ...')
        
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
        b_start_head= rospy.get_param('~b_start_head', 'A')
        b_stop_head = rospy.get_param('~b_stop_head', 'B')

        key_mapper = xbox.KeyMapper()
        self.b_idx_pause = key_mapper.get_button_id(self.b_pause)
        self.b_idx_increment = key_mapper.get_button_id(b_increment)
        self.b_idx_decrement = key_mapper.get_button_id(b_decrement)
        self.b_idx_phrase_1  = key_mapper.get_button_id(b_phrase_1)
        self.b_idx_phrase_2  = key_mapper.get_button_id(b_phrase_2)
        self.b_idx_start_head= key_mapper.get_button_id(b_start_head)
        self.b_idx_stop_head = key_mapper.get_button_id(b_stop_head)
        
        # check
        self.assert_params()


        # control
        self.is_paused = False
        self.tts_level = 0

        # ready to work
        rospy.Subscriber('joy', Joy, self.callback, queue_size=1)
        rospy.loginfo('Joystick for TTS is ready')


    def assert_params(self):
        """
        checks whether the user parameters are valid or no.
        """
        assert isinstance(self.b_idx_pause, int)
        assert isinstance(self.b_idx_increment, int)
        assert isinstance(self.b_idx_decrement, int)
        assert isinstance(self.b_idx_phrase_1, int)
        assert isinstance(self.b_idx_phrase_2, int)
        assert isinstance(self.b_idx_start_head, int)
        assert isinstance(self.b_idx_stop_head, int)

        # check config
        if not len(self.tts_phrases):
            rospy.logwarn("Using Zero phrases for tts!")


    def plugin(self):
        pass
        #     void  Joystick::synthesize(std::string text){
        #     uchile_srvs::String speech_text;
        #     std::string talk = text;
        #     std::string text_evaluate = text;
        #
        #     std::size_t found_enter,found_emotion;
        #     bool end = false;
        #     std::string emo;
        #     while(!end){
        #         found_enter = text_evaluate.find("//");//Enter
        #         found_emotion = text_evaluate.find("@");//Enter
        #         talk = text_evaluate;
        #         if (found_enter!=std::string::npos && found_enter<found_emotion){
        #             talk =  text_evaluate.substr (0,found_enter);
        #             text_evaluate = text_evaluate.substr (found_enter+3);
        #         }
        #         if (found_emotion!=std::string::npos && found_emotion<found_enter){
        #             talk =  text_evaluate.substr (0,found_emotion);
        #
        #             std::size_t found_space = text_evaluate.find(" ",found_emotion);
        #             if (found_space!=std::string::npos){
        #                 emo = text_evaluate.substr (found_emotion+1,found_space - found_emotion -1);
        #                 text_evaluate = text_evaluate.substr (found_space);
        #             }else{
        #                 emo = text_evaluate.substr (found_emotion+1);
        #                 text_evaluate = text_evaluate.substr (found_emotion);
        #                 end=true;
        #             }
        #             show_emotion(emo);
        #         }
        #         if (found_enter==std::string::npos && found_emotion==std::string::npos ) end=true;
        #         speech_text.request.data = talk;
        #         speech_serv_.call(speech_text);
        #     }
        # }


    def synthesize(self, text):
        
        try:
            rospy.loginfo("synthesizing text: %s" % text)
            self.robot.tts.say_with_gestures(text)
            rospy.sleep(0.5)
        except Exception, e:
            rospy.logwarn("TTS server failed!")

    def start_head(self):
        self.robot.basic_awareness.look_around()
        

    def stop_head(self):
        self.robot.basic_awareness.stop()
        self.robot.basic_motion.set_posture()

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
            elif msg.buttons[self.b_idx_start_head]:
                self.start_head()

            # stop head
            elif msg.buttons[self.b_idx_stop_head]:
                self.stop_head()
            
            return

if __name__ == '__main__':
    rospy.init_node('joy_tts')
    
    robot = robot_factory.build(["tts","basic_awareness","basic_motion"], core=False)
    JoystickTTS(robot)
    rospy.spin()
