#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

from std_srvs.srv import Empty, EmptyResponse
from naoqi_bridge_msgs.msg import WordRecognized

class AuditionSkill(object):
    """
    The TTSSkill

    TODO
    """
    _type = "tts_old"

    def __init__(self):
        self._description = "the tts skill"

        self._start_topic = "/start_recognition"
        self._stop_topic = "/stop_recognition"
        self._word_subscriber_topic = "/word_recognized"

        self._start_client = None
        self._stop_client = None
        self._word_subscriber = None

        self._last_word_recognized = ''

        self._recognition_started = False

        self._rate = rospy.Rate(10)


        
    def check(self, timeout = 1.0):
        # rospy.loginfo("{skill: %s}: check()." % self._type)

        try:
            rospy.wait_for_service(self._start_topic, timeout)
        except rospy.ROSException:

            return False

        try:
            rospy.wait_for_service(self._set_stop_topic, timeout)
        except rospy.ROSException:

            return False

        return True
    
    def setup(self):
        # rospy.loginfo("{skill: %s}: setup()." % self._type)
        self._start_client = rospy.ServiceProxy(self._start_topic, Empty)
        self._stop_client = rospy.ServiceProxy(self._stop_topic, Empty)
        self._word_subscriber = rospy.Subscriber(self._word_subscriber_topic, WordRecognized, self.wordCallback)
        return True

    def shutdown(self):
        rospy.loginfo("{skill: %s}: shutdown()." % self._type)
        return True

    def start(self):
        rospy.loginfo("{skill: %s}: start()." % self._type)
        return True

    def pause(self):
        rospy.loginfo("{skill: %s}: pause()." % self._type)
        self.stop()
        return True

    def recognize_with_grammar(self, dictionary="gpsr", timeout=10):
        """
        Send text to the tts system to speak it out loud

        Args:
            text (String): Text to synthesize.  Default: "bender yes"

        Examples:
            >>> robot.tts.say("Hello, my name is bender")
        """
        try:
            self._start_client()
            self._recognition_started = True
        except rospy.ROSException:

            return False
        while self._recognition_started and not rospy.is_shutdown():
            self._rate.sleep()
        if self._recognition_started:
            self._stop_client()

        return self._last_word_recognized


    def set_language(self, language="english"):
        """
        Set the language for tts system. Available languages: english/spanish

        Args:
            language (string): Bender's language. Default: "english"

        Examples:
            >>> robot.tts.set_language("spanish")
        """

        return True

    def stop(self):
        """
        Stop speech audio stream

        Args:
            None

        Examples:
            >>> robot.tts.stop()
        """
        try:
            self._stop_client()
        except rospy.ROSException:
            return False

        return True

    def wordCallback(self,msg):
        self._last_word_recognized = msg.words[0]
        if self._recognition_started:
            self._stop_client()
            self._recognition_started = False

if __name__ == "__main__":
    rospy.init_node("test_tts")
    tts = AuditionSkill()
    tts.setup()
    print tts.recognize_with_grammar()