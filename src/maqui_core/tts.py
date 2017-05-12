#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Speech recognition
"""
__author__ = "Cristopher Gomez" 
####################################################################
#To use this skill run $ roslaunch bender_speech recognizer.launch
#and to test, you have to run $ rosrun bender_skills listener_robot.py
#
#                                                                   
#####################################################################

# ROS Core
import rospy
import actionlib
# ROS Messages
#from bender_speech.msg import DoRecognitionAction, DoRecognitionGoal
# Robot skill
from std_msgs.msg import( String )
from std_srvs.srv import( Empty, EmptyResponse )
from naoqi_bridge_msgs.msg import(
    SpeechWithFeedbackGoal,
    SpeechWithFeedbackResult,
    SpeechWithFeedbackFeedback,
    SpeechWithFeedbackAction )

class TTSSkill(object):
    """
    Base class for speech recognition using speech recognition action server
    """

    _type = "tts"


    def __init__(self):
        """
        Base class for speech recognition using speech recognition action server

        Args:
            None
        Raises:
            None
        """
        self._description = "Speech recognition using command action"
      
  
        # Speech recognition action topic
        self._sra_topic = "speech_action"
        # ROS clients (avoid linter warnings)
        self._sra_client = None

    def check(self, timeout=1.0):
        # Check client for speech recognition action (SRA)
        sra_client = actionlib.SimpleActionClient(self._sra_topic, SpeechWithFeedbackAction)
        # Wait for the sra server to start or exit
        if not sra_client.wait_for_server(timeout=rospy.Duration(timeout)):
            return False
        return True

    def setup(self):
        # Speech recognition action (sra)
        self._sra_client = actionlib.SimpleActionClient(self._sra_topic, SpeechWithFeedbackAction)
        self._sra_client.wait_for_server()
        rospy.sleep(0.1)
        return True

    def shutdown(self):
       # Cancel goals
        self._sra_client.cancel_all_goals()
        return True

    def start(self):
  
        return True

    def pause(self):

        return True

    # Speech recognition related methods
    def say(self, word="hi",timeout=16.0):
        """
        Blocking
        Send speech recognition command to start the recognition.  It return a string of the recognized sentence. 
        If doesn't recognizes anything it returns a empty string

        Args:
            dictionary (string): Name of the grammar file to recognize with.  Default = Stage1/Stage2gpsr
            timeout (number): Time (seconds) to wait for recognizer.  Default = 16.0
        Returns:


        Examples:
            >>> audition.recognize_with_grammar("Stage1/Stage2gpsr")
        """
        self.send_goal(word)
  
        return self.wait_for_result(timeout)

    def recognize(self, dictionary="Stage1/Stage2gpsr"):
        """
        Send a goal to speech recognition server.  When a result is received
        """

    def done_cb(self, state, result):
        """
        Callback 
        """

      
    def send_goal(self, word="hi"):
        """
        Send goal to action server of speech recognition

        Args:
            dictionary (string): Name of the grammar file to recognize with.  Default = Stage1/Stage2gpsr
        """

        goal = SpeechWithFeedbackGoal(say=word)

        
        self._sra_client.send_goal(goal)

    def wait_for_result(self,timeout = 3.0):
        """
        Blocks until the recognition is done or until the timeout.  

        Returns:
        True if the goal finished. False if the goal didn't finish within the allocated timeout 
        
        Args:
            timeout (number): Time (seconds) to wait for recognizer.  Default = 3.0

        """
        return self._sra_client.wait_for_result(rospy.Duration(timeout))

    def get_result(self):
        """
        Gets result of the current goal

        Returns:
        String with the recognized sentence

        Args:
            None
        """

        return self._sra_client.get_result()
 
if __name__ == "__main__":
    rospy.init_node("test_tts")
    tts = TTSSkill()
    tts.setup()
    tts.say("ki pa")