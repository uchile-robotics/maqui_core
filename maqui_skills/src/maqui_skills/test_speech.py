#!/usr/bin/env python

#Basic ROS
import rospy

#Robot building
from tts import TTSSkill
from audition import AuditionSkill




if __name__ == '__main__':
    rospy.init_node('test')
    tts = TTSSkill()
    audition = AuditionSkill()
    tts.setup()
    audition.setup()
    while not rospy.is_shutdown():
        tts.say(audition.recognize_with_grammar())
