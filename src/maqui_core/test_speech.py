#!/usr/bin/env python

#Basic ROS
import rospy

#Robot building
from maqui_core.tts import TTSSkill
from maqui_core.audition import AuditionSkill




if __name__ == '__main__':
    rospy.init_node('test')
    tts = TTSSkill()
    audition = AuditionSkill()
    tts.setup()
    audition.setup()
    while not rospy.is_shutdown():
        tts.say(audition.recognize_with_grammar())
