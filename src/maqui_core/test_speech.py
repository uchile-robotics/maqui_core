#!/usr/bin/env python

#Basic ROS
import rospy

#Robot building
from maqui_core.tts import TTSSkill




if __name__ == '__main__':
    rospy.init_node('test')
    while not rospy.is_shutdown():
        tts.say("hello")
