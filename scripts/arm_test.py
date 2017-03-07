#!/usr/bin/env python
# -*- coding: utf-8 -*-

__author__ = "Rodrigo Muñoz"

import rospy
import tf
from bender_core.robot import Robot
# Robot hardware
from maqui_core.arm import LeftArm, RightArm

"""
Example of use of basic arm interfaces

With gazebo simulation:
$ roslaunch maqui_sim maqui.launch
"""
if __name__ == "__main__":
    rospy.init_node("arm_example")
    l_arm = LeftArm()
    r_arm = RightArm()
    # Run check
    rospy.sleep(0.5)
    if not l_arm.check() or not r_arm.check():
        raise RuntimeError("Arm don't available")
    # Run setup
    if not l_arm.setup() or not r_arm.setup():
        raise RuntimeError("Robot setup failed")
    # Send arm goal and wait
    l_arm.home()
    r_arm.home()
    l_arm.wait_for_motion_done()
    r_arm.wait_for_motion_done()
    rospy.loginfo(r_arm.get_result())
    # Send arm goal and wait
    l_arm.send_joint_goal([0.0,0.0,0.0,0.0,0.0,0.0], interval=2.0)
    r_arm.send_joint_goal([0.0,0.0,0.0,0.0,0.0,0.0], interval=2.0)
    l_arm.wait_for_motion_done()
    r_arm.wait_for_motion_done()
    rospy.loginfo(r_arm.get_result())
