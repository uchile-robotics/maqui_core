#!/usr/bin/env python
# -*- coding: utf-8 -*-
import qi
import rospy 
from std_srv.srvs import Empty

class BasicMotionSkill(object):
    """
    """
    _type = "basic_motion"
    def __init__(self,robot):
        """
        Base Movement Skill
        """
        super(BasicMotionSkill, self).__init__()
        self._description = "skill to manage basic moves of maqui"
        self.robot = robot
        self._chains = ['Body','Legs', 'Arms', 'LArm', 'RArm','Head']

        self.memory = self.robot.session.service("ALMemory")
        self.motion = self.robot.session.service("ALMotion")
        self.posture = self.robot.session.service("ALRobotPosture")

        self.wake_up_srv = rospy.Service('wake_up', Empty, self.wake_up)
        self.rest_srv = rospy.Service('rest', Empty, self.rest)
        self.stop_posture_srv = rospy.Service('stop_posture', Empty, self.stop_posture)
        self.wake_up_srv = rospy.Service('wake_up', Empty, self.wake_up)
        self.wake_up_srv = rospy.Service('wake_up', Empty, self.wake_up)
        self.wake_up_srv = rospy.Service('wake_up', Empty, self.wake_up)
        self.wake_up_srv = rospy.Service('wake_up', Empty, self.wake_up)

        self.set_idle(bool = False)
        self.set_breathing(bool = False)


    def wake_up(self):
        
        """
        Turns the Stiffness of its motors on,
        Goes to a standing posture,
        Resumes its life
        """

        try:
            self.motion.wakeUp()
        except Exception as e:
            rospy.logerr(e)
        
        return

    def rest(self):
        """
        Goes to its resting posture,
        Turns the Stiffness of its motors off.

        """

        try:
            self.motion.rest()
        except Exception as e:
            rospy.logerr(e)
        return

    def robot_is_awake(self):
        try:
            result = self.motion.robotIsWakeUp()
        except Exception as e:
            rospy.logerr(e)
            return None
        return result

    def set_posture(self,posture = "Stand",speed = 0.8):
        
        if not posture in ["Stand", "StandZero","Crouch"]:
            return False
        try:
            self.posture.goToPosture(posture, speed) # block method
        except Exception as e:
            rospy.logerr(e)
            return False
        return True
    
    def stop_posture(self):

        try:
            self.posture.stopMove()
        except Exception as e:
            rospy.logerr(e)


    def set_breathing(self, chains = ["Body"], bool = True):

        for chain in chains:
            if chain in self._chains:
                try:
                    return self.motion.setBreathEnabled(chain, bool)

                except Exception as e:
                    rospy.logerr(e)
            else:
                rospy.logerr("Error setting Breathing: "+ chain + " is not a robot chain" )

    def set_idle(self, chains = ['Body','Legs', 'Arms', 'LArm', 'RArm','Head'] , bool = True):
        for chain in chains:
            if chain in self._chains:
            
                try:
                    self.motion.setIdlePostureEnabled(chain, bool)
                    return True
                except Exception as e:
                    rospy.logerr(e)
                    return False
            else:
                rospy.logerr("Error setting IDle: "+ chain + " is not a robot chain" )        
                return False

    def _set_security_shell(self,ss=True):
        
        msg ="" if ss else "des"
        try:
            self.motion.setExternalCollisionProtectionEnabled("Move",ss)

            print('security shell ' + msg+ 'activated')
            return True
        except Exception as e:
            rospy.logerr(e)
        
        return False

#    def _desactive_security_shell(self):
        

#        try:
#            self.motion.setExternalCollisionProtectionEnabled("Move",False)
#            print('security shell desactivated')

 #           return True
  #      except Exception as e:
 #           rospy.logerr(e)
        
#        return False




"""
Continue with Autonomous Life
"""