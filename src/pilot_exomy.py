#! /usr/bin/env python
from __future__ import print_function

import rospy
import actionlib

# import message structures for driving motors
from exomy.msg import RoverCommand
from locomotion_modes import LocomotionMode
import math
import time

# import the action server parts
from exomy.msg import PilotExomyAction
from exomy.msg import PilotExomyGoal,PilotExomyFeedback
from exomy.msg import PilotExomyResult
from std_msgs.msg import Float32,Int32

# define the action class

class PilotExomy_ActionServer(object):

    def __init__(self, name):

        self.action_name = name
        self.locomotion_mode = LocomotionMode.ACKERMANN.value
        self.motors_enabled = True

        self.a_server = actionlib.SimpleActionServer(
            self.action_name, PilotExomyAction, execute_cb=self.execute_cb, auto_start=False)
        self.pub = rospy.Publisher('/rover_command',RoverCommand, queue_size=1)
        rospy.loginfo('pilot_exomy server  started')
        self.a_server.start()

    def execute_cb(self, goal):

        # goal of action
        print('command issued = ' + goal.mode + ' ' + str(goal.vel) +' ' + str(goal.deg) + ' ' + str(goal.dur))

        rover_cmd = RoverCommand()

        if (goal.mode =='A'):
            self.locomotion_mode = LocomotionMode.ACKERMANN.value
        if (goal.mode =='X'):
            self.locomotion_mode = LocomotionMode.POINT_TURN.value
        if (goal.mode =='C'):
            self.locomotion_mode = LocomotionMode.CRABBING.value
        if (goal.mode =='A '):
            self.locomotion_mode = LocomotionMode.ACKERMANN.value
        if (goal.mode =='X '):
            self.locomotion_mode = LocomotionMode.POINT_TURN.value
        if (goal.mode =='C '):
            self.locomotion_mode = LocomotionMode.CRABBING.value

        rover_cmd.locomotion_mode = self.locomotion_mode

        rover_cmd.vel = goal.vel
        rover_cmd.steering = goal.deg
        rover_cmd.connected = True
        rover_cmd.motors_enabled = True

        success = True   # rest to false when interrupted
        feedback = PilotExomyFeedback()
        result = PilotExomyResult()
        rate = rospy.Rate(20)

        for i in range(0, goal.dur):
            if self.a_server.is_preempt_requested():
                self.a_server.set_preempted()
                success = False
                # reset the commands to motors
                rover_cmd.vel = 0
                rover_cmd.steering = 90
                rover_cmd.connected = True
                rover_cmd.motors_enabled = False
                self.pub.publish(rover_cmd)
                break

            self.pub.publish(rover_cmd)
            # still within motor turning part
            feedback.cycl = i
            feedback.diag = "runcycle"

            self.a_server.publish_feedback(feedback)
            rate.sleep()

        rover_cmd.vel = 0
        rover_cmd.steering = 90
        rover_cmd.connected = True
        rover_cmd.motors_enabled = False
        self.pub.publish(rover_cmd)


        # if nothing resets success
        if success:
            result.diag = "done"    
            self.a_server.set_succeeded(result)


if __name__ == "__main__":
    rospy.init_node("pilot_exomy")
    s = PilotExomy_ActionServer(rospy.get_name())
    rospy.spin()
