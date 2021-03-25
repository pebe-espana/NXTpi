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
from exomy.msg import PilotExomyAction,PilotExomyFeedback
from exomy.msg import PilotExomyResult
from std_msgs.msg import Float32,Int32

# define the action class

class PilotExomy_ActionServer(object):

    def __init__(self, name):

        self.action_name = name
        self.locomotion_mode = LocomotionMode.ACKERMANN.value
        self.motors_enabled = True

        # sensor data from NXT (if available)
        self.cmps = 720.0  #Float32
        self.sonar = 512   #Int32
        #calculated achieved goal
        self.dist = 512
        self.head = 720.0
        self.distsign = 1
        self.degsign = 1

        self.a_server = actionlib.SimpleActionServer(
            self.action_name, PilotExomyAction, execute_cb=self.execute_cb, auto_start=False)
        self.pub = rospy.Publisher('/rover_command',RoverCommand, queue_size=1)
        rospy.Subscriber("NXTcmps", Float32, self.callbackcmps)
        rospy.Subscriber("NXTsonar",Int32,self.callbacksonar)
        rospy.loginfo('pilot_exomy server  started')
        self.a_server.start()

    def callbackcmps(self,data):
        heard_str = "nxt cmps heard at %s" % rospy.get_time()
        rospy.loginfo('heard_str')
        self.cmps = data.data

    def callbacksonar(self,data):
        heard_str = "nxt sonar heard at %s" % rospy.get_time()
        rospy.loginfo('heard_str')
        self.sonar = data.data

    def execute_cb(self, goal):

        # goal of action
        print('command issued = ' + goal.mode + ' ' + str(goal.vel) +' ' + str(goal.deg) + ' ' + str(goal.dur))

        rover_cmd = RoverCommand()

        if (goal.mode =='A'):
            self.locomotion_mode = LocomotionMode.ACKERMANN.value
        if (goal.mode =='X'):
            self.locomotion_mode = LocomotionMode.POINT_TURN.value
            if goal.deg <90 :
                degsign = -1
            if goal.deg >90 :
                degsign = 1
        if (goal.mode =='C'):
            self.locomotion_mode = LocomotionMode.CRABBING.value

        rover_cmd.locomotion_mode = self.locomotion_mode

        rover_cmd.vel = goal.vel
        rover_cmd.steering = goal.deg
        rover_cmd.connected = True
        rover_cmd.motors_enabled = True

        success = True   # rest to false when interrupted
        diag = ''
        feedback = PilotExomyFeedback()
        result = PilotExomyResult()
        rate = rospy.Rate(20)

        waitdur = 40   # 20 = 1sec extra wait for cmps or sonar
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
            feedback.head = self.cmps
            feedback.dist = self.sonar
            feedback.diag = "runtime"

            self.a_server.publish_feedback(feedback)
            rate.sleep()

        rover_cmd.vel = 0
        rover_cmd.steering = 90
        rover_cmd.connected = True
        rover_cmd.motors_enabled = False
        self.pub.publish(rover_cmd)

        for j in range(0, waitdur):
            # wait for NXT data
            feedback.cycl = j + goal.dur
            feedback.head = self.cmps
            feedback.dist = self.sonar
            feedback.diag = "waiting"
            rate.sleep()

        # if nothing resets success
        if success:
            result.head = self.cmps
            result.dist = self.sonar
            result.diag = feedback.diag    # the last feedback counts still
            self.a_server.set_succeeded(result)


if __name__ == "__main__":
    rospy.init_node("pilot_exomy")
    s = PilotExomy_ActionServer(rospy.get_name())
    rospy.spin()
