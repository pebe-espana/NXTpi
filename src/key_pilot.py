#! /usr/bin/env python
#from __future__ import print_function

import rospy
import actionlib
from exomy.msg import PilotExomyAction, PilotExomyGoal

class ClientActionClass(object):

    def __init__(self):

        self.client = actionlib.SimpleActionClient('pilot_exomy', PilotExomyAction)

        self.client.wait_for_server()

        goal = PilotExomyGoal()

        file = open('keyboard.txt')
        content = file.readlines()
        last_line = len(content)
        command = content[last_line-1]

        goal.mode = command[0:3]   # 'A'
        goal.vel = int(command[4:7])    # 20
        goal.deg = int(command[8:11])   # 90
        goal.dur = int(command[12:15])  # 5

        self.client.send_goal(goal, active_cb=self.active_cb,
                           feedback_cb=self.feedback_cb,
                           done_cb=self.done_cb)
        rospy.loginfo("Pilot goal has been sent to server")

    def active_cb(self):
        rospy.loginfo("KeyPilot server is processing command to rover")

    def feedback_cb(self,feedback):
        rospy.loginfo("feedback: %s" % str(feedback))
        # analyse feedback
        if feedback.cycl == 99 :
            self.client.cancel_goal()

    def done_cb(self, state, result):
        rospy.loginfo("Action server has completed goal state: %s, result: %s"
                   % (str(state),str(result)))


if __name__ == '__main__':

    try:
        rospy.init_node('key_pilot')
        client = ClientActionClass()
        rospy.spin()
    except rospy.ROSInterruptException as e:
        print 'Something went wrong:', e
        rospy.loginfo("KeyPilot server interrupted before completion")
