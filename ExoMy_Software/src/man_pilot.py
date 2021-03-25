#! /usr/bin/env python
#from __future__ import print_function

import rospy
import actionlib
from exomy.msg import PilotExomyAction, PilotExomyGoal

config_filename = '../config/keyboard.txt'  # file with script

class ClientActionClass(object):

    def __init__(self):

        self.client = actionlib.SimpleActionClient('pilot_exomy', PilotExomyAction)
        self.client.wait_for_server()
        goal = PilotExomyGoal()

        self.head = 0.0
        self.dist = 0.0

        # collect the command to be executed
        print('enter the command ( 2 and 4 spaces per entry):')
        print("mask           :-.-...-...-...")
        command = raw_input("mode-vel-deg-dur")

        print(command)
        goal.mode = command[0:2]   # 'A'
        goal.vel = int(command[2:6])    # 20
        goal.deg = int(command[6:10])   # 90
        goal.dur = int(command[10:14])  # 5
        print(goal)

        self.client.send_goal(goal, active_cb=self.active_cb,
                           feedback_cb=self.feedback_cb,
                           done_cb=self.done_cb)
        rospy.loginfo("Pilot goal has been sent to server")


    def active_cb(self):
        rospy.loginfo("PilotExomy server is processing goal")

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
        rospy.init_node('man_pilot')
        client = ClientActionClass()
        rospy.spin()
    except rospy.ROSInterruptException as e:
        print 'Something went wrong:', e
        rospy.loginfo("ManPilot client interrupted before completion")
