#! /usr/bin/env python
from __future__ import print_function

import rospy
import actionlib
from exomy.msg import PilotExomyAction, PilotExomyGoal
from std_msgs.msg import Float32,Int32

from parse_content import ParseContentClass

config_filename = '../config/keyboard.txt'  # file with script

class ClientActionClass(object):

    def __init__(self):

        self.client = actionlib.SimpleActionClient('pilot_exomy', PilotExomyAction)
        self.client.wait_for_server()
        goal = PilotExomyGoal()

        self.parse_content_class = ParseContentClass()

        self.head = 0.0
        self.dist = 0.0

        # collect the command to be executed
        # open the sample file used
        file = open(config_filename)

        # read the content of the file opened
        content = file.readlines()
        last_line = len(content)
        file.close()

        self.active_line=0
        self.true_len = 0
        content2 = content

        # parse content to actual command primitives A,X,C
        content = self.parse_content_class.parse_it(all_content=content2)
        self.true_len = self.parse_content_class.new_len

        # content now contains all lines that are not comment
        #print(content)

        command = content[0]
        self.content = content[0:self.true_len]

        # command is now the first encountered line which is not a comment
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
        self.active_line = 1  # the next line to be sent later 

    def send_nextgoal(self):
        goal = PilotExomyGoal()
        command = self.content[self.active_line]
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
        self.active_line = self.active_line + 1  # the next line to be sent late]

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
        if self.active_line < self.true_len:
            self.send_nextgoal()


if __name__ == '__main__':

    try:
        rospy.init_node('auto_pilot')
        client = ClientActionClass()
        rospy.spin()
    except rospy.ROSInterruptException as e:
        rospy.loginfo("AutoPilot client interrupted before completion")
