#! /usr/bin/env python
from __future__ import print_function

import rospy
import actionlib
from exomy.msg import PilotExomyAction, PilotExomyGoal, PilotExomyFeedback,PilotExomyResult
from std_msgs.msg import Float32,Int32,Float64

from parse_content import ParseContentClass

config_filename = '../config/task_list.txt'  # file with location of script

class ClientActionClass(object):

    def __init__(self):

        # define subscriptions and publications
        rospy.Subscriber('NXTcmps',Float32, self.callbackcmps)
        rospy.Subscriber('NXTsonar',Int32, self.callbacksonar)

        self.mov_pub_dist = rospy.Publisher('APdist',Float64, queue_size=10)
        self.mov_pub_head = rospy.Publisher('APhead',Float64, queue_size=10)
        self.mov_pub_obst = rospy.Publisher('APobst',Float64, queue_size=10)

	self.sensed = False     # flag to denote that real world sensor data has been acquired previously
        self.delay = False      # flag to force 3 sensor readings between task lines
        self.has_run = False    # flag to denote state that auto_pilot has started to move already

        self.client = actionlib.SimpleActionClient('pilot_exomy', PilotExomyAction)
        self.client.wait_for_server()

        self.goal = PilotExomyGoal()
        self.parse_content_class = ParseContentClass()

        self.cmps = 0.0     # 720.0 for recognisable default compass
        self.sonar = 256      # recognisable default sonar
        self.head = 0.0         # heading and distance have meaning once has_run is true
        self.dist = 0.0
        self.obst = 255.0   # distance to any obstacle at 90 degrees (=front)

        # -----  collect the command to be executed
        # open the sample file used
        file = open(config_filename)
        task_name_details = file.readlines()
        file.close()

        # use the task details read
        task_name = task_name_details[0]
        task_name = task_name[0:len(task_name)-1]
        delay = task_name_details[1]
        delay = delay[0:len(delay)-1]
        if delay == 'delay ON':
            self.delay = True
            print(' delay is ON ')
        file = open(task_name)
        # read the content of the task file opened
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
        #print(command)
        self.goal.mode = command[0:2]   # 'A'
        self.goal.vel = int(command[2:6])    # 20
        self.goal.deg = int(command[6:10])   # 90
        self.goal.dur = int(command[10:14])  # 5
        #print(self.goal)

        # start sending goals only once sensor data is available or if timed out
        rate = rospy.Rate(20)
        for i in range (0, 20):   # 60 cycle of 50ms = 1 sec max
	    if self.sensed:
                rospy.loginfo("auto-pilot acquired sensor status after " + str(i))
                break
            rate.sleep()

        if self.delay:
            self.sensed = False
            for i in range (0, 20):   # 60 cycle of 50ms = 1 sec max
                if self.sensed:
                    rospy.loginfo("auto-pilot acquired 2nd sensor status after " + str(i))
                    break
                rate.sleep()

            self.sensed = False
            for i in range (0, 20):   # 60 cycle of 50ms = 1 sec max
                if self.sensed:
                    rospy.loginfo("auto-pilot acquired 3rd sensor status after " + str(i))
                    break
                rate.sleep()

        # can now send a goal as starting status is known
        self.headstart = self.cmps    # if no compass, assume facing N = 0
        self.diststart = self.dist
        if self.sonar < 60 :
            self.obst = self.sonar  # sonar only valid below 60 cm
        self.client.send_goal(self.goal, active_cb=self.active_cb,
                           feedback_cb=self.feedback_cb,
                           done_cb=self.done_cb)
        rospy.loginfo("Pilot goal has been sent %s" % str(self.goal.mode))
        rospy.loginfo("Pilot goal vel %s" % str(self.goal.vel))
        rospy.loginfo("Pilot goal deg %s" % str(self.goal.deg))
        rospy.loginfo("Pilot goal dur %s" % str(self.goal.dur))
        self.active_line = 1  # the next line to be sent later

    def callbackcmps(self,msg) :
        self.cmps = msg.data
        rospy.loginfo("NXTcmps: " + str(msg.data))
        if (self.has_run and self.sensed) :
            # sensor data and move data available, so can publish moves
            self.mov_pub_head.publish(self.head)
            self.mov_pub_dist.publish(self.dist)
            rospy.loginfo('published dist %s ' % str(self.dist))
            rospy.loginfo('published head %s ' % str(self.head))
        self.sensed = True

    def callbacksonar(self,msg) :
        self.sonar = msg.data
        rospy.loginfo('NXTsonar: %s ' % str(self.sonar))
        if self.sonar < 60 :
            self.obst = self.sonar  # sonar only valid below 60 cm
            self.mov_pub_dist.publish(self.obst)
            rospy.loginfo('published obst %s ' % str(self.obst))
        self.sensed = True



    def send_nextgoal(self):
        command = self.content[self.active_line]

        # check if this command is a wait cycle
        if command[0:2] == 'W ' :
            waitCycl = int(command[2:6])
            rate = rospy.Rate(20)
            for i in range (0, waitCycl):   # 50ms per cycle
                rate.sleep()
            self.active_line = self.active_line + 1
            command = self.content[self.active_line]
        # wait for next compass update
        rate = rospy.Rate(20)
        self.sensed = False
        for i in range (0, 20):   # 60 cycle of 50ms = 1 sec max
            if self.sensed:
                rospy.loginfo("auto-pilot acquired sensor status after " + str(i))
                break
            rate.sleep()

        if self.delay:
            self.sensed = False
            for i in range (0, 20):   # 60 cycle of 50ms = 1 sec max
                if self.sensed:
                    rospy.loginfo("auto-pilot acquired 2nd sensor status after " + str(i))
                    break
                rate.sleep()

            self.sensed = False
            for i in range (0, 20):   # 60 cycle of 50ms = 1 sec max
                if self.sensed:
                    rospy.loginfo("auto-pilot acquired 3rd sensor status after " +  str(i))
                    break
                rate.sleep()

        #print(command)
        self.goal.mode = command[0:2]   # 'A'
        self.goal.vel = int(command[2:6])    # 20
        self.goal.deg = int(command[6:10])   # 90
        self.goal.dur = int(command[10:14])  # 5
        #print(self.goal)

        # if there is no compass, then achieved heading is not challenged
        if self.sensed:
           self.head = self.cmps # could substitute true heading
        self.headstart = self.head
        self.diststart = self.dist
        if self.sonar < 60 :
            self.distobst = self.sonar  # sonar only valid below 60 cm

        self.client.send_goal(self.goal, active_cb=self.active_cb,
                           feedback_cb=self.feedback_cb,
                           done_cb=self.done_cb)
        rospy.loginfo("Pilot goal has been sent %s" % str(self.goal.mode))
        rospy.loginfo("Pilot goal vel %s" % str(self.goal.vel))
        rospy.loginfo("Pilot goal deg %s" % str(self.goal.deg))
        rospy.loginfo("Pilot goal dur %s" % str(self.goal.dur))
        self.active_line = self.active_line + 1  # the next line to be sent late]

    def active_cb(self):
        rospy.loginfo("PilotExomy server processing goal %s"
                            % str(self.active_line))


    def feedback_cb(self,feedback):
        #rospy.loginfo("raw feedback: %s" % str(feedback))
        # analyse feedback
        #self.head = self.cmps
        #self.dist = self.dist
        fb = PilotExomyFeedback()
        fb = self.parse_content_class.expected_move(
                sent_goal = self.goal, headstart = self.headstart, diststart = self.diststart, cycl = feedback.cycl)
        rospy.loginfo("analysed feedback head: %s" % str(fb.head))
        rospy.loginfo("analysed feedback dist: %s" % str(fb.dist))
        self.has_run = True
        self.head = fb.head
        self.dist = fb.dist
        # just a nota bene how to cancel an ongoing action as result of feedback - no expected use yet
        if feedback.cycl == 999 :
            self.client.cancel_goal()

    def done_cb(self, state, result):
        rospy.loginfo("Action server has completed goal state: %s "
                   % str(state))
        res = PilotExomyResult()
        res = self.parse_content_class.expected_move(
                sent_goal = self.goal, headstart = self.headstart, diststart = self.diststart, cycl = result.cycl)
        self.head = res.head
        self.dist = res.dist
        # go on to next goal
        if self.active_line < self.true_len:
            self.send_nextgoal()


if __name__ == '__main__':

    try:
        rospy.init_node('auto_pilot')
        client = ClientActionClass()
        rospy.spin()
    except rospy.ROSInterruptException as e:
        rospy.loginfo("AutoPilot client interrupted before completion")
