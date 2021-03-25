#! /usr/bin/env python
from exomy.msg import PilotExomyAction, PilotExomyFeedback

class ParseContentClass(object):

    def __init__(self):
        # class parameters
        self.new_len = 0
        #self.sent_goal = PilotExomyGoal()
        # calibration factors
        self.deltatheta = -4.76/50.0   # degrees per 50ms cycle in X mode
        self.deltatheta = -0.1655
        self.deltar = 0.00046    # linear m per 50ms cycle % vel in A mode

    def parse_it(self,all_content):

        # first order parsing : remove all comment lines #---

        last_line = len(all_content)
        self.content = all_content   # just a type definition

        for i in range (0,last_line):
            command = all_content[i]
            if command[0] != '#' :
                self.content[self.new_len]= all_content[i]
                self.new_len = self.new_len+1

        # content now contains all lines that are not comment
        #print(self.content)


        return self.content

    def expected_move(self,sent_goal,headstart,diststart,cycl):
        # calculates the expected motion per command cycle
        self.fback = PilotExomyFeedback()  # just a type definition 
        #print("in expected move")
        #print(sent_goal)
        self.fback.cycl = cycl
        self.fback.diag = "analysed"
        turnsign = +1
        if sent_goal.vel < 0:
            turnsign = -1
        if sent_goal.mode == 'X ':
            self.fback.head = headstart + cycl * self.deltatheta * sent_goal.vel
            self.fback.dist = diststart
        if sent_goal.mode == 'A ':     # assuming straight moves only at present
            self.fback.head = headstart
            self.fback.dist = diststart + cycl * self.deltar * sent_goal.vel
        if sent_goal.mode == 'C ':
            self.fback.head = headstart
            self.fback.dist = diststart
        if sent_goal.mode == 'X':
            self.fback.head = headstart + cycl * self.deltatheta * sent_goal.vel
            self.fback.dist = diststart
        if sent_goal.mode == 'A':
            self.fback.head = headstart
            self.fback.dist = diststart + cycl * self.deltar * sent_goal.vel
        if sent_goal.mode == 'C':
            self.fback.head = headstart
            self.fback.dist = diststart + cycl * self.deltar * 50
        #print(self.fback)
        return self.fback

if __name__ == '__main__':
    parse_content_class = ParseContentClass()
