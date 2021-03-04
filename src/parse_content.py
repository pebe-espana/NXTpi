#! /usr/bin/env python

class ParseContentClass(object):

    def __init__(self):
        self.new_len = 0

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

if __name__ == '__main__':
    parse_content_class = ParseContentClass()
