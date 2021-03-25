#!/usr/bin/env python
from __future__ import print_function

import rospy
from exomy.msg import RoverCommand
from locomotion_modes import LocomotionMode
import math
import time

from exomy.srv import AutoJoy

# Define locomotion modes
global locomotion_mode
global motors_enabled

locomotion_mode = LocomotionMode.ACKERMANN.value
motors_enabled = True

def handler_autojoy(req):
    global locomotion_mode
    global motors_enabled

    print("activating rover by script")

    rover_cmd = RoverCommand()

    # req.mode = A,X,C
    if (req.mode =='A'):
        locomotion_mode = LocomotionMode.ACKERMANN.value
    if (req.mode =='X'):
        locomotion_mode = LocomotionMode.POINT_TURN.value
    if (req.mode =='C'):
        locomotion_mode = LocomotionMode.CRABBING.value

    rover_cmd.locomotion_mode = locomotion_mode

    rover_cmd.vel = req.vel
    rover_cmd.steering = req.deg
    rover_cmd.connected = True
    rover_cmd.motors_enabled = True

    r = rospy.Rate(20)  #10Hz
    timer = 0
    while (timer < req.dur):
        pub.publish(rover_cmd)
        timer = timer + 1
        r.sleep()
    # time is up
    rover_cmd.vel = 0
    rover_cmd.steering = 90
    rover_cmd.connected = True
    rover_cmd.motors_enabled = False
    pub.publish(rover_cmd)
    outcome = "move completed " + str(timer)

    return outcome


def autojoy_server():
    global pub

    rospy.init_node('autojoy_server')
    rospy.loginfo('autojoy_server started')

    pub = rospy.Publisher('/rover_command',RoverCommand, queue_size=1)
    s = rospy.Service('autojoy',AutoJoy,handler_autojoy)
    print("ready to activate Joystick run")
    rospy.spin()

if __name__ == '__main__':
    autojoy_server()
