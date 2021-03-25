#!/usr/bin/env python

# this node is to test extending ROS beyond the container network


import rospy
from std_msgs.msg import String

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + " I heard %s", data.data)
    #pub = rospy.Publisher('recvd_nxt', String, queue_size=10)
    # following the model of ROS2 tutorial, the definition of pub is not in callback but in node definition already
    heard_str = "nxt heard at %s" % rospy.get_time()
    rospy.loginfo(heard_str)
    pub.publish(heard_str)

def listen4nxt():
    global pub

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously
    rospy.init_node('listen4nxt', anonymous=True)
    pub = rospy.Publisher('recvd_nxt', String, queue_size=10)
    rospy.Subscriber("NXTready", String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listen4nxt()
