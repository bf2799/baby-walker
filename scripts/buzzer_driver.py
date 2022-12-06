#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool, Float64

def callback_buzz(data):
    rospy.loginfo(data)

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('buzz_listener', anonymous=True)

    rospy.Subscriber('buzz_info', Bool, callback_buzz)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
