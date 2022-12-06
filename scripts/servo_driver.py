#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32, Bool

def callback_servo(data):
    rospy.loginfo(data)

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('servo_listener', anonymous=True)

    rospy.Subscriber('servo_info', Float32, callback_servo)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
