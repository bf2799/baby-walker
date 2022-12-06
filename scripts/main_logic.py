#!/usr/bin/env python
# -*- coding: utf-8 -*-

from sqlite3 import Timestamp
from time import time
from numpy import float32, float64
import rospy
import serial
from math import sin, cos, pi
from std_msgs.msg import Float64, Float32, Bool
# from sensor_msgs.msg import Imu
# from sensor_msgs.msg import MagneticField

def callback_mag(data):
    rospy.loginfo(data)

def callback_hall(data):
    rospy.loginfo(data)    

if __name__ == '__main__':
    rospy.init_node('main_node', anonymous=True)

    rate = rospy.Rate(2) # 40hz
    rospy.loginfo("Initialization complete")

    rospy.Subscriber('hall_info', Bool, callback_hall)
    pub_servo = rospy.Publisher('servo_info', Float32, queue_size=10)
    pub_buzz = rospy.Publisher('buzz_info', Bool, queue_size=10)

    try:
        while not rospy.is_shutdown():
            pos = 1.0
            on_off = 0
            buzz_rate = 1

            rospy.loginfo(rospy.get_time())

            pub_servo.publish(pos)
            pub_buzz.publish(on_off)

            # imu_str = port.readline()
            # rospy.loginfo(imu_str)
            # line = str(imu_str, 'UTF-8')
            # if line.startswith('$VNYMR'):
            #     #rospy.loginfo("Find: "+line)
            #     try: mylist = line[7:].split(',')
            #     except:
            #         rospy.loginfo("Data exception: "+line)
            #         continue

            #     Yaw = mylist[0] #Yaw
            #     Pitch = mylist[1] #Pitch
            #     Roll = mylist[2] #Roll
                
            #     try:
            #         cy = cos(float(Yaw) * 0.5)
            #         sy = sin(float(Yaw) * 0.5)
            #         cp = cos(float(Pitch) * 0.5)
            #         sp = sin(float(Pitch) * 0.5)
            #         cr = cos(float(Roll) * 0.5)
            #         sr = sin(float(Roll) * 0.5)
            #     except:
            #         rospy.loginfo("Data Wrong: "+line)
            #         continue

            #     imu.header.seq = 0
            #     imu.header.stamp = rospy.Time.now()
            #     imu.header.frame_id = 'IMU'
            #     imu.orientation.w = cr * cp * cy + sr * sp * sy
            #     imu.orientation.x = sr * cp * cy - cr * sp * sy
            #     imu.orientation.y = cr * sp * cy + sr * cp * sy
            #     imu.orientation.z = cr * cp * sy - sr * sp * cy

            #     imu.orientation_covariance[0] = 0 #-1.0

            #     imu.linear_acceleration.x = float(mylist[6])
            #     imu.linear_acceleration.y = float(mylist[7])
            #     imu.linear_acceleration.z = float(mylist[8])

            #     imu.angular_velocity.x = float(mylist[9])
            #     imu.angular_velocity.y = float(mylist[10])
            #     imu.angular_velocity.z = float(mylist[11].split('*')[0])

            #     mag.header.seq = 0
            #     mag.header.stamp = rospy.Time.now()
            #     mag.header.frame_id = 'MAG'
            #     mag.magnetic_field.x = float(mylist[3])
            #     mag.magnetic_field.y = float(mylist[4])
            #     mag.magnetic_field.z = float(mylist[5])
 
            #     rospy.loginfo(imu)
            #     pub1.publish(imu)

            rate.sleep()

    except rospy.ROSInterruptException:
        rospy.loginfo("SROSInterruptException...")
        #port.close()
    
    except serial.serialutil.SerialException:
        rospy.loginfo("Shutting down paro_depth node...")

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
