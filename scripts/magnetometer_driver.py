#!/usr/bin/env python
# -*- coding: utf-8 -*-

from sqlite3 import Timestamp
from time import time
from numpy import float32, float64
import rospy
import serial
from math import sin, cos, pi
from std_msgs.msg import String
from sensor_msgs.msg import MagneticField

if __name__ == '__main__':
    pub1 = rospy.Publisher('mag_info', String, queue_size=10)

    rospy.init_node('mag_talker', anonymous=True)
    rate = rospy.Rate(1) # 40hz
    # serial_port = rospy.get_param('~port','/dev/ttyUSB0')
    # serial_baud = rospy.get_param('~baudrate',115200)
    # port = serial.Serial(serial_port, serial_baud, timeout=3.)
    rospy.loginfo("Initialization complete")
    # port.write('$VNWRG,07,40*XX'.encode())  # cmd from 01 to 00 to sample continuously  
    # imu = Imu()
    # mag = MagneticField()

    try:
        while not rospy.is_shutdown():
            hello_str = "Hellow from Magnetometer!" # % rospy.get_time()
            rospy.loginfo(hello_str)
            pub1.publish(hello_str)

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



#     SENSOR_NAME = "paro"
#     rospy.init_node('depth_paro')
#     serial_port = rospy.get_param('~port','/dev/ttyUSB0')
#     serial_baud = rospy.get_param('~baudrate',115200)
#     sampling_rate = rospy.get_param('~sampling_rate',5.0)
#     offset = rospy.get_param('~atm_offset',12.121) # in meter ??
#     latitude_deg = rospy.get_param('~latitude',41.526)  # deg 41.526 N is Woods Hole
  
#     port = serial.Serial(serial_port, serial_baud, timeout=3.)
#     rospy.logdebug("Using depth sensor on port "+serial_port+" at "+str(serial_baud))
#     rospy.logdebug("Using latitude = "+str(latitude_deg)+" & atmosphere offset = "+str(offset))
#     rospy.logdebug("Initializing sensor with *0100P4\\r\\n ...")
    
#     sampling_count = int(round(1/(sampling_rate*0.007913)))
#     port_string_data='*0100EW*0100PR='+str(sampling_count)+'\r\n'
#     port.write(port_string_data.encode())
#     #port.write('*0100EW*0100PR='+str(sampling_count)+'\r\n')  # cmd from 01 to 00 to set sampling period
#     rospy.sleep(0.2)
#     line = port.readline()
#     port.write('*0100P4\r\n'.encode())  # cmd from 01 to 00 to sample continuously
    
    
#     #publish
#     pub = rospy.Publisher('topicMessage', String, queue_size = 10)
#     msg = String()

#     rospy.loginfo("Initialization complete")    
#     rospy.loginfo("Publishing pressure and depth.")
    
#     odom_msg = Odometry()
#     odom_msg.header.frame_id = "odom"
#     odom_msg.child_frame_id = SENSOR_NAME
#     odom_msg.header.seq=0
    
#     sleep_time = 1/sampling_rate - 0.025
   
#     try:
#         while not rospy.is_shutdown():
#             line = port.readline()
#             #line.decode("utf-8")
#             line = str(line, 'UTF-8')
#             #rospy.loginfo(line)
#             print(line)
#             if line == '':
#                 rospy.loginfo("DEPTH: No data")
#             else:
#                 if line.startswith('$GPGGA'):
#                     #rospy.loginfo(line)
#                     rospy.loginfo("Got you! "+line)
#                     odom_msg.header.stamp=rospy.Time.now()
#                     try: mylist = line[7:].split(',')
#                     except:
#                         rospy.loginfo("Data exception: "+line)
#                         continue
#                     rospy.loginfo(mylist)
                    
#                     latitude_dd = 0 if mylist[1][0:2] == '' else int(mylist[1][0:2]) 
#                     latitude_mm = 0 if mylist[1][2:] == '' else float(mylist[1][2:]) / 60
#                     longitude_ddd = 0 if mylist[3][0:3] == '' else mylist[3][0:3]
#                     longitude_mmm = 0 if mylist[3][3:] == '' else float(mylist[3][3:]) / 60
#                     altitude = 0 if mylist[8] == '' else float(mylist[8])

#                     if longitude_ddd[0] == '0':
#                         longitude_ddd = int(longitude_ddd[1:3]) * (-1)
#                         longitude_mmm = longitude_mmm * (-1)
#                     else:
#                         longitude_ddd = int(longitude_ddd[:3])

#                     latitude = float(latitude_dd) + latitude_mm
#                     longitude = float(longitude_ddd) + longitude_mmm

#                     UTMdata = utm.from_latlon(latitude, longitude)
#                     rospy.loginfo(UTMdata)

#                     msg.header="Message Header"
#                     msg.latitude = latitude
#                     msg.longitude = longitude
#                     msg.altitude = altitude
#                     msg.easting = UTMdata[0]
#                     msg.northing = UTMdata[1]
#                     msg.zone_number = UTMdata[2]
#                     msg.zone_letter = UTMdata[3]
#                     pub.publish(msg)
#                     #pressure_pub.publish(pressure)
#                     #depth_mes = paro_to_depth(pressure - offset, latitude_deg)
#                     #depth_pub.publish(depth_mes)
#                     #odom_msg.pose.pose.position.z = -depth_mes
#                     #odom_msg.header.seq+=0
#                     #odom_pub.publish(odom_msg)
#             rospy.sleep(sleep_time)
            



