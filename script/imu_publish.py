#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

import roslib; roslib.load_manifest('imu_publisher')
import serial
import random
import rospy
import string
import struct
import math
from std_msgs.msg import String
from sensor_msgs.msg import Imu
def talker():

    pub = rospy.Publisher('imu_body', Imu)
    rospy.init_node('talker', anonymous=True)
    r = rospy.Rate(1000) # 10hz
 
    Imu_body = Imu();
    acc = [0.0,0.0,0.0]
    gyro = [0.0,0.0,0.0]

    ser = serial.Serial('/dev/ttyUSB0',230400)  # open first serial port
    print ser.name          # check which port was really used
    rospy.loginfo("start connecting!")
    isMsg = False
    buff = []
    with open('./imu.txt', 'w') as f:
        f.write("This imu data is record in ENU coordinate\r\n")
        f.write("acc_x\tacc_y\tacc_z\tgyro_x\tgyro_y\tgryo_z\r\n")
        while not rospy.is_shutdown():
            ch = ser.read()

            if ch == 'I':
                Imu_body.header.stamp = rospy.get_rostime()
                #rospy.loginfo("got package!")
                read_buff = ser.read(12);
                #print len(read_buff[0:1])
                acc[0] = struct.unpack("h",read_buff[0:2])[0]/16384.0*9.8 
                acc[1] = struct.unpack("h",read_buff[2:4])[0]/16384.0*9.8 
                acc[2] = struct.unpack("h",read_buff[4:6])[0]/16384.0*9.8 
                gyro[0] = struct.unpack("h",read_buff[6:8])[0]/131.0 
                gyro[1] = struct.unpack("h",read_buff[8:10])[0]/131.0 
                gyro[2] = struct.unpack("h",read_buff[10:12])[0]/131.0 
                Imu_body.linear_acceleration.x = -acc[1]
                Imu_body.linear_acceleration.y = acc[2]
                Imu_body.linear_acceleration.z = -acc[0]
                Imu_body.angular_velocity.x = -gyro[1]*math.pi/180#-gyro[1]
                Imu_body.angular_velocity.y = gyro[2]*math.pi/180#gyro[2]
                Imu_body.angular_velocity.z = -gyro[0]*math.pi/180#-gyro[0]
                f.write("{0}\t{1}\t{2}\t{3}\t{4}\t{5}\r\n".format(
                    Imu_body.linear_acceleration.x,
                    Imu_body.linear_acceleration.x,
                    Imu_body.linear_acceleration.x,
                    Imu_body.angular_velocity.x,
                    Imu_body.angular_velocity.y,
                    Imu_body.angular_velocity.z))


            #rospy.loginfo(Imu_body)
            pub.publish(Imu_body)
            r.sleep()
    f.closed

    ser.close()    
        
if __name__ == '__main__':


    try:
        talker()
    except rospy.ROSInterruptException: pass

