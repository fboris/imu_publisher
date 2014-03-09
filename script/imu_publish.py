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
from std_msgs.msg import String
from sensor_msgs.msg import Imu
def talker():

    pub = rospy.Publisher('imu_measurements', Imu)
    rospy.init_node('talker', anonymous=True)
    r = rospy.Rate(200) # 10hz
    myImu = Imu();

    ser = serial.Serial('/dev/ttyUSB0',57600)  # open first serial port
    print ser.name          # check which port was really used
    rospy.loginfo("start connecting!")
    isMsg = False
    buff = []
    while not rospy.is_shutdown():
        ch = ser.read()

        if ch == 'I':
            myImu.header.stamp = rospy.get_rostime()
            rospy.loginfo("got package!")
            read_buff = ser.read(12);
            print len(read_buff[0:1])
            myImu.linear_acceleration.x = struct.unpack("h",read_buff[0:2])[0]/16384.0
            myImu.linear_acceleration.y = struct.unpack("h",read_buff[2:4])[0]/16384.0;
            myImu.linear_acceleration.z = struct.unpack("h",read_buff[4:6])[0]/16384.0;
            myImu.angular_velocity.x = struct.unpack("h",read_buff[6:8])[0]/131.0;
            myImu.angular_velocity.y = struct.unpack("h",read_buff[8:10])[0]/131.0;
            myImu.angular_velocity.z = struct.unpack("h",read_buff[10:12])[0]/131.0;



        rospy.loginfo(myImu)
        pub.publish(myImu)
        r.sleep()

    ser.close()    
        
if __name__ == '__main__':


    try:
        talker()
    except rospy.ROSInterruptException: pass

