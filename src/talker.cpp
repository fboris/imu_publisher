#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Imu.h"
#include <sstream>
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */
#include <string>
#include <iostream>
#include <cstdio>
#include <unistd.h>
#include "serial/serial.h"
using std::string;
using std::exception;
using std::cout;
using std::cerr;
using std::endl;
void my_sleep(unsigned long milliseconds) {

    usleep(milliseconds*1000); // 100 ms

}
/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{

  srand (time(NULL));

  ros::init(argc, argv, "imu_show");


  ros::NodeHandle n;

  ros::Publisher chatter_pub = n.advertise<sensor_msgs::Imu>("imu_measurement", 1000);

  ros::Rate loop_rate(100);

  
  string port("/dev/ttyUSB0");
  /*Open serial port*/
 
  serial::Serial my_serial(port, 9600, serial::Timeout::simpleTimeout(1000));
  
  

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;
  sensor_msgs::Imu imu_msg;
  string buff;

  while (ros::ok())
  {
    my_serial.readline( buff,   (size_t )100, (std::string )"\r\n");
    ROS_INFO("%s", buff.c_str());
    imu_msg.header.stamp = ros::Time::now();
    imu_msg.angular_velocity.x = 1;
    imu_msg.angular_velocity.y = 1;
    imu_msg.angular_velocity.z = 1;
    imu_msg.linear_acceleration.x =5*(rand()%10);
    imu_msg.linear_acceleration.y =1*(rand()%10);
    imu_msg.linear_acceleration.z =0.1*(rand()%10);
    // /**
    //  * This is a message object. You stuff it with data, and then publish it.
    //  */
    

    // std::stringstream ss;
    // ss << "hello world " << count;
    // msg.data = ss.str();

    // ROS_INFO("%s", msg.data.c_str());

    // *
    //  * The publish() function is how you send messages. The parameter
    //  * is the message object. The type of this object must agree with the type
    //  * given as a template parameter to the advertise<>() call, as was done
    //  * in the constructor above.
     
    chatter_pub.publish( imu_msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}