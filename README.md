imu_publisher
=============

simple imu data plot on ROS

#Prerequisite

- ROS groovy: use apt-get, please refer this [installation guide](http://wiki.ros.org/groovy/Installation) 
- PySerial: use pip or easy_install, please go [this website](http://pyserial.sourceforge.net/pyserial.html#installation)

#Getting start
1.create your own catkin workspace
2.go to the root of your catkin workspace, type 
```
catkin_make
./devel/setup.sh
```
3.Execute the node
`rosrun imu_publisher`
If you want to see the imu measurements plot, please open new tab of your terminal. And execute the shell script in `/src/srcipt/plot_acc.sh` or `/src/srcipt/plot_gyro.sh`
