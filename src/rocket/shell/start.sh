#!/bin/bash
# For starting up rocket stuff. Since we need GPIO access, this requires to be be a superuser

# check if super user
if [ $(id -u) != "0" ]; then
	echo "Please first switch to superuser using the command 'sudo su' before running this script"
	exit 1
fi

echo "Sourcing ROS setup files"
source /home/pi/ros_catkin_ws/install_isolated/setup.bash
source /home/pi/catkin_ws/devel/setup.bash
echo "Starting roscore"
roscore &
sleep 10
echo "Starting deployment node"
rosrun rocket deployment.py
