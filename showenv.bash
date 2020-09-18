#!/bin/bash
source ~/catkin_ws/devel/setup.bash
echo -e "++++++++++++++++++++++++++++++++++++++++++++++++++"
echo "SETMODE is $SETSTATE"
echo "ROS_IP is $ROS_IP"
echo "ROS_MASTER_URI is $ROS_MASTER_URI"
echo "ROS_HOSTNAME is $ROS_HOSTNAME"
echo -e "and '~/catkin_ws' is the current Workspace"
echo "++++++++++++++++++++++++++++++++++++++++++++++++++"
cd ~/catkin_ws
