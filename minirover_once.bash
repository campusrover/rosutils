#!/bin/bash

echo "[running minirover_once.bash]"
# Thanks to Bernardo
echo 127.0.0.1 `hostname` >> /etc/hosts

# Put here the local IP of the host device (robot)
export LANIFACE=$(ip route get 1.1.1.1 | grep -Po '(?<=dev\s)\w+' | cut -f1 -d ' ')
export HOST_IP=$(ifconfig ${LANIFACE} | awk '/inet / {print $2}')

export ROS_IP=${HOST_IP}
echo export ROS_IP=${HOST_IP} >> ~/.bashrc

# Default ROS master port is 11311
#  - You could use 80 if accessing over the internet: roscore -p 80
export ROS_MASTER_URI=http://${HOST_IP}:11311
echo export ROS_MASTER_URI=http://${HOST_IP}:11311 >> ~/.bashrc

# Source ROS configuration
echo "Sourcing ROS Melodic configuration..."
source /opt/ros/melodic/setup.bash