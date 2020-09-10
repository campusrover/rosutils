#!/bin/bash

echo "[running clouddesktop_once.bash]"

export HOST_IP=$(myvpnip)

export ROS_IP=${HOST_IP}
echo export ROS_IP=${HOST_IP} >> ~/.bashrc

# Default ROS master port is 11311
#  - You could use 80 if accessing over the internet: roscore -p 80
export ROS_MASTER_URI=http://${HOST_IP}:11311
echo export ROS_MASTER_URI=http://${HOST_IP}:11311 >> ~/.bashrc

# Source ROS configuration
echo "Sourcing ROS Melodic configuration..."
source /opt/ros/melodic/setup.bash