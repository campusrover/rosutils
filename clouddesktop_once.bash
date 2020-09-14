#!/bin/bash
echo "[running clouddesktop_once.bash]"
myvpnip() { ip addr show dev tailscale0 | grep -Eo '([0-9]{1,3}[\.]){3}[0-9]{1,3}' ; }
myip() { ip route get 1.2.3.4 | awk '{print $7}'; }

# uncomment to be on VPN or not
export HOST_IP=$(myvpnip)
#export HOST_IP=$(myip)

ROSCORE_IP=100.94.206.80

export ROS_IP=${HOST_IP}
export ROS_HOSTNAME=${HOST_IP}
echo export ROS_IP=$ROS_IP >> ~/.bashrc
echo export ROS_HOSTNAME=$HOST_IP >> ~/.bashrc

# Default ROS master port is 11311
#  - You could use 80 if accessing over the internet: roscore -p 80
export ROS_MASTER_URI=http://${ROSCORE_IP}:11311
echo export ROS_MASTER_URI=http://${ROSCORE_IP}:11311 >> ~/.bashrc

# Source ROS configuration
echo "Sourcing ROS Melodic configuration..."
source /opt/ros/melodic/setup.bash