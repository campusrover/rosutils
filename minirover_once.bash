#!/bin/bash

echo "[running minirover_once.bash]"
# Thanks to Bernardo

sudo echo 127.0.0.1 `hostname` >> /etc/hosts
myvpnip() { ip addr show dev tailscale0 | grep -Eo '([0-9]{1,3}[\.]){3}[0-9]{1,3}' ; }

# Put here the local IP of the host device (robot)
export LANIFACE=$(ip route get 1.1.1.1 | grep -Po '(?<=dev\s)\w+' | cut -f1 -d ' ')
export HOST_IP=$(ifconfig ${LANIFACE} | awk '/inet / {print $2}')

# uncomment to be on VPN
export HOST_IP=$(myvpnip)

export ROS_IP=${HOST_IP}
echo export ROS_IP=${HOST_IP} >> ~/.bashrc

# Default ROS master port is 11311
export ROS_MASTER_URI=http://${HOST_IP}:11311
echo export ROS_MASTER_URI=http://${HOST_IP}:11311 >> ~/.bashrc

# Source ROS configuration
echo "Sourcing ROS Melodic configuration..."
source /opt/ros/melodic/setup.bash