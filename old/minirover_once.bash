#!/bin/bash

echo "[running minirover_once.bash]"
# Thanks to Bernardo

echo 127.0.0.1 `hostname` >> /etc/hosts
myvpnip() { ip addr show dev tailscale0 | grep -Eo '([0-9]{1,3}[\.]){3}[0-9]{1,3}' ; }
myip() { ip route get 1.2.3.4 | awk '{print $7}'; }

# uncomment to be on VPN or not
export HOST_IP=$(myvpnip)
#export HOST_IP=$(myip)

export ROS_IP=$HOST_IP
echo export ROS_IP=$ROS_IP >> ~/.bashrc

# Default ROS master port is 11311
export ROS_MASTER_URI=http://${HOST_IP}:11311
echo export ROS_MASTER_URI=http://${HOST_IP}:11311 >> ~/.bashrc

# Source ROS configuration
echo "Sourcing ROS Melodic configuration..."
source /opt/ros/melodic/setup.bash