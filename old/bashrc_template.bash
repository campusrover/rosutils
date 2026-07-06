echo "********************************************"
echo "Edit this file by typing nano ~/.bashrc"
echo "********************************************"

# Instructions
# Generally you need to edit this the first time you use this vnc, and 
# again when you switch robots.
#
# Instructions for use in sumulation
# Make sure that the following two lines are not commented (below)
# $(bru mode sim) and $(bru name sim -m $(myvpnip))

# Instructions for working with a real robot
# Find out what the robot's name and vpn ip is. 
# The robot is marked with its name.
# Then ssh'ing to the robot
# And type the command `myvpnip`
#     if nothing comes out, then tailscale is not working. Get help from a TA
#     if an ip comes out, write it down.
# Close this tab and open a new one to get the changes to be applied

echo "[running ~/.bashrc]"
source ~/rosutils/common_alias.bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash

# Support for new bru mode setter
export BRU_MY_IP=$(myip)
export BRU_VPN_IP=$(myvpnip)

# Setting for simulation mode. Comment them out if you are working with a real robot
$(bru mode sim)
$(bru name sim -m $(myvpnip))

# Settings for a physical robot. Comment them out if you are using sim
# $(bru mode real)
# $(bru name <name> -m <robots vpn ip>)


# Support for Linorobot
export LINOBASE="2wd"
export LINOLIDAR="ydlidar"
