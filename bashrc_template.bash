echo "********************************************"
echo "Copy this file to ~/.bashrc and edit it according to the instuctions below"
echo "********************************************"

echo "[running ~/.bashrc]"
source ~/rosutils/common_alias.bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash

# Support for new bru mode setter
export BRU_MY_IP=$(myip)
export BRU_VPN_IP=$(myvpnip)

$(bru mode sim)
$(bru name <name> -m $(myvpnip))

# Support for Linorobot
export LINOLIDAR="2wd"
export LINOBASE="ydlidar"
