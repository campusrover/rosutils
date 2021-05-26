echo "********************************************"
echo "Copy this file to ~/.bashrc and edit it according to the instuctions below"
echo "********************************************"

echo "[running ~/.bashrc]"
source ~/rosutils/common_alias.bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash

cd ~/catkin_ws

# Support for new bru mode setter
export BRU_MY_IP=$(myip)
export BRU_VPN_IP=$(myvpnip)

bru mode sim
source ~/.bruenv
bru name ... fill in name ...
source ~/.bruenv

