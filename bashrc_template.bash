echo "[running ~/.bashrc]"
source ~/rosutils/common_alias.bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash

# Support for new bru mode setter
export BRU_MY_IP=$(myip)
export BRU_VPN_IP=$(myvpnip)

$(bru mode onboard)
$(bru name pitosalas -m $(myvpnip))

# Support for Linorobot
export LINOLIDAR="2wd"
export LINOBASE="ydlidar"

