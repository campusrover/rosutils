echo "[running ~/rosutils/remote_bashrc]"

HISTSIZE=1000
HISTFILESIZE=2000

source ~/rosutils/ros2_common_bashrc
source /opt/ros/humble/setup.bash
source ~/rosutils/ros2_common_alias.bash
export ROS_DOMAIN_ID=5

