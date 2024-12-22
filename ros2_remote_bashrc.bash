echo "[running ~/rosutils/remote_bashrc]"

HISTSIZE=1000
HISTFILESIZE=2000

export ROS2_WS=$HOME/ros2_ws
source ~/rosutils/ros2_common_bashrc.bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=5

