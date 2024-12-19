echo "[running ~/rosutils/ros2_robot_bashrc]"

HISTSIZE=1000
HISTFILESIZE=2000

source /opt/ros/humble/setup.bash
source ~/rosutils/ros2_common_bashrc.bash

export ROS_DOMAIN_ID=5
export LINOROBOT2_BASE=2wd
export LINOROBOT2_LASER_SENSOR=a1
export LINOROBOT2_DEPTH_SENSOR=

source $HOME/linorobot2_ws/install/setup.bash
alias bringup=ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 --baudrate 921600


