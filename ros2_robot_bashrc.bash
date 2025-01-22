#!/bin/bash
echo "[running ~/rosutils/ros2_robot_bashrc]"

HISTSIZE=1000
HISTFILESIZE=2000

source /opt/ros/humble/setup.bash
source ~/rosutils/ros2_common_bashrc.bash

export LINOROBOT2_BASE=2wd
export LINOROBOT2_LASER_SENSOR=
export LINOROBOT2_DEPTH_SENSOR=


export ROS2_WS=linorobot2_ws
source ~/$ROS2_WS/install/setup.bash

eval "$(mcfly init bash)"
