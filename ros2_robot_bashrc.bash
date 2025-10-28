#!/bin/bash
echo "[running ~/rosutils/ros2_robot_bashrc.bash]"

HISTSIZE=1000
HISTFILESIZE=2000

source /opt/ros/jazzy/setup.bash
source ~/rosutils/ros2_common_bashrc.bash

export LINOROBOT2_BASE=2wd
export LINOROBOT2_LASER_SENSOR=ld19
export LINOROBOT2_DEPTH_SENSOR=

export ROS2_WS=linorobot2_ws
source ~/$ROS2_WS/install/setup.bash

eval "$(mcfly init bash)"
export MCFLY_LIGHT=FALSE

PATH="$PATH:$HOME/.platformio/penv/bin"
