#!/bin/bash
echo "[running ~/rosutils/ros2_robot_bashrc.bash]"

: ${ROS_DISTRO:?ERROR: ROS_DISTRO must be set before sourcing ros2_robot_bashrc.bash}

HISTSIZE=1000
HISTFILESIZE=2000

source /opt/ros/${ROS_DISTRO}/setup.bash
source ~/rosutils/ros2_common_bashrc.bash

export ROS2_WS=ros2_ws
export DOME_HOME=${DOME_HOME:-~/.dome}
export DOME_MODE=robot

source ~/rosutils/ros2_dome_bashrc.bash

PATH="$PATH:$HOME/.platformio/penv/bin:/usr/local/bin"

eval "$(mcfly init bash)"
export MCFLY_LIGHT=FALSE
