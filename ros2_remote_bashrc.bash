#!/bin/bash
echo "[running ~/rosutils/ros2_remote_bashrc.bash]"

: ${ROS_DISTRO:?ERROR: ROS_DISTRO must be set before sourcing ros2_remote_bashrc.bash}

HISTSIZE=1000
HISTFILESIZE=2000

source /opt/ros/${ROS_DISTRO}/setup.bash
source ~/rosutils/ros2_common_bashrc.bash

export ROS2_WS=ros2_ws
export DOME_HOME=${DOME_HOME:-~/.dome}
export DOME_MODE=remote

source ~/rosutils/ros2_dome_bashrc.bash

export LIBGL_ALWAYS_SOFTWARE=1

eval "$(mcfly init bash)"
export MCFLY_LIGHT=FALSE
