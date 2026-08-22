#!/bin/bash
echo "[running ~/rosutils/ros2_remote_bashrc.bash]"

HISTSIZE=1000
HISTFILESIZE=2000

# Native macOS has no /opt/ros install (this repo assumes a real ROS 2 +
# Linux setup) - skip the ROS-specific sourcing there instead of erroring.
if [ "$(uname -s)" = "Darwin" ]; then
  echo "[ros2_remote_bashrc] macOS detected - skipping ROS 2 setup.bash (not installed natively)"
else
  : ${ROS_DISTRO:?ERROR: ROS_DISTRO must be set before sourcing ros2_remote_bashrc.bash}
  source /opt/ros/${ROS_DISTRO}/setup.bash
fi

source ~/rosutils/ros2_common_bashrc.bash

export ROS2_WS=ros2_ws
export DOME_HOME=${DOME_HOME:-~/.dome}
export DOME_MODE=remote

source ~/rosutils/ros2_dome_bashrc.bash

export LIBGL_ALWAYS_SOFTWARE=1

if command -v mcfly >/dev/null 2>&1; then
  eval "$(mcfly init bash)"
  export MCFLY_LIGHT=FALSE
fi
