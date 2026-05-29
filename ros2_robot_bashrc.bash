#!/bin/bash
echo "[running ~/rosutils/ros2_robot_bashrc.bash]"

HISTSIZE=1000
HISTFILESIZE=2000

source /opt/ros/${ROS_DISTRO:-kilted}/setup.bash
source ~/rosutils/ros2_common_bashrc.bash

# Linorobot
export LINOROBOT2_BASE=2wd
export LINOROBOT2_LASER_SENSOR=ld19
export LINOROBOT2_DEPTH_SENSOR=

export ROS2_WS=ros2_ws

eval "$(mcfly init bash)"
export MCFLY_LIGHT=FALSE

PATH="$PATH:$HOME/.platformio/penv/bin:/usr/local/bin"

# The dopller secrets command creates a list of set commands, which create shell local
# variables. By adding -a, we export those variables to the environment, so they are available to ROS2 nodes.
# Another approach would be to write the secrets to a file, and then source that file which might
# be a a little faster. If needed we can do this in the future.

if doppler configure get token --plain &>/dev/null 2>&1; then
  set -a
  eval $(doppler secrets download --no-file --format env)
  set +a
else
  echo "[doppler] no token configured — skipping secrets"
fi

export DOME_MODE=robot
