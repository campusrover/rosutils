echo "[running ~/rosutils/remote_bashrc]"

HISTSIZE=1000
HISTFILESIZE=2000

export ROS2_WS=ros2_ws
source ~/rosutils/ros2_common_bashrc.bash
source /opt/ros/jazzy/setup.bash


export LINOROBOT2_BASE=2wd
export LINOROBOT2_LASER_SENSOR=
export LINOROBOT2_DEPTH_SENSOR=

export LIBGL_ALWAYS_SOFTWARE=1

eval "$(mcfly init bash)"
export MCFLY_LIGHT=TRUE
