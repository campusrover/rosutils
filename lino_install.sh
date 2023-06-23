#!/usr/bin/env bash

set -e

source /opt/ros/$(dir /opt/ros)/setup.bash

wget https://www.pjrc.com/teensy/00-teensy.rules
sudo cp 00-teensy.rules /etc/udev/rules.d/

ROSDISTRO=noetic
ARCH="$(uname -m)"
echo $ARCH
echo
echo "INSTALLING NOW...."

sudo apt-get update
sudo apt-get install -y \
avahi-daemon \
openssh-server \
python-dev \
build-essential \
python-gudev \
python-gobject

sudo pip install -U platformio
source /opt/ros/noetic/setup.bash

cd ~
mkdir -p linorobot_ws/src
cd ~/linorobot_ws/src
catkin_make

sudo apt-get install -y \
ros-noetic-roslint \
ros-noetic-rosserial \
ros-noetic-rosserial-arduino \
ros-noetic-imu-filter-madgwick \
ros-noetic-gmapping \
ros-noetic-map-server \
ros-noetic-navigation \
ros-noetic-robot-localization \
ros-noetic-tf2 \
ros-noetic-tf2-ros \
robot-state-publisher \
ros-noetic-joy \
ros-noetic-xacro

sudo apt install ros-noetic-image-transport
sudo apt install ros-noetic-compressed-image-transport
sudo apt install ros-noetic-camera-info-manager
sudo apt install libraspberrypi-dev libraspberrypi0
sudo apt install libraspberrypi-bin
sudo apt-get install ros-noetic-teleop-twist-keyboard

cd ~/linorobot_ws/src
git clone https://github.com/EAIBOT/ydlidar.git

cd ~/linorobot_ws/src
git clone https://github.com/linorobot/linorobot.git
git clone https://github.com/linorobot/imu_calib.git
git clone https://github.com/linorobot/lino_pid.git
git clone https://github.com/linorobot/lino_udev.git
git clone https://github.com/linorobot/lino_msgs.git

cd $HOME/linorobot_ws/src/linorobot/teensy/firmware
# export PLATFORMIO_CI_SRC=$PWD/src/firmware.ino
# platformio ci --project-conf=./platformio.ini --lib="./lib/ros_lib" --lib="./lib/config"  --lib="./lib/motor"  --lib="./lib/kinematics"  --lib="./lib/pid"  --lib="./lib/imu" --lib="./lib/encoder"

pio run --target upload -e teensy41
cmsingle lino_msgs
cmall

# echo
# echo "INSTALLATION DONE!"
# echo
