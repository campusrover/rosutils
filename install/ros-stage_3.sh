#!/bin/bash
echo "stage-3: ROS is installed and all the packages we need are installed."
echo "Now we create and conflgure catkin_ws"

# Save the current directory
original_dir=$(pwd)


mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
cd src
git clone git@github.com:campusrover/gpg_bran4.git

# Return to the original directory
cd "$original_dir"
