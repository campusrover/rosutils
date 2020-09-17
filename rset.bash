#!/bin/bash
# rset <mode>
#       robot - desktop working with real robot
#       docker - docker working with a simulated robot
#       cloud - cloud working with simulated robot
#       pi - on board actual robot

if test $1 == robot; then
    echo setting environment ROBOT
    export ROS_MASTER_URI=http://$ROBOT_IP:11311
    export ROS_IP=$(myvpnip)
    export SETSTATE=ROBOT
elif test $1 == docker; then
    echo setting environment DOCKER
    export ROS_MASTER_URI=http://$(myip):11311; 
    export ROS_IP=$(myip)
    export SETSTATE=docker
elif test $1 == cloud; then
    echo setting environment CLOUD
    export ROS_MASTER_URI=http://$(myvpnip):11311
    export ROS_IP=$(myvpnip)
    export SETSTATE=CLOUD
elif test $1 == pi; then
    echo setting environment CLOUD
    export ROS_MASTER_URI=http://$(myvpnip):11311
    export ROS_IP=$(myvpnip)
    export SETSTATE=PI
else
    echo bad input to rset. Must be pi, robot, docker or cloud
fi
