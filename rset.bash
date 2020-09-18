#!/bin/bash
# rset <mode>
#       robot - desktop working with real robot
#       docker - docker working with a simulated robot
#       cloud - cloud working with simulated robot
#       pi - on board actual robot
# Enter by hand the known ip of the rapsberry pi on the robot
export ROBOT_IP=100.94.206.80

# Useful functions
myip() { ip route get 1.2.3.4 | awk '{print $7}'; }
myvpnip() { ip addr show dev tailscale0 | grep -Eo '([0-9]{1,3}[\.]){3}[0-9]{1,3}' ; }

echo [running rset.bash]
if test -z $1; then
    source ~/rosutils/showenv.bash
    exit
elif test "$1" == "robot"; then
    echo setting environment ROBOT
    export ROS_MASTER_URI=http://$ROBOT_IP:11311
    export ROS_IP=$(myvpnip)
    export SETSTATE=ROBOT
elif test "$1" == docker; then
    echo   [setting environment DOCKER]
    export ROS_MASTER_URI=http://$(myip):11311; 
    export ROS_IP=$(myip)
    export SETSTATE=docker
elif test "$1" == cloud; then
    echo   [setting environment CLOUD]
    export ROS_MASTER_URI=http://$(myvpnip):11311
    export ROS_IP=$(myvpnip)
    export SETSTATE=CLOUD
elif test "$1" == pi; then
    echo   [setting environment pi]
    export ROS_MASTER_URI=http://$(myvpnip):11311
    export ROS_IP=$(myvpnip)
    export SETSTATE=PI
else
    echo bad input to rset. Must be pi, robot, docker or cloud
    exit 2
fi
source ~/rosutils/showenv.bash
