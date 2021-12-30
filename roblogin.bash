#!/bin/bash

# read -p  'Robot Name: ' robname

# if [$robname == "robc"]
# then
#     m="100.82.10.90"
# fi
# export ROS_MASTER_URI=$m
# read -sp 'Robot Password: ' robpass
# echo ""
# sshpass -p ${robpass} ssh -l ubuntu ${robip} 'source .bashrc'
export SSHPASS="ROSlab134"
bu="/home/ubuntu/bringup.sh -m "$(printenv ROS_MASTER_URI)
roscore & sshpass -e ssh 'ubuntu@100.82.10.90' $bu
