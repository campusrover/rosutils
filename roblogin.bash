#!/bin/bash

read -p  'Robot Name: ' robname

export SSHPASS="ROSlab134"
bu="/home/ubuntu/bringup.sh -m "$(printenv ROS_MASTER_URI)
sshpass -e ssh 'ubuntu@'$(printenv $robname) $bu
