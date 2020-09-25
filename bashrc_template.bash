echo "[running ~/.bashrc]"
source ~/rosutils/common_alias.bash
echo "********************************************"
echo "Copy this file to ~/.bashrc"
echo "Uncomment the right rset command below"
echo "If want to access a physical robot, place its vpnip below"
echo "And delete these echo lines"
echo "********************************************"

# Uncomment one of these"
# rset robot # if you are on your web desktop and want to control a real robot
# rset cloud # if you are on your web desktiop and working with a simulated robot
# rset pi # if this bashrc is on your raspberry pi

# If you are working with a real robot (or are the real robot) uncomment
# this line and put the vpnip of the robot
# export ROBOT_IP=100.94.206.80


