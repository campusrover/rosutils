# Pito's handy ROS shell setup
# Placed in ~/mydev/rosutils/.bashrc
# Add `source ~/mydev/rosutils/.bashrc` in ~/.bashrc at the end

alias gazempty='roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch'
alias teleop='roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch'
alias settime='sudo ntpdate ntp.ubuntu.com'
alias bringup="roslaunch turtlebot3_bringup turtlebot3_robot.launch"
alias sshdon="sudo ssh robot@129.64.243.61"
alias sshraf="sudo ssh rafael@129.64.243.62"
alias sshtb2="ssh turtlebot@129.64.243.64"

stopnow() { rostopic pub /cmd_vel geometry_msgs/Twist '{ linear: { x: 0.0,  y: 0.0,  z: 0.0 }, angular: { x: 0.0,  y: 0.0, z: 0.0 } } ';  }
myip() { ip route get 1.2.3.4 | awk '{print $7}'; }
export ROS_IP=$(myip)
# export ROS_MASTER_URI=http://roscore1.cs.brandeis.edu:11311
export ROS_MASTER_URI="http://$ROS_IP:11311"
export TB3_MODEL=burger
export TURTLEBOT3_MODEL=burger
