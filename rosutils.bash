# Pito's handy ROS shell setup
# Placed in ~/mydev/rosutils/.rosbash
# Add `source ~/mydev/rosutils/.rosbash` in ~/.bashrc at the end

myip() { ip route get 8.8.8.8 | awk '{print $NF; exit}'; }

alias gazempty='roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch'
alias teleop='roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch'
alias settime='sudo ntpdate ntp.ubuntu.com'
alias bringup="roslaunch turtlebot3_bringup turtlebot3_robot.launch"
alias sshdon="sudo ssh robot@129.64.243.61"
alias sshraf="sudo ssh rafael@129.64.243.62"
alias sshtb2="ssh turtlebot@129.64.243.64"

stopnow() { rostopic pub /cmd_vel geometry_msgs/Twist '{ linear: { x: 0.0,  y: 0.0,  z: 0.0 }, angular: { x: 0.0,  y: 0.0, z: 0.0 } } ';  }

export IP="$(ifconfig | sed -En 's/127.0.0.1//;s/.*inet (addr:)?(([0-9]*\.){3}[0-9]*).*/\2/p')"`
export ROS_IP=$IP
export ROS_MASTER_URI=http://roscore1.cs.brandeis.edu:11311
export TB3_MODEL=burger
export TURTLEBOT3_MODEL=burger

