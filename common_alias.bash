echo "[running common_alias.bash]"

export ROBOT_IP=100.94.206.80

# Bash Functions
stopnow() { rostopic pub /cmd_vel geometry_msgs/Twist '{ linear: { x: 0.0,  y: 0.0,  z: 0.0 }, angular: { x: 0.0,  y: 0.0, z: 0.0 } } ';  }
myip() { ip route get 1.2.3.4 | awk '{print $7}'; }
myvpnip() { ip addr show dev tailscale0 | grep -Eo '([0-9]{1,3}[\.]){3}[0-9]{1,3}' ; }

# Mode setters just change the ROS_MASTER_URI because everything else is the same
alias setcloud='export ROS_MASTER_URI=http://$(myvpnip):11311; export SETSTATE=cloud'
alias setrobot='export ROS_MASTER_URI=http://$ROBOT_IP:11311; export SETSTATE=robot'
alias setdocker='export ROS_MASTER_URI=http://$(myip):11311; export SETSTATE=docker'

# Aliases
alias gazempty='roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch'
alias teleop='roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch'
alias settime='sudo ntpdate ntp.ubuntu.com'
alias eb='nano ~/.bashrc'
alias sb='source ~/.bashrc'
alias gs='git status'
alias gp='git pull'
alias cw='cd ~/catkin_ws'
alias cs='cd ~/catkin_ws/src'
alias cm='cd ~/catkin_ws && catkin_make'
alias ll='ls -alF'
alias la='ls -A'
alias l='ls -CF'
alias showenv='source ~/rosutils/showenv.bash'

# Prompt

PS1="\w$ "