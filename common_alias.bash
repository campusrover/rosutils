#!/bin/bash
echo "[running common_alias.bash]"


# Bash Functions
robotip() { nslookup "$1" >/dev/null | awk '/Address/&&!/#/{print $2}';  }
stopnow() { rostopic pub /cmd_vel geometry_msgs/Twist '{ linear: { x: 0.0,  y: 0.0,  z: 0.0 }, angular: { x: 0.0,  y: 0.0, z: 0.0 } } ';  }
myip() { ip route get 1.2.3.4 | awk '{print $7}'; }
myvpnip() { ip addr show dev tailscale0 | grep -Eo '([0-9]{1,3}[\.]){3}[0-9]{1,3}' ; }

# Aliases
alias cm='cd ~/catkin_ws && catkin_make'
alias cs='cd ~/catkin_ws/src'
alias cu='cd ~/rosutils'
alias cu='cd ~/rosutils'
alias eb='nano ~/.bashrc'
alias gazempty='roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch'
alias gp='git pull'
alias gs='git status'
alias l='ls -CF'
alias la='ls -A'
alias ll='ls -alF'
alias lw='cd ~/linorobot_ws/; source devel/setup.bash'
alias cw='cd ~/catkin_ws; source devel/setup.bash'
alias restart='supervisorctl -u root -p dev@ros restart x:*'
alias sb='source ~/.bashrc'
alias settime='sudo ntpdate ntp.ubuntu.com'
alias teleop='roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch'
alias eduroam='nmcli connection up eduroam --ask'
alias cmsingle='catkin_make --only-pkg-with-deps'
alias cmall='catkin_make -DCATKIN_WHITELIST_PACKAGES=""'
alias cameraon='roslaunch raspicam_node camerav2_1280x960_10fps.launch'
alias real='$(bru mode real)'
alias sim='$(bru mode sim)'
alias bringup='roslaunch turtlebot3_bringup turtlebot3_robot.launch' # Will be 'roslaunch platform full_bringup.launch' if platform robot

# Prompt

PS1="[\$BRU_MODE:\$BRU_NAME]\w$ "
