#!/bin/bash
echo "[running ~/rosutils/ros2_common_alias.bash]"

# Bash Functions
robotip() { nslookup "$1" >/dev/null | awk '/Address/&&!/#/{print $2}';  }
stopnow() { rostopic pub -1 /cmd_vel geometry_msgs/Twist '{ linear: { x: 0.0,  y: 0.0,  z: 0.0 }, angular: { x: 0.0,  y: 0.0, z: 0.0 } } ';  }
myip() { ip route get 1.2.3.4 | awk '{print $7}'; }
myvpnip() { ip addr show dev tailscale0 | grep -Eo '([0-9]{1,3}[\.]){3}[0-9]{1,3}' ; }

# Aliases
alias cm='cd ~/catkin_ws && catkin_make'
alias cs='cd ~/catkin_ws/src'
alias cu='cd ~/rosutils'
alias cu='cd ~/rosutils'
alias eb='nano ~/.bashrc'
alias gazempty='roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch'
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
alias cmsingle='catkin_make --only-pkg-with-deps'
alias cmall='catkin_make -DCATKIN_WHITELIST_PACKAGES=""'
alias cameraon='roslaunch raspicam_node camerav2_1280x960_10fps.launch'
alias reboot='sudo shutdown -r now'
alias poweroff='sudo shutdown now'
alias gitkey='eval "$(ssh-agent -s)";ssh-add "/root/.ssh/cluster"'

# Alias for online Tailscale nodes
alias tson='tailscale status --json | jq -r '\''.Peer[] | select(.Online) | .DNSName |= (.[:-1] | split("-clouddesktop")[0]) | .TailscaleIPs[0] as $ip | .DNSName + " - " + $ip'\'''

# Alias for offline Tailscale nodes
alias tsoff='tailscale status --json | jq -r '\''.Peer[] | select(.Online == false) | .DNSName |= (.[:-1] | split("-clouddesktop")[0]) | .TailscaleIPs[0] as $ip | .DNSName + " - " + $ip'\'''


alias real='$(bru mode real)'
alias sim='$(bru mode sim)'
alias sshrobot='ssh ubuntu@$BRU_MASTER_IP'
alias bringup='roslaunch turtlebot3_bringup turtlebot3_robot.launch' # Will be 'roslaunch platform full_bringup.launch' if platform robot

alias multibringup='roslaunch turtlebot3_bringup turtlebot3_multi_robot.launch'
alias pio-upload="source ~/rosutils/install/pio-upload.sh"
alias pio-compile="source ~/rosutils/install/pio-compile.sh"
alias ros-stage_3="source ~/rosutils/install/ros-stage_3.sh"

PS1="[\$BRU_MODE:\$BRU_NAME]\w$ "

# More complicated commands
source ~/rosutils/publish_arm_command.sh
alias armcmd='publish_arm_command'

# Git push with zero or more params
alias gp='if [ $# -eq 0 ]; \
          then git add --all; \
               git commit -m "work in progress"; \
               git push; \
          else MSG=$*; \
               git add --all; \
               
               git commit -m "$MSG"; \
               git push; \
          fi'
