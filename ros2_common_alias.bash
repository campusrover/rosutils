#!/bin/bash
echo "[running ~/rosutils/ros2_common_alias.bash]"

# Bash Functions
myip() { ip route get 1.2.3.4 | awk '{print $7}'; }
myvpnip() { ip addr show dev tailscale0 | grep -Eo '([0-9]{1,3}[\.]){3}[0-9]{1,3}' ; }

# Aliases
alias cm='cd ~/$ROS2_WS && colcon build --symlink-install'
alias cs='cd ~/$ROS2_WS/src'
alias cw='cd ~/$ROS2_WS; source ~/$ROS2_WS/install/setup.bash'
alias gs='git status'
alias l='ls -CF'
alias la='ls -A'
alias ll='ls -alF'
alias cw='cd ~/ros2_ws; source ~/ros2_ws/install/setup.bash'
alias sb='source ~/.bashrc'
alias rb='sudo shutdown -r now'
alias po='sudo shutdown now'
alias sb='source ~/.bashrc'
alias ch='cd ~/linorobot2_hardware'
alias cu='cd ~/rosutils'

PS1="[\$BRU_MODE:\$BRU_NAME]\w$ "

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
