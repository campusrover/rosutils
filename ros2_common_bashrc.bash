#!/bin/bash
echo "[running ~/rosutils/ros2_common_bashrc]"
source ~/rosutils/ros2_common_alias.bash
export PATH=$PATH:$HOME/.local/bin
export PS1='[\h:\w] '
export BL_TUI_KEYBINDS="exit: c-c; nodes: c-n; loglevel: c-l; mute: c-m"
