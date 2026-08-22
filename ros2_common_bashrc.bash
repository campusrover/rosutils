#!/bin/bash
echo "[running ~/rosutils/ros2_common_bashrc]"
source ~/rosutils/ros2_common_alias.bash
export PATH=$PATH:$HOME/.local/bin

# What kind of machine is this? (raspi | macos | ubuntu-vm | ubuntu)
detect_rosutils_platform() {
  case "$(uname -s)" in
    Darwin) echo "macos"; return ;;
  esac
  if grep -qi "raspberry pi" /proc/device-tree/model 2>/dev/null; then
    echo "raspi"; return
  fi
  if command -v systemd-detect-virt >/dev/null 2>&1; then
    local virt
    virt="$(systemd-detect-virt 2>/dev/null)"
    if [ -n "$virt" ] && [ "$virt" != "none" ]; then
      echo "ubuntu-vm"; return
    fi
  fi
  echo "ubuntu"
}
export ROSUTILS_DIR="$HOME/rosutils"
export ROSUTILS_PLATFORM="$(detect_rosutils_platform)"
unset -f detect_rosutils_platform

export PS1='[\h:\w] '
export BL_TUI_KEYBINDS="exit: c-c; nodes: c-n; loglevel: c-l; mute: c-m"
export BL_UI_OVERRIDE=enable
export BL_COLORMODE_OVERRIDE=source
