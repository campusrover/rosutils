#!/bin/bash
echo "[running ~/rosutils/ros2_common_alias.bash]"

# Aliases
alias allros='ps aux | grep -E "ros|nav|slam|rviz"'
alias cs='cd ~/$ROS2_WS/src'
alias cw='cd ~/$ROS2_WS; source ~/$ROS2_WS/install/setup.bash'
alias gs='git status'
alias sb='source ~/.bashrc'
alias rb='sudo shutdown -r now'
alias po='sudo shutdown now'
alias ch='cd ~/linorobot2_hardware'
alias cu='cd ~/rosutils'
alias gu='git remote -v'
alias cb='colcon build'
alias cbp='colcon build --packages-select'
alias cbi='colcon build --symlink-install'
alias new='ls -lt | head -n 20'
alias pio='platformio'
alias pioupload='platformio run --target upload'
alias pioenv='platformio run --environment'
alias killdome="ssh dome1 'sudo shutdown now'"
alias bringupdome="ssh dome1 'cw; bl dome mini_bringup_bl.launch.py --rviz_arg false'"

# Bash Functions
myip() { ip route get 1.2.3.4 | awk '{print $7}'; }

myvpnip() { ip addr show dev tailscale0 | grep -Eo '([0-9]{1,3}[\.]){3}[0-9]{1,3}'; }

gp() {
    local msg branch remote exit_code

    if ! git rev-parse --git-dir > /dev/null 2>&1; then
        echo "Error: Not a git repository" >&2
        return 1
    fi

    branch=$(git symbolic-ref --short HEAD 2>/dev/null)
    if [ -z "$branch" ]; then
        echo "Error: Unable to determine current branch" >&2
        return 1
    fi

    if git diff-index --quiet HEAD -- 2>/dev/null && \
       [ -z "$(git ls-files --others --exclude-standard)" ]; then
        echo "Working tree clean - nothing to commit"
        return 0
    fi

    msg=${*:-"work in progress"}

    echo "Staging all changes..."
    git add --all || { echo "Error: Failed to stage changes" >&2; return 1; }

    echo "Committing: $msg"
    git commit -m "$msg" || { echo "Error: Commit failed" >&2; return 1; }

    remote=$(git config branch."$branch".remote 2>/dev/null || echo "origin")
    if ! git remote get-url "$remote" > /dev/null 2>&1; then
        echo "Warning: Remote '$remote' not found, defaulting to 'origin'" >&2
        remote="origin"
    fi

    echo "Pushing to $remote/$branch..."
    git push "$remote" "$branch" || { echo "Error: Push failed" >&2; return 1; }

    echo "✓ Successfully pushed to $remote/$branch"
}