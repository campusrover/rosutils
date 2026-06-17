#!/usr/bin/env bash
# Kill all ROS2-related processes

usage() {
  echo "Usage: kill_all_ros2.bash [-f | -i]"
  echo "  -f  force: kill all without prompting"
  echo "  -i  interactive: prompt for each process (default)"
  echo "  -h  show this help"
  exit 0
}

case "$1" in
  -f) MODE="force" ;;
  -i) MODE="interactive" ;;
  -h|--help) usage ;;
  *) usage ;;
esac

PATTERN='ros2|nav2|slam|rviz|gazebo|gzserver|gzclient|robot_state_pub|joint_state_pub|rosbridge|component_container|parameter_server'

pids=$(ps aux | grep -E "$PATTERN" | grep -v grep | awk '{print $2}')

if [[ -z "$pids" ]]; then
  echo "No ROS2 processes found."
  exit 0
fi

echo "Stopping ROS2 daemon..."
ros2 daemon stop 2>/dev/null
pkill -f ros2_daemon 2>/dev/null
echo "Done."

for pid in $pids; do
  info=$(ps -p $pid -o pid,pgid,command --no-headers 2>/dev/null) || continue
  echo "$info"
  if [[ "$MODE" == "force" ]]; then
    answer="y"
  else
    read -p "Kill PID $pid? (y/n): " answer
  fi
  if [[ "$answer" == "y" ]]; then
    pgid=$(echo "$info" | awk '{print $2}')
    if [[ "$pid" == "$pgid" ]]; then
      kill -TERM -"$pid" 2>/dev/null && echo "Killed process group $pid"
    else
      kill -TERM "$pid" 2>/dev/null && echo "Killed process $pid"
    fi
  fi
done
