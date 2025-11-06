for pid in $(ps aux | grep -E 'ros2|nav2|slam|rviz|gazebo' | grep -v grep | awk '{print $2}'); do
  info=$(ps -p $pid -o pid,pgid,command --no-headers)
  echo "$info"
  read -p "Kill PID $pid? (y/n): " answer
  if [ "$answer" = "y" ]; then
    pgid=$(echo $info | awk '{print $2}')
    if [ "$pid" = "$pgid" ]; then
      kill -TERM -$pid && echo "Killed process group $pid"
    else
      kill -TERM $pid && echo "Killed process $pid"
    fi
  fi
done
