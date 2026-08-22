#!/usr/bin/env bash
# Named repo workspaces managed by mr.sh.
#
# To add a workspace: set WORKSPACE_DIR[<name>] to its src directory and
# define a REPOS_<name> array listing the repo directories under it.
# Edit these lists by hand when repos are added/removed.

declare -A WORKSPACE_DIR

DEFAULT_WORKSPACE=ros2_ws

WORKSPACE_DIR[ros2_ws]="$HOME/ros2_ws/src"
REPOS_ros2_ws=(
  dome
  dome2
  dome_control
  dome_mission
  dome_nav
  dome_nav_msgs
  dome_semantic
  dome_semantic_msgs
  dome_telemetry
  dome_vision
  dome_voice
  explore
  metawtf
  ros2diag
  status_panel
)
