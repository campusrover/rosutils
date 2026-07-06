#!/bin/bash
echo "[running ~/rosutils/ros2_dome_bashrc.bash]"

# Paths consistent across all DOME-family computers (robots and remotes).
# Assumes consistent repo layout: ~/ros2_ws/src/dome_*/

export DOME_CAMERA=oak
export DOME_PIPER_MODEL_PATH=~/ros2_ws/src/dome_control/piper_model/en_US-lessac-medium.onnx
export DOME_PIPER_BIN=~/ros2_ws/src/dome_control/bin/piper/piper
export DOME_VOICE_CONFIG=~/ros2_ws/src/dome_voice/config/voice_config.yaml
export DOME_VISION_CONFIG=~/ros2_ws/src/dome_vision/dome_vision_ros/config/roboflow_oak.yaml
export DOME_SPIN_SURVEY_MAP=$DOME_HOME/spin_survey_map.json
