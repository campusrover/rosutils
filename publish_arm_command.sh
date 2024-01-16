function publish_arm_command() {
    local command=$1
    local value=$2

    if [ $# == 1 ]; then
        rostopic pub -1 /arm lino_msgs/ArmMsg "{command: '$command', arg1: 0.0, arg2: 0.0}"
    elif [ $# == 2 ]; then
        rostopic pub -1 /arm lino_msgs/ArmMsg "{command: '$command', arg1: $value, arg2: 0.0}"
    fi
}