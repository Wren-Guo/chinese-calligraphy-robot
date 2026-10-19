#!/bin/bash
# Restart the command server, then stop and disable the arm.
# Called by safe_stop.sh. Paths are relative to this script.
set +u

HERE="$(cd "$(dirname "$0")" && pwd)"

source /opt/ros/humble/setup.bash
source "$HOME/fr5_ros2_ws/install/setup.bash"

pkill -f 'ros2_cmd_serve[r]' 2>/dev/null
nohup ros2 run fairino_hardware_v3_9_8 ros2_cmd_server > /tmp/cmd_server.log 2>&1 &
sleep 7

python3 "$HERE/stop_arm.py"