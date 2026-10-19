#!/bin/bash
# 示教纸面点：自动 source 环境 + 启动 cmd_server + 记录笔尖位置
source /opt/ros/humble/setup.bash
source ~/fr5_ros2_ws/install/setup.bash
pgrep -f 'ros2_cmd_serve[r]' >/dev/null || {
  nohup ros2 run fairino_hardware_v3_9_8 ros2_cmd_server > /tmp/cmd_server.log 2>&1 &
  sleep 6
}
python3 ~/hanzi_task/teach_z_points.py
