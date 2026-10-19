#!/bin/bash
# 启动命令服务（日志丢弃，setsid 常驻）
source /opt/ros/humble/setup.bash
source ~/fr5_ros2_ws/install/setup.bash
pkill -9 -f 'ros2_cmd_serve[r]' 2>/dev/null
sleep 2
setsid nohup ros2 run fairino_hardware_v3_9_8 ros2_cmd_server \
  > /dev/null 2>&1 < /dev/null &
sleep 10
echo "  cmd_server 实例数: $(pgrep -f 'ros2_cmd_serve[r]' | wc -l)"
