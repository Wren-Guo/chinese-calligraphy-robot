#!/usr/bin/env bash
# 画龙点睛任务一键运行
set -e
source /opt/ros/humble/setup.bash
source ~/fr5_ros2_ws/install/setup.bash

echo "== 重启命令服务 =="
pkill -f "ros2_cmd_serve[r]" 2>/dev/null || true
sleep 2
nohup ros2 run fairino_hardware_v3_9_8 ros2_cmd_server > /tmp/cmd_server.log 2>&1 &
sleep 8

ros2 service call /fairino_remote_command_service fairino_msgs/srv/RemoteCmdInterface "{cmd_str: 'ResetAllError()'}"
ros2 service call /fairino_remote_command_service fairino_msgs/srv/RemoteCmdInterface "{cmd_str: 'RobotEnable(1)'}"
ros2 service call /fairino_remote_command_service fairino_msgs/srv/RemoteCmdInterface "{cmd_str: 'SetSpeed(50)'}"
sleep 1

echo "== 清理并重启手节点（必须只有 1 个，否则抢 CAN 总线导致抓不住）=="
bash ~/hanzi_task/start_dexhand.sh
HAND_N=$(pgrep -af 'dexhand_ros.py' | grep -v pgrep | wc -l)
echo "手节点实例数: $HAND_N"
TFD=$(grep -ac 'TransmitFD failed' /tmp/dexhand.log 2>/dev/null; true)
echo "手节点 TransmitFD 错误数: $TFD"
if [ "$HAND_N" != "1" ]; then
  echo "!!! 手节点实例数不是 1，抓握会失败，请检查 !!!"
fi
if [ "$TFD" != "0" ]; then
  echo "!!! 手节点有 TransmitFD 错误（CAN 发送失败），抓握可能失败 !!!"
fi

echo "== 运行序列 =="
python3 /home/trump/dot_task/run_dot_sequence.py
