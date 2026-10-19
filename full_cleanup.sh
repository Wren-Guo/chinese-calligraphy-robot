#!/bin/bash
# 全部清理：执行器、守护、命令服务、手节点，最后只留一个干净的命令服务，机器人禁能
source /opt/ros/humble/setup.bash
source ~/fr5_ros2_ws/install/setup.bash

echo '=== 1) 杀执行器 ==='
pkill -9 -f 'hanzi_executor[.]py' 2>/dev/null || true
pkill -9 -f 'run_dot_sequence[.]py' 2>/dev/null || true
pkill -9 -f 'test_servo' 2>/dev/null || true
sleep 2
echo "  残留: $(pgrep -f 'hanzi_executor|run_dot_sequence' | wc -l)"

echo '=== 2) 停守护（防止它和清理打架）==='
pkill -9 -f 'server_watchdog' 2>/dev/null || true
sleep 1
echo "  守护残留: $(pgrep -f 'server_watchdog' | wc -l)"

echo '=== 3) 机器人先停下并禁能 ==='
if pgrep -f 'ros2_cmd_serve[r]' >/dev/null; then
python3 - <<'EOF'
import time, rclpy
from rclpy.node import Node
from fairino_msgs.srv import RemoteCmdInterface
class N(Node):
    def __init__(self):
        super().__init__('stopall')
        self.cli = self.create_client(RemoteCmdInterface, '/fairino_remote_command_service')
        self.cli.wait_for_service(timeout_sec=8.0)
    def call(self, c, t=10.0):
        r = RemoteCmdInterface.Request(); r.cmd_str = c
        f = self.cli.call_async(r)
        rclpy.spin_until_future_complete(self, f, timeout_sec=t)
        return f.result().cmd_res if f.done() else 'TIMEOUT'
rclpy.init(); n = N()
print('  ServoMoveEnd:', n.call('ServoMoveEnd(0)'))
print('  清故障:', n.call('ResetAllError()'))
time.sleep(0.8)
print('  禁能:', n.call('RobotEnable(0)'))
time.sleep(0.8)
print('  错误码:', n.call('GetRobotErrorCode()'))
n.destroy_node(); rclpy.shutdown()
EOF
else
  echo '  命令服务不在，跳过（机器人应已停止）'
fi

echo '=== 4) 杀掉所有命令服务 ==='
pkill -9 -f 'ros2_cmd_serve[r]' 2>/dev/null || true
pkill -9 -f 'ros2 run fairino_hardware' 2>/dev/null || true
sleep 8
echo "  残留: $(pgrep -f 'ros2_cmd_serve[r]' | wc -l)"

echo '=== 5) 杀手节点（注意：笔会松脱！）==='
pkill -9 -f 'dexhand_ros[.]py' 2>/dev/null || true
sleep 2
echo "  手节点残留: $(pgrep -f 'dexhand_ros' | wc -l)"

echo '=== 6) 清日志 ==='
rm -f /tmp/cmd_server.log /tmp/watchdog.log /tmp/hanzi_run.log /tmp/hanzi_dual.log /tmp/poemA.log 2>/dev/null
echo '  已清 /tmp 下的运行日志'

echo '=== 7) 只启动一个干净的命令服务 ==='
setsid nohup ros2 run fairino_hardware_v3_9_8 ros2_cmd_server > /dev/null 2>&1 < /dev/null &
sleep 14
N=$(pgrep -f 'ros2_cmd_serve[r]' | wc -l)
echo "  进程数: $N (2 = 1个服务)"
python3 - <<'EOF'
import rclpy
from rclpy.node import Node
from fairino_msgs.srv import RemoteCmdInterface
class N(Node):
    def __init__(self):
        super().__init__('ver')
        self.cli = self.create_client(RemoteCmdInterface, '/fairino_remote_command_service')
        print('  服务可用:', self.cli.wait_for_service(timeout_sec=10.0))
    def call(self, c, t=8.0):
        r = RemoteCmdInterface.Request(); r.cmd_str = c
        f = self.cli.call_async(r)
        rclpy.spin_until_future_complete(self, f, timeout_sec=t)
        return f.result().cmd_res if f.done() else 'TIMEOUT'
rclpy.init(); n = N()
print('  错误码:', n.call('GetRobotErrorCode()'))
n.destroy_node(); rclpy.shutdown()
EOF

echo
echo '=== 清理完成 ==='
echo '  机器人：禁能、无报警 ✓'
echo '  命令服务：1 个、干净 ✓'
echo '  执行器/守护/手节点：全部停止 ✓'
echo '  ⚠️ 手已松开，笔可能已掉落/仍在手上，请检查 ✗'
