#!/bin/bash
# 三遍连写 —— 稳健版（执行器前台运行，用退出码判断，不用 pgrep 猜）
# 用法: bash run_three_seq.sh [诗号]
set +u
source /opt/ros/humble/setup.bash
source /home/trump/fr5_ros2_ws/install/setup.bash

BASE=/home/trump/hanzi_task
CFG=$BASE/hanzi_config.json
POEM="${1:-1}"

# 前置检查：不允许有残留执行器
if pgrep -f 'hanzi_executor[.]py' >/dev/null; then
  echo "!! 已有执行器在运行，请先 safe_stop.sh"
  exit 1
fi

# 确保命令服务在（自检一次，不重启）
if ! pgrep -f 'ros2_cmd_serve[r]' >/dev/null; then
  echo "命令服务不在，启动中…"
  bash "$BASE/start_server.sh"
fi

set_config() {   # $1=skip_pickup  $2=start  $3=stop
  python3 - "$1" "$2" "$3" <<'EOF'
import json, sys
sp = sys.argv[1].lower() == 'true'
st = int(sys.argv[2]); so = int(sys.argv[3])
p = '/home/trump/hanzi_task/hanzi_config.json'
c = json.load(open(p, encoding='utf-8'))
c['flow']['skip_pickup'] = sp
c['flow']['start_from_character'] = st
c['flow']['stop_after_character'] = so
c['flow']['dip_ink'] = True
json.dump(c, open(p, 'w', encoding='utf-8'), ensure_ascii=False, indent=2)
EOF
}

run_pass() {     # $1=遍号  $2=skip_pickup
  echo
  echo "############ 第 $1/3 遍 (skip_pickup=$2) ############"
  set_config "$2" 0 20
  # 每遍前：清故障 + 使能（幂等，安全）
  python3 - <<'EOF'
import time, rclpy
from rclpy.node import Node
from fairino_msgs.srv import RemoteCmdInterface
class N(Node):
    def __init__(self):
        super().__init__('prep')
        self.cli = self.create_client(RemoteCmdInterface, '/fairino_remote_command_service')
        self.cli.wait_for_service(timeout_sec=10.0)
    def call(self, c, t=12.0):
        r = RemoteCmdInterface.Request(); r.cmd_str = c
        f = self.cli.call_async(r)
        rclpy.spin_until_future_complete(self, f, timeout_sec=t)
        return f.result().cmd_res if f.done() else 'TIMEOUT'
rclpy.init(); n = N()
print('  ServoMoveEnd:', n.call('ServoMoveEnd(0)'))
print('  清故障:', n.call('ResetAllError()'))
time.sleep(0.6)
print('  使能:', n.call('RobotEnable(1)'))
time.sleep(0.8)
print('  错误码:', n.call('GetRobotErrorCode()'))
n.destroy_node(); rclpy.shutdown()
EOF
  # 前台运行执行器（阻塞直到结束，用退出码判断）
  : > /tmp/hanzi_run.log
  python3 "$BASE/hanzi_executor.py" --config "$CFG" --poem "$POEM" 2>&1 | tee /tmp/hanzi_run.log | grep -avE 'RTPS|fastrtps'
  RC=${PIPESTATUS[0]}
  echo "  ==> 第 $1 遍退出码: $RC"
  if grep -qa '全部书写完成' /tmp/hanzi_run.log; then
    echo "  ==> 第 $1 遍：完整写完 ✓"
    return 0
  else
    echo "  ==> 第 $1 遍：未完成 ✗（看 /tmp/hanzi_run.log）"
    return 1
  fi
}

# 第1遍：完整取笔
run_pass 1 false || { echo "第1遍失败，停止（不继续，避免连锁问题）"; exit 2; }

# 第2、3遍：不松手
run_pass 2 true || { echo "第2遍失败，停止"; exit 2; }
run_pass 3 true || { echo "第3遍失败，停止"; exit 2; }

echo
echo "############ 三遍全部完成 ✓ 笔仍在手上 ############"
