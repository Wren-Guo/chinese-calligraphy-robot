#!/bin/bash
# Safe stop: kill the executor, make sure the command server is alive,
# then stop and disable the arm.
set +u

HERE="$(cd "$(dirname "$0")" && pwd)"
source /opt/ros/humble/setup.bash
source "$HOME/fr5_ros2_ws/install/setup.bash"

echo '=== 1) Kill the executor ==='
pkill -9 -f 'hanzi_executor[.]py' 2>/dev/null
pkill -9 -f 'run_dot_sequence[.]py' 2>/dev/null
sleep 1
echo "  remaining: $(pgrep -f 'hanzi_executor[.]py|run_dot_sequence[.]py' | wc -l)"

echo '=== 2) Make sure the command server is up ==='
if ! pgrep -f 'ros2_cmd_serve[r]' >/dev/null; then
  echo '  server not running, starting...'
  nohup ros2 run fairino_hardware_v3_9_8 ros2_cmd_server > /tmp/cmd_server.log 2>&1 &
  sleep 10
fi
echo "  cmd_server processes: $(pgrep -f 'ros2_cmd_serve[r]' | wc -l)"

echo '=== 3) Stop and disable the arm ==='
bash "$HERE/stop_arm.sh" 2>&1 | tail -5