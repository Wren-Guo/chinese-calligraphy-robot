#!/usr/bin/env bash
# 确保「只有 1 个」手节点，并用 setsid 放到独立会话，SSH 断开也不会被杀
PAT='dexhand_ros[.]py'
count() { pgrep -f "$PAT" | wc -l; }

source /opt/ros/humble/setup.bash
source ~/dexhand_venv/bin/activate
cd ~/pyzlg_dexhand

echo "== 清理已有手节点（SIGTERM -> SIGKILL，最多 3 轮）=="
for i in 1 2 3; do
  n=$(count)
  [ "$n" = "0" ] && break
  echo "  第 $i 轮：仍有 $n 个，SIGTERM"
  pkill -f "$PAT" 2>/dev/null || true
  sleep 3
  if [ "$(count)" != "0" ]; then
    echo "  仍有 $(count) 个，SIGKILL"
    pkill -9 -f "$PAT" 2>/dev/null || true
    sleep 3
  fi
done
n=$(count)
echo "清理后剩余: $n"
if [ "$n" != "0" ]; then
  echo "!!! 杀不掉，请手动处理 !!!"; pgrep -af "$PAT"; exit 1
fi
echo "  等 3 秒释放 CAN…"
sleep 3

echo "== 用 setsid 启动唯一一个手节点（脱离 SSH 会话）=="
setsid nohup python examples/ros_node/dexhand_ros.py > /tmp/dexhand.log 2>&1 < /dev/null &
disown 2>/dev/null || true
sleep 10

n=$(count)
echo "启动后实例数: $n （期望 1）"
pgrep -af "$PAT"
echo "--- 日志尾部 ---"
tail -n 8 /tmp/dexhand.log
echo "--- TransmitFD 错误数 ---"
grep -ac 'TransmitFD failed' /tmp/dexhand.log 2>/dev/null || true
if [ "$n" != "1" ]; then
  echo "!!! 实例数不是 1 !!!"; exit 1
fi
echo "✓ 手节点就绪（独立会话，SSH 断开不会死）"
