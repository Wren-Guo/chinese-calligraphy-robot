#!/usr/bin/env python3
"""记录一个示教位置到 positions.json。

用法：
    python3 record_position.py <位置名> [--config /path/positions.json]

会读取当前关节角并写入 json。之后 run_dot_sequence.py 就会用新位置。
"""

import json
import sys

import rclpy
from rclpy.node import Node
from fairino_msgs.srv import RemoteCmdInterface


class R(Node):
    def __init__(self):
        super().__init__("record_position")
        self.cli = self.create_client(RemoteCmdInterface, "/fairino_remote_command_service")

    def call(self, cmd, timeout=5.0):
        if not self.cli.wait_for_service(timeout_sec=3.0):
            return None
        req = RemoteCmdInterface.Request()
        req.cmd_str = cmd
        fut = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        return fut.result().cmd_res if fut.done() else None


def main():
    name = sys.argv[1]
    cfg_path = "/home/trump/dot_task/positions.json"
    if "--config" in sys.argv:
        cfg_path = sys.argv[sys.argv.index("--config") + 1]

    rclpy.init()
    n = R()
    res = n.call("GetActualJointPosDegree(1)")
    parts = res.split(",")
    if parts[0] != "0":
        print("读关节角失败:", res)
        return
    joints = [round(float(v), 3) for v in parts[1:7]]

    try:
        data = json.load(open(cfg_path, encoding="utf-8"))
    except (FileNotFoundError, json.JSONDecodeError):
        data = {}
    data[name] = joints
    json.dump(data, open(cfg_path, "w", encoding="utf-8"), ensure_ascii=False, indent=2)
    print(f"已记录 {name}: {joints} -> {cfg_path}")

    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
