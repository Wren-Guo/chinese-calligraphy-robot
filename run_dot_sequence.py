#!/usr/bin/env python3
"""画龙点睛 任务序列（示教-回放，ServoJ 关节空间）。

流程固定，位置从 positions.json 读取（改位置只需改 json 或运行 record_position.py）。

用法：python3 run_dot_sequence.py [--dry-run] [--config /path/positions.json]
"""

import json
import sys
import time

import rclpy
from rclpy.node import Node
from fairino_msgs.srv import RemoteCmdInterface
from sensor_msgs.msg import JointState


# 固定流程（"抓握" 是机械手动作，其余是位置名）
FLOW = [
    "靠近笔前", "靠近笔", "抓握", "抬笔",
    "蘸墨", "抬墨", "点睛悬空", "点睛落笔", "点睛悬空", "抬笔",
]

HAND_NAMES = [
    'r_f_joint1_1','r_f_joint1_2','r_f_joint1_3','r_f_joint1_4',
    'r_f_joint2_1','r_f_joint2_2','r_f_joint2_3','r_f_joint2_4',
    'r_f_joint3_2','r_f_joint3_3','r_f_joint3_4',
    'r_f_joint4_1','r_f_joint4_2','r_f_joint4_3','r_f_joint4_4',
    'r_f_joint5_1','r_f_joint5_2','r_f_joint5_3','r_f_joint5_4',
]
HAND_GRASP = [
    1.894904, 0.215723, 0.644026, 0.644026,
    0.027925, 1.129577, 0.501259, 0.501259,
    1.215273, 0.625002, 0.625002,
    0.027925, 1.063080, 0.672650, 0.672650,
    0.027925, 0.834442, 0.644026, 0.644026,
]


class Seq(Node):
    def __init__(self, config):
        super().__init__("run_dot_sequence")
        self.cli = self.create_client(RemoteCmdInterface, "/fairino_remote_command_service")
        self.hand_pub = self.create_publisher(JointState, "/right_hand/joint_commands", 10)
        self.declare_parameter("duration", 4.0)
        self.declare_parameter("steps", 100)
        self.config = config

    def call(self, cmd, timeout=10.0):
        if not self.cli.wait_for_service(timeout_sec=3.0):
            return "NO_SERVICE"
        req = RemoteCmdInterface.Request()
        req.cmd_str = cmd
        fut = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        return fut.result().cmd_res if fut.done() else "TIMEOUT"

    def read_joints(self):
        res = self.call("GetActualJointPosDegree(1)")
        parts = res.split(",")
        if parts[0] != "0" or len(parts) < 7:
            return None
        return [float(v) for v in parts[1:7]]

    def servo_move_j(self, target, duration, steps):
        start = self.read_joints()
        if start is None:
            self.get_logger().error("读关节角失败")
            return False
        d = max(abs(target[k] - start[k]) for k in range(6))
        self.get_logger().info(f"移动: 当前 {[round(v,1) for v in start]} -> 目标 {target} 最大关节差 {d:.1f}°")

        r = self.call("ServoMoveStart(0)")
        if r.strip() != "0":
            self.get_logger().error(f"ServoMoveStart 失败: {r}")
            return False
        step_sleep = duration / steps
        for i in range(1, steps + 1):
            t = i / steps
            j = [start[k] + t * (target[k] - start[k]) for k in range(6)]
            cmd = (f"ServoJ({j[0]:.3f},{j[1]:.3f},{j[2]:.3f},{j[3]:.3f},"
                   f"{j[4]:.3f},{j[5]:.3f},0,0,0,0,0.008)")
            rr = self.call(cmd)
            if rr.strip() != "0" and i <= 2:
                self.get_logger().warn(f"ServoJ step {i}: {rr}")
            time.sleep(step_sleep)
        r2 = self.call("ServoMoveEnd(0)")
        self.get_logger().info(f"ServoMoveEnd: {r2}")
        return r2.strip() == "0"

    def grasp(self):
        m = JointState()
        m.name = list(HAND_NAMES)
        m.position = list(HAND_GRASP)
        for _ in range(3):
            self.hand_pub.publish(m)
            time.sleep(0.05)
        self.get_logger().info("机械手握爪指令已发布")


def main():
    dry = "--dry-run" in sys.argv
    cfg_path = "/home/trump/dot_task/positions.json"
    if "--config" in sys.argv:
        cfg_path = sys.argv[sys.argv.index("--config") + 1]
    positions = json.load(open(cfg_path, encoding="utf-8"))

    rclpy.init()
    n = Seq(cfg_path)
    duration = float(n.get_parameter("duration").value)
    steps = int(n.get_parameter("steps").value)

    for name in FLOW:
        if name == "抓握":
            if not dry:
                n.grasp()
                time.sleep(1.0)
            else:
                n.get_logger().info("[dry] 抓握")
            continue
        if name not in positions:
            n.get_logger().error(f"positions.json 缺少位置 {name}，停止")
            break
        target = positions[name]
        if dry:
            n.get_logger().info(f"[dry] 移动到 {name}: {target}")
            continue
        if not n.servo_move_j(target, duration, steps):
            n.get_logger().error(f"步骤 {name} 失败，停止")
            break
        time.sleep(0.5)

    n.get_logger().info("序列结束")
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
