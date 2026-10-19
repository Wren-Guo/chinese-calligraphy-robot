#!/usr/bin/env python3
"""安全停止机械臂：ServoMoveEnd -> StopMotion -> ResetAllError -> 禁能。

被 stop_arm.sh 调用；safe_stop.sh 再调用 stop_arm.sh。
用法: python3 stop_arm.py
"""
import time
import rclpy
from rclpy.node import Node
from fairino_msgs.srv import RemoteCmdInterface


class Stopper(Node):
    def __init__(self):
        super().__init__('stop_arm')
        self.cli = self.create_client(
            RemoteCmdInterface, '/fairino_remote_command_service')
        self.cli.wait_for_service(timeout_sec=10.0)

    def call(self, cmd, timeout=10.0):
        req = RemoteCmdInterface.Request()
        req.cmd_str = cmd
        fut = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        return fut.result().cmd_res if fut.done() else 'TIMEOUT'


def main():
    rclpy.init()
    node = Stopper()
    steps = [
        ('ServoMoveEnd(0)', '结束伺服'),
        ('StopMotion()',    '停止运动'),
        ('ResetAllError()', '清故障'),
        ('RobotEnable(0)',  '禁能'),
    ]
    for cmd, label in steps:
        print(f'{label:8}: {node.call(cmd)}')
        time.sleep(0.3)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
