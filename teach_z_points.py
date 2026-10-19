#!/usr/bin/env python3
"""示教纸面平面：记录笔尖当前基坐标 (bx, by, bz)，单位 mm。

用法：把笔尖放到纸面上某个点（刚好碰到纸），运行一次记录一个点。
建议 5 个点：纸的四个角 + 中心。每点运行一次：
    python3 teach_z_points.py
"""
import json, math, os
import rclpy
from rclpy.node import Node
from fairino_msgs.srv import RemoteCmdInterface

TCP = (-62.241, 163.157, 161.786)  # 工具偏移 mm（笔尖相对法兰）


def rpy2R(rx, ry, rz):
    rx, ry, rz = map(math.radians, (rx, ry, rz))
    cx, cy, cz = math.cos(rx), math.cos(ry), math.cos(rz)
    sx, sy, sz = math.sin(rx), math.sin(ry), math.sin(rz)
    return [[cz*cy, cz*sy*sx-sz*cx, cz*sy*cx+sz*sx],
            [sz*cy, sz*sy*sx+cz*cx, sz*sy*cx-cz*sx],
            [-sy, cy*sx, cy*cx]]


class N(Node):
    def __init__(self):
        super().__init__('teach_z_points')
        self.cli = self.create_client(RemoteCmdInterface, '/fairino_remote_command_service')
        while not self.cli.wait_for_service(timeout_sec=5.0):
            pass

    def call(self, c):
        req = RemoteCmdInterface.Request(); req.cmd_str = c
        f = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, f, timeout_sec=10.0)
        return f.result().cmd_res if f.done() else 'TIMEOUT'


rclpy.init()
n = N()
res = n.call('GetActualTCPPose(1)')  # 法兰 (x,y,z,rx,ry,rz) mm/deg
parts = res.split(',')
fx, fy, fz = float(parts[1]), float(parts[2]), float(parts[3])
rx, ry, rz = float(parts[4]), float(parts[5]), float(parts[6])
R = rpy2R(rx, ry, rz)
off = [sum(R[i][j] * TCP[j] for j in range(3)) for i in range(3)]
px, py, pz = fx + off[0], fy + off[1], fz + off[2]

path = '/home/trump/hanzi_task/paper_plane_points.json'
points = json.load(open(path, encoding='utf-8')) if os.path.exists(path) else []
points.append({'bx': round(px, 3), 'by': round(py, 3), 'bz': round(pz, 3)})
json.dump(points, open(path, 'w', encoding='utf-8'), ensure_ascii=False, indent=2)
print(f'已记录第 {len(points)} 点：笔尖 (bx={px:.2f}, by={py:.2f}, bz={pz:.2f}) mm')
n.destroy_node()
rclpy.shutdown()
