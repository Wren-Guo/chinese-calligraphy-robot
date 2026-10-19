#!/usr/bin/env python3
"""示教变换点：记录笔尖的 (table_x, table_y, base_x, base_y, base_z)，单位米。

用法（笔尖放到纸面某个已知点，提供该点的桌面坐标 table x/y，单位米）：
    python3 teach_transform.py --tx 0.404 --ty 0.099
建议 4 个点：纸的左上、右上、左下、右下（或中心+三角）。
"""
import json, math, os, sys
import rclpy
from rclpy.node import Node
from fairino_msgs.srv import RemoteCmdInterface

TCP = (-62.241, 163.157, 161.786)  # 工具偏移 mm


def rpy2R(rx, ry, rz):
    rx, ry, rz = map(math.radians, (rx, ry, rz))
    cx, cy, cz = math.cos(rx), math.cos(ry), math.cos(rz)
    sx, sy, sz = math.sin(rx), math.sin(ry), math.sin(rz)
    return [[cz*cy, cz*sy*sx-sz*cx, cz*sy*cx+sz*sx],
            [sz*cy, sz*sy*sx+cz*cx, sz*sy*cx-cz*sx],
            [-sy, cy*sx, cy*cx]]


class N(Node):
    def __init__(self):
        super().__init__('teach_transform')
        self.cli = self.create_client(RemoteCmdInterface, '/fairino_remote_command_service')
        while not self.cli.wait_for_service(timeout_sec=5.0):
            pass

    def call(self, c):
        req = RemoteCmdInterface.Request(); req.cmd_str = c
        f = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, f, timeout_sec=10.0)
        return f.result().cmd_res if f.done() else 'TIMEOUT'


if '--tx' not in sys.argv or '--ty' not in sys.argv:
    print('用法: teach_transform.py --tx 0.404 --ty 0.099')
    sys.exit(1)
tx = float(sys.argv[sys.argv.index('--tx') + 1])
ty = float(sys.argv[sys.argv.index('--ty') + 1])

rclpy.init()
n = N()
# 必须用 GetActualToolFlangePose：GetActualTCPPose 在工具坐标生效时返回的是笔尖(TCP)，
# 再叠加 R@TCP 会多算 238mm，标定会整体错掉。
res = n.call('GetActualToolFlangePose(1)')
parts = res.split(',')
fx, fy, fz = float(parts[1]), float(parts[2]), float(parts[3])
rx, ry, rz = float(parts[4]), float(parts[5]), float(parts[6])
R = rpy2R(rx, ry, rz)
off = [sum(R[i][j] * TCP[j] for j in range(3)) for i in range(3)]
bx = round((fx + off[0]) / 1000.0, 4)
by = round((fy + off[1]) / 1000.0, 4)
bz = round((fz + off[2]) / 1000.0, 4)

path = '/home/trump/hanzi_task/transform_points.json'
pts = json.load(open(path, encoding='utf-8')) if os.path.exists(path) else []
pts.append({'tx': tx, 'ty': ty, 'bx': bx, 'by': by, 'bz': bz,
            'fx': round(fx / 1000.0, 4), 'fy': round(fy / 1000.0, 4),
            'fz': round(fz / 1000.0, 4),
            'rx': round(rx, 3), 'ry': round(ry, 3), 'rz': round(rz, 3)})
json.dump(pts, open(path, 'w', encoding='utf-8'), ensure_ascii=False, indent=2)
print(f'已记录第 {len(pts)} 点: table=({tx:.3f},{ty:.3f}) -> base=({bx:.3f},{by:.3f},{bz:.3f}) m')
n.destroy_node()
rclpy.shutdown()
