#!/usr/bin/env python3
"""四点法标定工具偏移（笔长）。不用 WebApp。

用法：
  python3 tcp_cal.py reset          # 清空，重新开始
  python3 tcp_cal.py record         # 记录当前姿态（笔尖顶住固定点时用）
  python3 tcp_cal.py show           # 查看已记录
  python3 tcp_cal.py solve          # 解算并写入 hanzi_config.json 的 tool_tcp

原理：笔尖固定在空间同一点 P，法兰位姿为 (p_i, R_i)，则
      p_i + R_i · t = P     （t = 法兰坐标系下的笔尖偏移）
      4 个姿态 = 12 个方程，解 6 个未知数 (t, P)，最小二乘。
"""
import json, math, os, sys, shutil, datetime

BASE = '/home/trump/hanzi_task'
DAT = f'{BASE}/tcp_points.json'
CFG = f'{BASE}/hanzi_config.json'


def rpy2R(rx, ry, rz):
    """与 teach_transform.py / hanzi_executor.py 完全一致的姿态换算。"""
    rx, ry, rz = map(math.radians, (rx, ry, rz))
    cx, cy, cz = math.cos(rx), math.cos(ry), math.cos(rz)
    sx, sy, sz = math.sin(rx), math.sin(ry), math.sin(rz)
    return [[cz * cy, cz * sy * sx - sz * cx, cz * sy * cx + sz * sx],
            [sz * cy, sz * sy * sx + cz * cx, sz * sy * cx - cz * sx],
            [-sy, cy * sx, cy * cx]]


def load():
    if os.path.exists(DAT):
        return json.load(open(DAT, encoding='utf-8'))
    return []


def save(pts):
    json.dump(pts, open(DAT, 'w', encoding='utf-8'), ensure_ascii=False, indent=2)


def cmd_record():
    import rclpy
    from rclpy.node import Node
    from fairino_msgs.srv import RemoteCmdInterface

    class N(Node):
        def __init__(self):
            super().__init__('tcp_cal_record')
            self.cli = self.create_client(RemoteCmdInterface, '/fairino_remote_command_service')
            if not self.cli.wait_for_service(timeout_sec=10.0):
                print('✗ 命令服务不可用，先跑 clean_server.sh')
                sys.exit(1)

        def call(self, c, t=10.0):
            r = RemoteCmdInterface.Request(); r.cmd_str = c
            f = self.cli.call_async(r)
            rclpy.spin_until_future_complete(self, f, timeout_sec=t)
            return f.result().cmd_res if f.done() else 'TIMEOUT'

    rclpy.init()
    n = N()
    n.call('SetToolCoord(1,0,0,0,0,0,0)')     # 清零工具坐标，保证读的是法兰
    import time
    time.sleep(0.4)

    fl = n.call('GetActualToolFlangePose(1)').split(',')
    js = n.call('GetActualJointPosDegree(1)').split(',')
    n.destroy_node(); rclpy.shutdown()
    if fl[0].strip() != '0' or js[0].strip() != '0':
        print('✗ 读取失败:', fl[0], js[0]); sys.exit(1)

    p = [float(v) for v in fl[1:7]]
    j = [round(float(v), 3) for v in js[1:7]]
    pts = load()
    pts.append({'flange': [round(v, 4) for v in p], 'joints': j})
    save(pts)
    print(f'✓ 已记录第 {len(pts)} 点')
    print(f'   法兰位置 = ({p[0]:.2f}, {p[1]:.2f}, {p[2]:.2f}) mm')
    print(f'   法兰姿态 = ({p[3]:.2f}, {p[4]:.2f}, {p[5]:.2f}) °')
    print(f'   关节     = {j}')
    if len(pts) >= 2:
        print()
        print('  各点姿态差异（越大越好，用于检查你换了足够不同的姿态）：')
        for i in range(1, len(pts)):
            a, b = pts[0]['flange'], pts[i]['flange']
            d = max(abs(a[k] - b[k]) for k in (3, 4, 5))
            print(f'    点1 vs 点{i+1}: 最大姿态差 {d:.1f}°')
    if len(pts) < 4:
        print(f'\n   还需要 {4-len(pts)} 个不同姿态的点')
    else:
        print('\n   已够 4 点，可以解算了：python3 tcp_cal.py solve')


def cmd_solve():
    pts = load()
    if len(pts) < 4:
        print(f'✗ 只记录了 {len(pts)} 点，至少需要 4 点'); return
    # 姿态差异检查
    worst = 0.0
    for i in range(1, len(pts)):
        d = max(abs(pts[0]['flange'][k] - pts[i]['flange'][k]) for k in (3, 4, 5))
        worst = max(worst, d)
    print(f'姿态最大差异 = {worst:.1f}°  ' + ('✓' if worst > 15 else '⚠ 太小，解算会不稳'))

    # 组装 A x = b ：每个姿态 3 行，未知数 [tx,ty,tz,Px,Py,Pz]
    A, b = [], []
    for pt in pts:
        f = pt['flange']
        p = [f[0], f[1], f[2]]
        R = rpy2R(f[3], f[4], f[5])
        for r in range(3):
            row = [R[r][0], R[r][1], R[r][2], -1.0 if r == 0 else 0.0,
                   -1.0 if r == 1 else 0.0, -1.0 if r == 2 else 0.0]
            A.append(row)
            b.append(-p[r])
    m, ncol = len(A), 6
    # 正规方程 A^T A x = A^T b
    M = [[sum(A[k][i] * A[k][j] for k in range(m)) for j in range(ncol)] +
         [sum(A[k][i] * b[k] for k in range(m))] for i in range(ncol)]
    for col in range(ncol):
        piv = max(range(col, ncol), key=lambda r: abs(M[r][col]))
        M[col], M[piv] = M[piv], M[col]
        pv = M[col][col]
        if abs(pv) < 1e-12:
            print('✗ 方程病态，请用差异更大的姿态重标'); return
        for j in range(col, ncol + 1):
            M[col][j] /= pv
        for r in range(ncol):
            if r != col:
                fac = M[r][col]
                for j in range(col, ncol + 1):
                    M[r][j] -= fac * M[col][j]
    x = [M[i][ncol] for i in range(ncol)]
    t = x[0:3]
    P = x[3:6]

    # 残差
    errs = []
    for pt in pts:
        f = pt['flange']
        R = rpy2R(f[3], f[4], f[5])
        tip = [f[k] + sum(R[k][j] * t[j] for j in range(3)) for k in range(3)]
        errs.append(math.dist(tip, P))
    rms = math.sqrt(sum(e * e for e in errs) / len(errs))

    print()
    print(f'解算结果：')
    print(f'  笔尖偏移 t (法兰坐标系) = ({t[0]:.3f}, {t[1]:.3f}, {t[2]:.3f}) mm')
    print(f'  笔长 |t| = {math.sqrt(sum(v*v for v in t)):.2f} mm')
    print(f'  参考点 P (基座坐标系)   = ({P[0]:.2f}, {P[1]:.2f}, {P[2]:.2f}) mm')
    print(f'  残差 RMS = {rms:.3f} mm  ' + ('✓ 很好' if rms < 1.0 else ('△ 一般' if rms < 3 else '✗ 偏大，建议重标')))

    cfg = json.load(open(CFG, encoding='utf-8'))
    old = cfg.get('tool_tcp', {})
    print()
    print(f'  配置原值: {old}')
    if '--no-write' in sys.argv:
        print('  (--no-write，未写入)'); return
    shutil.copy(CFG, CFG + '.bak_tcp_' + datetime.datetime.now().strftime('%H%M%S'))
    cfg['tool_tcp'] = {'tool_id': 1, 'x': round(t[0], 3), 'y': round(t[1], 3), 'z': round(t[2], 3)}
    json.dump(cfg, open(CFG, 'w', encoding='utf-8'), ensure_ascii=False, indent=2)
    print(f'✓ 已写入 tool_tcp = {cfg["tool_tcp"]}')


if __name__ == '__main__':
    mode = sys.argv[1] if len(sys.argv) > 1 else 'show'
    if mode == 'reset':
        if os.path.exists(DAT):
            shutil.copy(DAT, DAT + '.bak_' + datetime.datetime.now().strftime('%H%M%S'))
        save([])
        print('✓ 已清空，可以开始记录第 1 点')
    elif mode == 'record':
        cmd_record()
    elif mode == 'show':
        for i, pt in enumerate(load()):
            f = pt['flange']
            print(f'  点{i+1}: 法兰({f[0]:.2f}, {f[1]:.2f}, {f[2]:.2f}) rpy=({f[3]:.2f},{f[4]:.2f},{f[5]:.2f})')
    elif mode == 'solve':
        cmd_solve()
    else:
        print(__doc__)
