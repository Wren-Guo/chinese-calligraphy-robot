#!/usr/bin/env python3
"""联合解算：同时求出「笔长偏移 t」和「桌面->基座 仿射 A,b」。

数据：每个标定点给出 桌面坐标 u=(tx,ty)、法兰位置 p=(fx,fy,fz)、法兰姿态 R(rx,ry,rz)
模型：笔尖 = A·u + b  且  笔尖 = p + R·t
  =>  (R·t)_x - (a00·tx + a01·ty) - b0 = -px
      (R·t)_y - (a10·tx + a11·ty) - b1 = -py
未知数 9 个: t(3) + a00,a01,a10,a11(4) + b0,b1(2)
（z 方向由 z_heights 单独管，不参与本解算）

用法: python3 solve_joint.py            # 只看结果
      python3 solve_joint.py --write    # 写回 hanzi_config.json
"""
import json, math, os, sys, shutil, datetime

BASE = '/home/trump/hanzi_task'
PTS = f'{BASE}/transform_points.json'
CFG = f'{BASE}/hanzi_config.json'


def rpy2R(rx, ry, rz):
    rx, ry, rz = map(math.radians, (rx, ry, rz))
    cx, cy, cz = math.cos(rx), math.cos(ry), math.cos(rz)
    sx, sy, sz = math.sin(rx), math.sin(ry), math.sin(rz)
    return [[cz*cy, cz*sy*sx - sz*cx, cz*sy*cx + sz*sx],
            [sz*cy, sz*sy*sx + cz*cx, sz*sy*cx - cz*sx],
            [-sy, cy*sx, cy*cx]]


pts = json.load(open(PTS, encoding='utf-8'))
print(f'标定点 {len(pts)} 个：')
for i, p in enumerate(pts):
    if 'fx' not in p:
        print(f'  点{i+1}: ✗ 缺少法兰位姿字段（该点是补丁之前记的），请重标')
        sys.exit(1)
    print(f'  点{i+1}: table=({p["tx"]:+.4f},{p["ty"]:+.4f})  '
          f'flange=({p["fx"]*1000:+.1f},{p["fy"]*1000:+.1f},{p["fz"]*1000:+.1f})mm  '
          f'rpy=({p["rx"]:+.2f},{p["ry"]:+.2f},{p["rz"]:+.2f})')

rpy = [(p['rx'], p['ry'], p['rz']) for p in pts]
spread = max(max(abs(a[k]-b[k]) for k in range(3)) for a in rpy for b in rpy)
print(f'\n各点姿态最大差异 = {spread:.1f}°  ' +
      ('✓ 足够分离笔长' if spread > 8 else '⚠ 姿态太接近，笔长会解不准（建议标点时故意换换手腕角度）'))
if len(pts) < 5:
    print(f'⚠ 只有 {len(pts)} 点，未知数 9 个需要至少 5 点才够（建议 6~8 点）')

# 组装最小二乘：未知数 [t0,t1,t2, a00,a01,a10,a11, b0,b1]
rows, rhs = [], []
for p in pts:
    R = rpy2R(p['rx'], p['ry'], p['rz'])
    px, py = p['fx'] * 1000.0, p['fy'] * 1000.0
    tx, ty = p['tx'], p['ty']
    # x 方程
    rows.append([R[0][0], R[0][1], R[0][2], -tx, -ty, 0, 0, -1, 0]); rhs.append(-px)
    # y 方程
    rows.append([R[1][0], R[1][1], R[1][2], 0, 0, -tx, -ty, 0, -1]); rhs.append(-py)

m, n = len(rows), 9
M = [[sum(rows[k][i]*rows[k][j] for k in range(m)) for j in range(n)] +
     [sum(rows[k][i]*rhs[k] for k in range(m))] for i in range(n)]
for col in range(n):
    piv = max(range(col, n), key=lambda r: abs(M[r][col]))
    M[col], M[piv] = M[piv], M[col]
    pv = M[col][col]
    if abs(pv) < 1e-12:
        print('✗ 方程奇异：姿态变化不足或点太集中'); sys.exit(1)
    for j in range(col, n+1):
        M[col][j] /= pv
    for r in range(n):
        if r != col:
            f = M[r][col]
            for j in range(col, n+1):
                M[r][j] -= f*M[col][j]
x = [M[i][n] for i in range(n)]
t = x[0:3]
A = [[x[3], x[4]], [x[5], x[6]]]
b = [x[7], x[8]]

errs = []
for p in pts:
    R = rpy2R(p['rx'], p['ry'], p['rz'])
    tipx = p['fx']*1000 + sum(R[0][j]*t[j] for j in range(3))
    tipy = p['fy']*1000 + sum(R[1][j]*t[j] for j in range(3))
    ex = tipx - (A[0][0]*p['tx'] + A[0][1]*p['ty'] + b[0])
    ey = tipy - (A[1][0]*p['tx'] + A[1][1]*p['ty'] + b[1])
    errs.append(math.hypot(ex, ey))
rms = math.sqrt(sum(e*e for e in errs)/len(errs))

print()
print(f'=== 解算结果 ===')
print(f'  笔尖偏移 t = ({t[0]:+.3f}, {t[1]:+.3f}, {t[2]:+.3f}) mm   笔长 |t| = {math.sqrt(sum(v*v for v in t)):.2f} mm')
print(f'  仿射 A = [[{A[0][0]:+.6f}, {A[0][1]:+.6f}], [{A[1][0]:+.6f}, {A[1][1]:+.6f}]]')
print(f'  平移 b = ({b[0]:+.2f}, {b[1]:+.2f}) mm')
det = A[0][0]*A[1][1] - A[0][1]*A[1][0]
print(f'  行列式 = {det:+.4f}   缩放 ≈ {math.sqrt(abs(det)):.4f}')
print(f'  残差 RMS = {rms:.2f} mm  ' + ('✓ 很好' if rms < 3 else ('△ 一般' if rms < 8 else '✗ 偏大')))
print()
print('  各点残差: ' + ', '.join(f'{e:.1f}' for e in errs) + ' mm')

cfg = json.load(open(CFG, encoding='utf-8'))
print()
print(f'  配置现值: tool_tcp={cfg["tool_tcp"]}')
print(f'           table_transform={cfg["table_transform"]}')
if '--write' not in sys.argv:
    print('\n（未写入，加 --write 才写回）')
    sys.exit(0)

shutil.copy(CFG, CFG + '.bak_joint_' + datetime.datetime.now().strftime('%H%M%S'))
cfg['tool_tcp'] = {'tool_id': 1, 'x': round(t[0], 3), 'y': round(t[1], 3), 'z': round(t[2], 3)}
# 注意单位：A、b 解出来是 mm/桌面单位，配置里要求 米/桌面单位 -> 除以 1000
cfg['table_transform'] = {'a00': round(A[0][0] / 1000.0, 8), 'a01': round(A[0][1] / 1000.0, 8),
                          'b0': round(b[0] / 1000.0, 8),
                          'a10': round(A[1][0] / 1000.0, 8), 'a11': round(A[1][1] / 1000.0, 8),
                          'b1': round(b[1] / 1000.0, 8)}
json.dump(cfg, open(CFG, 'w', encoding='utf-8'), ensure_ascii=False, indent=2)
print('\n✓ 已写回 tool_tcp + table_transform')
