#!/usr/bin/env python3
"""用联合解算出的笔长，重新计算各标定点的笔尖高度 -> 得到书写高度 z_down/z_up。

前提：标定时笔尖是竖直且碰到纸面的（用户确认）。
"""
import json, math, shutil, datetime, sys

BASE = '/home/trump/hanzi_task'
PTS = f'{BASE}/transform_points.json'
CFG = f'{BASE}/hanzi_config.json'
LIFT = 43.8          # 抬笔高度差，沿用原配置


def rpy2R(rx, ry, rz):
    rx, ry, rz = map(math.radians, (rx, ry, rz))
    cx, cy, cz = math.cos(rx), math.cos(ry), math.cos(rz)
    sx, sy, sz = math.sin(rx), math.sin(ry), math.sin(rz)
    return [[cz*cy, cz*sy*sx - sz*cx, cz*sy*cx + sz*sx],
            [sz*cy, sz*sy*sx + cz*cx, sz*sy*cx - cz*sx],
            [-sy, cy*sx, cy*cx]]


cfg = json.load(open(CFG, encoding='utf-8'))
t = (cfg['tool_tcp']['x'], cfg['tool_tcp']['y'], cfg['tool_tcp']['z'])
pts = json.load(open(PTS, encoding='utf-8'))

print(f'当前 tool_tcp = {t}\n')
print(f'{"点":>3s} | {"桌面坐标":^22s} | {"法兰z(mm)":>10s} | {"笔尖z(mm)":>10s}')
print('-' * 60)
zs = []
for i, p in enumerate(pts):
    R = rpy2R(p['rx'], p['ry'], p['rz'])
    offz = sum(R[2][j] * t[j] for j in range(3))
    tipz = p['fz'] * 1000.0 + offz
    zs.append(tipz)
    print(f'{i+1:3d} | ({p["tx"]:+.4f},{p["ty"]:+.4f}) | {p["fz"]*1000:10.2f} | {tipz:10.2f}')

mean = sum(zs) / len(zs)
print(f'\n笔尖高度: 均值 {mean:.2f}  最小 {min(zs):.2f}  最大 {max(zs):.2f}  极差 {max(zs)-min(zs):.2f} mm')
print('（极差就是桌面/纸面的不平度；越小说明你顶得越一致、桌面越平）')

# 顺便看是否有系统性倾斜（拟合平面）
n = len(pts)
S = [[sum(p['tx']**2 for p in pts), sum(p['tx']*p['ty'] for p in pts), sum(p['tx'] for p in pts)],
     [sum(p['tx']*p['ty'] for p in pts), sum(p['ty']**2 for p in pts), sum(p['ty'] for p in pts)],
     [sum(p['tx'] for p in pts), sum(p['ty'] for p in pts), float(n)]]
rhs = [sum(pts[i]['tx']*zs[i] for i in range(n)), sum(pts[i]['ty']*zs[i] for i in range(n)), sum(zs)]
S = [S[i] + [rhs[i]] for i in range(3)]   # 每行补上右端项，凑成 3x4
for col in range(3):
    piv = max(range(col, 3), key=lambda r: abs(S[r][col]))
    S[col], S[piv] = S[piv], S[col]
    pv = S[col][col]
    for j in range(col, 4):
        S[col][j] /= pv
    for r in range(3):
        if r != col:
            f = S[r][col]
            for j in range(col, 4):
                S[r][j] -= f*S[col][j]
a, b, c0 = S[0][3], S[1][3], S[2][3]
print(f'平面拟合: z = {c0:.2f} + {a*1000:.2f}*tx + {b*1000:.2f}*ty  (mm)')
print(f'  即桌面在 0.6m 范围内倾斜约 {math.hypot(a, b)*0.6*1000:.1f} mm')

cfg['z_heights_mm']['down'] = round(mean + (1.0 if '--plus1' in sys.argv else 0.0), 2)
cfg['z_heights_mm']['up'] = round(cfg['z_heights_mm']['down'] + LIFT, 2)
if '--write' not in sys.argv:
    print(f'\n（未写入。将设置 down={cfg["z_heights_mm"]["down"]}, up={cfg["z_heights_mm"]["up"]}）')
    sys.exit(0)
shutil.copy(CFG, CFG + '.bak_z_' + datetime.datetime.now().strftime('%H%M%S'))
json.dump(cfg, open(CFG, 'w', encoding='utf-8'), ensure_ascii=False, indent=2)
print(f'\n✓ 已写入 z_heights_mm: down={cfg["z_heights_mm"]["down"]}, up={cfg["z_heights_mm"]["up"]}')
print('   （用均值，相当于把每个字都写在平均纸面上）')
