import json, math

CFG = '/home/trump/hanzi_task/hanzi_config.json'
PTS = '/home/trump/hanzi_task/transform_points.json'
cfg = json.load(open(CFG, encoding='utf-8'))
pts = json.load(open(PTS, encoding='utf-8'))
a = cfg['table_transform']
t = cfg['tool_tcp']
tcp = (t['x'], t['y'], t['z'])


def rpy2R(rx, ry, rz):
    rx, ry, rz = map(math.radians, (rx, ry, rz))
    cx, cy, cz = math.cos(rx), math.cos(ry), math.cos(rz)
    sx, sy, sz = math.sin(rx), math.sin(ry), math.sin(rz)
    return [[cz*cy, cz*sy*sx - sz*cx, cz*sy*cx + sz*sx],
            [sz*cy, sz*sy*sx + cz*cx, sz*sy*cx - cz*sx],
            [-sy, cy*sx, cy*cx]]


print(f'{"桌面坐标":^24} {"记录法兰":^22} {"+R×TCP 得笔尖":^22} {"桌面映射预测":^22} {"误差mm":>8}')
print('-' * 110)
res = []
for p in pts:
    R = rpy2R(p['rx'], p['ry'], p['rz'])
    tipx = p['bx'] + sum(R[0][j]*tcp[j] for j in range(3))/1000.0
    tipy = p['by'] + sum(R[1][j]*tcp[j] for j in range(3))/1000.0
    px = a['a00']*p['tx'] + a['a01']*p['ty'] + a['b0']
    py = a['a10']*p['tx'] + a['a11']*p['ty'] + a['b1']
    e = math.hypot(tipx - px, tipy - py) * 1000
    res.append(e)
    print(f'({p["tx"]:+.3f},{p["ty"]:+.3f})     '
          f'({p["bx"]*1000:+7.1f},{p["by"]*1000:+7.1f})   '
          f'({tipx*1000:+7.1f},{tipy*1000:+7.1f})   '
          f'({px*1000:+7.1f},{py*1000:+7.1f})   {e:7.2f}')
print()
print(f'平均残差 {sum(res)/len(res):.2f} mm   最大 {max(res):.2f} mm')
