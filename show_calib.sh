#!/bin/bash
echo '=== 1) 笔的 TCP 标定（笔尖相对法兰的偏移）==='
python3 -c "
import json
c=json.load(open('/home/trump/hanzi_task/hanzi_config.json',encoding='utf-8'))
t=c['tool_tcp']
print(f\"  tool_id={t.get('tool_id')}  x={t['x']:.3f} y={t['y']:.3f} z={t['z']:.3f} mm\")
import math
print(f'  笔长 = {math.sqrt(t[\"x\"]**2+t[\"y\"]**2+t[\"z\"]**2):.1f} mm')
"

echo
echo '=== 2) 桌面->基座 的标定（仿射）==='
python3 -c "
import json, math
c=json.load(open('/home/trump/hanzi_task/hanzi_config.json',encoding='utf-8'))
a=c['table_transform']
print(f\"  a00={a['a00']:+.6f} a01={a['a01']:+.6f} b0={a['b0']:+.6f}\")
print(f\"  a10={a['a10']:+.6f} a11={a['a11']:+.6f} b1={a['b1']:+.6f}\")
sx=math.hypot(a['a00'],a['a10']); sy=math.hypot(a['a01'],a['a11'])
ang=math.degrees(math.atan2(a['a10'],a['a00']))
print(f'  比例: x方向 {sx:.4f}  y方向 {sy:.4f}   (1=不缩放，说明桌面单位与米一致)')
print(f'  旋转角: {ang:.2f}°')
"

echo
echo '=== 3) 标定点与残差 ==='
python3 -c "
import json, math
pts=json.load(open('/home/trump/hanzi_task/transform_points.json',encoding='utf-8'))
c=json.load(open('/home/trump/hanzi_task/hanzi_config.json',encoding='utf-8'))
a=c['table_transform']
print(f'  标定点 {len(pts)} 个:')
res=[]
for p in pts:
    px=a['a00']*p['tx']+a['a01']*p['ty']+a['b0']
    py=a['a10']*p['tx']+a['a11']*p['ty']+a['b1']
    e=math.hypot(px-p['bx'], py-p['by'])*1000
    res.append(e)
    print(f\"    ({p['tx']:+.4f},{p['ty']:+.4f}) -> 残差 {e:.2f} mm\")
print(f'  平均残差 {sum(res)/len(res):.2f} mm, 最大 {max(res):.2f} mm')
"

echo
echo '=== 4) 高度标定 ==='
python3 -c "
import json
c=json.load(open('/home/trump/hanzi_task/hanzi_config.json',encoding='utf-8'))
print('  z_heights_mm:', {k:v for k,v in c['z_heights_mm'].items() if not k.startswith('_')})
z=c.get('z_offsets_mm',{})
print('  逐字偏移(第1~20字):', [z.get(str(i),0) for i in range(20)])
"

echo
echo '=== 5) 栅格（40格）==='
python3 -c "
import json
d=json.load(open('/home/trump/hanzi_task/poem1_trajectories.json',encoding='utf-8'))
cs=d['cells']
print(f'  格数: {len(cs)}')
for i in [0,1,9,10,19,20,39]:
    m=cs[i]['center_m']
    print(f\"    格{i+1:2d}: table=({m[0]:+.4f},{m[1]:+.4f})\")
"
