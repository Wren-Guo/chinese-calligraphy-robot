# 任务2：写字（版本1）

## 这一版是什么
- 执行器：1570 行 Python（ROS 2 节点 + FR SDK）
- 笔画内：固定 16ms 周期 ServoJ + 弧长重采样 + 五次曲线
- 笔画间：**Bézier 弧线跳笔**（替代“抬笔→平移→落笔”三段，消除速度突变）
- 抓握：发 60 条指令（3s）+ 等待 3s（共 6s 给手合拢）
- 上提速度：40 mm/s（慢速，防止笔滑）

## 目录
```
程序/      hanzi_executor.py        执行器主程序
配置/      hanzi_config.json        全部参数（高度/速度/蘸墨/姿态）
           transform_points.json    5 点标定数据
轨迹/      poem1/2/3_trajectories.json   三首诗的笔画轨迹
           dual_trajectories.json       两首连写
一键脚本/  运行、停止、复位、服务器与手节点管理
工具/      标定与诊断脚本
```

## 关键参数（配置里）
| 键 | 值 | 说明 |
|---|---|---|
| `z_heights_mm.down` | −27.65 | 写字高度（笔尖触纸）|
| `z_heights_mm.up` | −8.85 | 抬笔高度（余量 18.8mm）|
| `ink.dip_depth_mm` | 18.0 | 蘸墨下探深度 |
| `flow.dip_every_n` | 20 | 每 20 字蘸一次墨 |
| `writing.servo_timing_mode` | fixed | 固定周期模式 |
| `writing.servo_period_s` | 0.016 | 16ms |
| `writing.use_bezier_jump` | true | Bézier 跳笔 |
| `writing.speed_mm_s` | 100 | 书写速度 |
| `writing.lift_speed_mm_s` | 40 | 上提速度 |
| `ink.sleep_after_grasp_s` | 3.0 | 抓握后等待 |
| `flow.simplify_tol_mm` | 0.0 | 不简化，保留原始字形 |

## 运行
```bash
bash 一键脚本/start_dexhand.sh        # 启动灵巧手
bash 一键脚本/run_three_seq.sh 1      # 三遍连写
bash 一键脚本/safe_stop.sh            # 停止
```
