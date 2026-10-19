# Chinese Calligraphy Robot (FR5 + DexHand021 + RealSense)

A robot that holds a brush pen with an industrial arm and writes Chinese characters,  
plus a dotting ("dot the eye") task.

This repository contains two complete, field-tested programs:

- **Task 2 — Writing** (`任务2_写字_版本1/`): stroke-by-stroke Chinese calligraphy on a grid
- **Task 1 — Dotting** (`任务1_点睛/`): placing ink dots at taught positions

Both run on a real FR5 arm and include the full coordinate-calibration toolchain.

---

## Table of Contents

- [1. System Architecture](#1-system-architecture)
- [2. Hardware](#2-hardware)
- [3. Software Environment](#3-software-environment)
- [4. Coordinate Systems and Calibration](#4-coordinate-systems-and-calibration)
- [5. Algorithms](#5-algorithms)
- [6. Execution Flow](#6-execution-flow)
- [7. Configuration Reference](#7-configuration-reference)
- [8. Calibration Procedure](#8-calibration-procedure)
- [9. Measured Performance](#9-measured-performance)
- [10. Troubleshooting / Field Notes](#10-troubleshooting--field-notes)
- [11. Repository Layout](#11-repository-layout)

---

## 1. System Architecture

```
┌──────────────────────────────────────────────────────────────┐
│                    Trajectory data (JSON)                     │
│  poem1_trajectories.json: 20 chars × N strokes × M points     │
└───────────────────────────┬──────────────────────────────────┘
                            │ load
                            ▼
┌──────────────────────────────────────────────────────────────┐
│            Executor  hanzi_executor.py  (ROS 2 node)          │
│                                                              │
│  1. table coords  → base coords   (affine transform)          │
│  2. per-character Z offset        (paper tilt compensation)   │
│  3. orientation override          (avoid IK branch flips)      │
│  4. pen tip → flange              (TCP offset)                │
│  5. inverse kinematics            (GetInverseKinRef, seeded)   │
│  6. fixed-period ServoJ streaming                              │
└──────────┬────────────────────────────┬──────────────────────┘
           │ ROS 2 service              │ ROS 2 topic
           ▼                            ▼
┌─────────────────────────┐   ┌──────────────────────────┐
│  ros2_cmd_server        │   │  dexhand_ros (hand)       │
│  (Fairino FR SDK)       │   │  /right_hand/joint_commands│
│  string command API     │   │  /right_hand/joint_states  │
└──────────┬──────────────┘   └────────────┬─────────────┘
           │ TCP/IP                        │ USB-CANFD
           ▼                               ▼
┌─────────────────────────┐   ┌──────────────────────────┐
│   FR5 robot arm         │   │  DexHand021 (19 DOF)      │
│   controller 192.168.58.2│  │                          │
└─────────────────────────┘   └──────────────────────────┘
```

**Design decision:** the executor only sends *joint angles*; the controller interpolates.  
All pen-tip ↔ flange geometry is done in software and the controller's tool frame is  
**zeroed**. This keeps every geometric computation in exactly one place, so there is  
never a "two coordinate systems disagree" bug.

---

## 2. Hardware

| Part           | Model                  | Key specs                                    |
| -------------- | ---------------------- | -------------------------------------------- |
| Robot arm      | FR5 (Fairino)          | controller V3.9.8, 6-axis, 5 kg payload      |
| Dexterous hand | DexHand021             | 19 DOF, 5 fingers                            |
| CAN adapter    | ZLG USBCANFD-100U-mini | USB → CAN FD                                 |
| Camera         | Intel RealSense        | used for teaching / visual verification      |
| Pen            | felt brush pen         | **pen tip → flange = 279.8 mm** (calibrated) |
| Paper          | A3 grid sheets         | 2×2 tiled, 40 cells                          |

**Communication:**

```
Ubuntu (VM) ──TCP/IP──> FR5 controller   (192.168.58.2)
Ubuntu (VM) ──USB──> ZCAN adapter ──CAN FD──> DexHand021
```

---

## 3. Software Environment

| Component   | Version                           |
| ----------- | --------------------------------- |
| OS          | Ubuntu 22.04 (VMware VM)          |
| ROS         | ROS 2 Humble                      |
| Python      | 3.10                              |
| Fairino SDK | 2.3.8 (`fairino_hardware_v3_9_8`) |
| Hand driver | `pyzlg_dexhand` (separate venv)   |

**Startup order (all required):**

```bash
source /opt/ros/humble/setup.bash
source ~/fr5_ros2_ws/install/setup.bash
# only then start the executor
```

---

## 4. Coordinate Systems and Calibration

There are four frames, chained by homogeneous transforms:

```
table (m)  ──affine──>  base (mm)
                          │
                          │  orientation R (rpy)
                          ▼
                       flange
                          │  + R @ TCP
                          ▼
                       pen tip
```

### 4.1 Table → Base (affine)

```python
bx = a00 * tx + a01 * ty + b0
by = a10 * tx + a11 * ty + b1
```

Calibrated values (5-point least squares):

```
a00 = -0.02234423   a01 = -0.9759539    b0 = 0.12185192
a10 = -0.9475345    a11 = -0.02877384   b1 = -0.14636167
```

- Scale: x 0.948, y 0.976
- Rotation: **−91.35°** (table almost orthogonal to base frame)
- **Residual: 1.36 mm RMS** over the 5 calibration points

### 4.2 Orientation (rpy → rotation matrix)

ZYX Euler angles:

```python
R = [[ cz*cy,  cz*sy*sx - sz*cx,  cz*sy*cx + sz*sx],
     [ sz*cy,  sz*sy*sx + cz*cx,  sz*sy*cx - cz*sx],
     [ -sy,    cy*sx,             cy*cx            ]]
```

Pen points down: `rx ≈ 89.86°` (pen axis nearly vertical).

### 4.3 Pen tip → Flange (TCP offset)

```
pen    = flange + R @ TCP
flange = pen    - R @ TCP
```

TCP (pen tip relative to flange):

```
x =  71.084 mm
y = -233.762 mm
z =  136.344 mm
|TCP| = 279.8 mm    ← pen length
```

> Why these odd numbers: the pen sits at an angle in the gripper, not along  
> the flange Z axis. All three components were solved from multi-pose teaching  
> with least squares — measured indirectly, not with a ruler.

### 4.4 Z heights

```
z_heights_mm.down = -27.65    # pen tip touching paper
z_heights_mm.up   =  -8.85    # pen lifted (clearance 18.8 mm)
```

**Per-character offset** `z_offsets_mm`: compensates paper tilt. Each of the  
20 characters has its own offset; positive = higher (lighter stroke),  
negative = lower (darker stroke).

---

## 5. Algorithms

### 5.1 Trajectory source

Each character is a set of strokes; each stroke is a list of **table-frame points**  
(metres, derived from glyph data):

```json
{
  "character": "亭",
  "strokes": [
    { "points": [ {"x_m": 0.3009, "y_m": -0.2827}, ... ] },
    ...
  ]
}
```

### 5.2 Bézier arc pen jump ★ core

**Problem.** Between strokes the pen must leave the paper and fly to the next  
stroke's start. The classic approach is "lift vertically → translate horizontally →  
drop vertically". Each of the three segments starts and stops at zero velocity,  
producing **three velocity discontinuities**. The arm shakes violently and the whole  
bench vibrates.

**Solution.** One cubic Bézier curve does the whole transfer in a single motion:

```
P0 = end of previous stroke
P1 = (P0.x, P0.y, control_z)     ← directly above P0
P2 = (P3.x, P3.y, control_z)     ← directly above P3
P3 = start of next stroke

B(t) = (1-t)³·P0 + 3(1-t)²t·P1 + 3(1-t)t²·P2 + t³·P3
```

**Arc-height control** (so the apex lands exactly at the target height):

```
apex_z    = z0 + base_height + ratio * planar_distance
apex_z    = clamp(apex_z, min_height, max_height)
control_z = (apex_z - 0.125·(z0 + z3)) / 0.75
```

Derivation: at `t=0.5`, the z component of `B` is  
`0.125·z0 + 0.375·z1 + 0.375·z2 + 0.125·z3`.  
Setting `z1 = z2 = control_z` and solving for `control_z` gives the formula above.

**Why it kills the shake:**

- take-off tangent points mostly *up* (natural pen lift)
- landing tangent points mostly *down* (natural pen touch-down)
- **velocity is continuous** — there is no corner anywhere

Parameters:

```
bezier_jump_base_height_mm = 3.0    # base lift
bezier_jump_height_ratio   = 0.20   # arc height = 20% of planar distance
bezier_jump_min_height_mm  = 6.0
bezier_jump_max_height_mm  = 12.0
bezier_jump_speed_mm_s     = 180.0
bezier_jump_ramp_time_s    = 0.05
```

### 5.3 Arc-length resampling + quintic profile

**Problem.** Glyph points are unevenly spaced (dense at corners, sparse on straight  
runs). If time is distributed per point, the pen speed pulses.

**Solution:**

1. **Cumulative arc length**: `s[i] = Σ |P[i] − P[i−1]|`
2. **Resample at a fixed time step**: advance `v·dt` along the arc each tick
3. **Quintic ramps at both ends** (S-curve velocity profile):

```
q(u)  = 10u³ − 15u⁴ + 6u⁵      # normalized displacement
q'(0) = q'(1) = 0              # zero velocity at both ends
q''(0)= q''(1)= 0              # zero acceleration at both ends
max q' = 1.875  (at u = 0.5)
```

**Velocity profile:**

```
long stroke (length ≥ v · ramp_time):
    accelerate for ramp_time ── constant ── decelerate for ramp_time
short stroke:
    pure quintic (peak velocity still ≤ v)
```

### 5.4 Inverse kinematics

**Problem.** A single end-effector pose can have up to 8 joint solutions. Solving  
each point independently can silently "jump branches" mid-stroke and the wrist  
would flip.

**Solution:**

1. **`GetInverseKinRef`** with the previous joint vector as the seed → continuous solutions
2. **Jump check**: adjacent-point joint error > 25° → abort (never flip a branch)
3. **Joint speed check**: `Δjoint / period > 120 °/s` → automatically lengthen the  
   period and re-plan that stroke

### 5.5 Fixed-period ServoJ

Inside a stroke the executor streams servo commands instead of issuing point-by-point  
`MoveL`:

```python
for each sample:
    ServoJ(j1..j6, 0,0,0,0, dt)      # dt = 16 ms
    sleep_until(next 16 ms boundary)
```

```
servo_period_s = 0.016      # 62.5 Hz
```

**Caveat:** when ServoJ is relayed through the ROS 2 *string* service, the period  
must be **≥ 25 ms**, otherwise the controller periodically returns `-4`  
(command queue overflow). At 16 ms this happens occasionally on the real robot.

### 5.6 Soft landing

A stroke must not slam onto the paper. The approach is split into 5 cubic-decelerated  
steps:

```
z_k = z_start + (z_end − z_start) · (k/5)³     k = 0..5
```

Fast at first, nearly zero on the last step → gentle touch-down.

### 5.7 Ink dipping strategy

```
dip when  ai % dip_every_n == 0        # ai = character index (0-based)
dip_every_n = 20                       # once per 20 chars = once per poem
```

**Dip motion** (fully servo-controlled):

```
safe high → inkwell hover → descend 18 mm → return to hover
```


`dip_depth_mm = 18.0` is the descent depth; positive = deeper.

> **Observed in practice:** the three dips need *different* depths.  
> The ink level drops and the pen shifts slightly in the gripper, so dips 2 and 3  
> end up too deep if the same depth is reused.

---

## 6. Execution Flow

```
start
 │
 ├─ 1. self-check
 │     ├─ read joints / error code
 │     ├─ ResetAllError
 │     ├─ RobotEnable(1)
 │     ├─ SetSpeed(100)
 │     └─ SetToolCoord(zero)
 │
 ├─ 2. pick up the pen   (pass 1 only)
 │     ├─ ServoJ → "approach" pose          (pre_pickup_s)
 │     ├─ ServoJ → pickup pose              (pickup_move_s)
 │     ├─ publish 60 grasp commands @ 50 ms → 3.0 s
 │     ├─ wait 3.0 s                        (sleep_after_grasp_s)
 │     │                                    → 6 s total for the hand to close
 │     └─ lift 220 mm vertically @ 40 mm/s  (pickup_lift_mm)
 │
 └─ 3. three passes   for pass in 1..3:
      │
      └─ each pass writes 20 chars in two halves (1–10 / 11–20):
          │
          ├─ dip ink (at the start of each half)
          │   ├─ ServoJ → safe high   (skipped on the very first half)
          │   ├─ ServoJ → inkwell hover
          │   ├─ ServoJ → descend 18 mm
          │   └─ ServoJ → back to hover
          │
          └─ write each character:
              │
              └─ write each stroke:
                  ├─ Bézier pen jump (previous end → this start)
                  ├─ soft landing (5 decelerating steps)
                  ├─ stroke body (arc-length resample + quintic + fixed-period ServoJ)
                  └─ micro-lift 2 mm (inline_release_mm)

end: the pen stays in the hand (never auto-released)
```

**Pass differences:**

| Pass | Pick up         | Ink dip          | Start char |
| ---- | --------------- | ---------------- | ---------- |
| 1    | ✅ full pickup   | ✅ every 20 chars | 1          |
| 2    | ❌ keeps holding | ✅ every 20 chars | 1          |
| 3    | ❌ keeps holding | ✅ every 20 chars | 1          |

---

## 7. Configuration Reference

File: `配置/hanzi_config.json`

### 7.1 Geometry

| Key                 | Value                       | Meaning                              |
| ------------------- | --------------------------- | ------------------------------------ |
| `tool_tcp.x/y/z`    | 71.084 / -233.762 / 136.344 | pen tip relative to flange (mm)      |
| `table_transform`   | see 4.1                     | table → base affine                  |
| `z_heights_mm.down` | **-27.65**                  | pen-tip writing height               |
| `z_heights_mm.up`   | **-8.85**                   | lift height (clearance 18.8 mm)      |
| `z_offsets_mm`      | 20 entries                  | per-character Z offset (paper tilt)  |
| `posture_by_char`   | chars 18–20                 | orientation override (avoid IK flip) |

### 7.2 Writing

| Key                 | Value | Meaning                                 |
| ------------------- | ----- | --------------------------------------- |
| `speed_mm_s`        | 100   | writing speed                           |
| `lift_speed_mm_s`   | 40    | lift speed (deliberately slow)          |
| `servo_timing_mode` | fixed | fixed-period mode                       |
| `servo_period_s`    | 0.016 | 16 ms                                   |
| `use_bezier_jump`   | true  | Bézier pen jumps                        |
| `soft_landing`      | true  | gentle touch-down                       |
| `soft_ramp`         | true  | start/stop ramps                        |
| `simplify_tol_mm`   | 0.0   | no simplification (keep original glyph) |
| `corner_keep_deg`   | 40.0  | keep corners with >40° direction change |

### 7.3 Ink and grasp

| Key                       | Value     | Meaning                                  |
| ------------------------- | --------- | ---------------------------------------- |
| `ink.dip_depth_mm`        | 18.0      | dip descent depth                        |
| `ink.dip_touch_joint`     | 6 numbers | taught touch-ink joint pose              |
| `ink.pickup_lift_mm`      | 220       | vertical lift after grasp                |
| `ink.sleep_after_grasp_s` | 3.0       | wait after grasp (+3 s publishing = 6 s) |
| `flow.dip_every_n`        | 20        | dip every 20 characters                  |
| `flow.dip_ink`            | true      | master switch for dipping                |

---

## 8. Calibration Procedure

Re-calibrate whenever you change paper, pen, or bench.

### 8.1 TCP (pen tip position)

```bash
python3 工具/tcp_cal.py
```

Touch the same point with 5 different orientations; least squares solves the TCP.

### 8.2 Table mapping

```bash
bash 工具/teach_transform.sh
```

Teach 5 cell centres, solve the affine matrix.

### 8.3 Per-character Z heights

```bash
bash 工具/teach_z_points.sh
```

Lower the pen on each character until it just touches, record the offset.

### 8.4 Verification

```bash
python3 工具/calib_resid.py      # residual (should be < 2 mm)
bash 工具/show_calib.sh          # print current calibration
```

---

## 9. Measured Performance

| Metric               | Value                            |
| -------------------- | -------------------------------- |
| One pass (20 chars)  | **≈ 150–175 s**                  |
| Three passes         | ≈ 8–9 min                        |
| Per character        | 3–17 s (depends on stroke count) |
| 16 ms overruns       | **0** (normal case)              |
| Calibration residual | 1.36 mm                          |

**Per-character times** (pass 1):

```
一 4.9s   去 7.9s   二 3.3s   三 5.0s   里 10.8s
烟 14.5s  村 10.0s  四 8.1s   五 6.6s   家 11.4s
亭 16.9s  台 ...    六 ...    七 ...    座 ...
```

---

## 10. Troubleshooting / Field Notes

Every item below was learned the hard way on the real robot.

### ① The acceleration argument is 0–10, not a percentage

```python
MoveJ(CART1, speed, acc, 0,0,0,0,0)
                    ↑ acc ∈ [0, 10]
```

Passing 20+ makes the controller reject the command (returns `-1`) — and the arm  
simply **does not move, without an obvious error**. Very easy to misdiagnose as a  
communication problem.

### ② Only one command server

```bash
pgrep -f ros2_cmd_serve | wc -l    # should be 2  (2 processes = 1 service)
```

4 processes = 2 services = **fighting over the command stream → the arm freezes**.

### ③ Only one hand node

Two instances cause CAN contention → "the hand won't grip".

### ④ Never run two executors

They compete for the same ROS service and the arm receives interleaved garbage.

### ⑤ The hand's CAN link drops occasionally

Symptom: `/tmp/dexhand.log` fills with `TransmitFD failed`,  
`/right_hand/joint_states` goes silent, the hand does not move at all.  
**Fix: restart the hand node**

```bash
bash 一键脚本/start_dexhand.sh
```

### ⑥ Do not use joint-space servo for lifting

Verified on hardware: the arm crashes into the table (a joint-space straight line  
is not a Cartesian straight line).

### ⑦ Do not merge lifting into the stroke servo stream

90° turns shake badly and the characters look awful. Keep the dedicated Bézier jump.

### ⑧ ServoJ returns 101 when the robot is disabled

Always run before moving:

```python
ResetAllError()
RobotEnable(1)
```

### ⑨ Shutdown order

Kill the executor first, *then* send stop commands. **Never kill the command server  
first** — the executor will interpret it as a communication fault and hang.

### ⑩ Start the hand node with `setsid`

Otherwise it dies when the SSH session closes → the hand opens → the pen drops.

### ⑪ Fixed-period ServoJ relayed over the ROS string service needs ≥25 ms

At 16 ms the controller periodically returns `-4` (queue overflow).  
Either slow down to 25–30 ms or talk to the SDK directly (C++).

### ⑫ Heights and dip depth must be tuned on the real robot

The same numbers behave very differently on different paper and different pens.  
After every paper change, write 2–3 characters and check before the full run.

---

## 11. Repository Layout

```
.
├── README.md                     this file (includes license)
├── 任务1_点睛/                    Task 1 - dotting
│   ├── README.md
│   ├── positions.json            dot positions
│   ├── record_position.py        teaching / recording
│   ├── run_dot_sequence.py       sequence execution
│   └── run.sh                    one-command runner
└── 任务2_写字_版本1/              Task 2 - writing (3-pass version)
    ├── README.md                 version notes + parameter table
    ├── 程序/
    │   └── hanzi_executor.py     main executor (1570 lines)
    ├── 配置/
    │   ├── hanzi_config.json     all parameters
    │   └── transform_points.json 5-point calibration data
    ├── 轨迹/
    │   └── poem1_trajectories.json   poem 1 (20 chars x 3 passes)
    ├── 一键脚本/                  only 5 essentials
    │   ├── run_three_seq.sh      run 3 passes
    │   ├── safe_stop.sh          safe stop
    │   ├── full_cleanup.sh       kill all nodes
    │   ├── start_server.sh       start command server
    │   └── start_dexhand.sh      start dexterous hand
    └── 工具/                      calibration only (9 scripts)
        ├── tcp_cal.py            TCP calibration
        ├── teach_transform.py/.sh table->base affine
        ├── teach_z_points.py/.sh  per-char Z heights
        ├── calib_resid.py        residual check
        ├── show_calib.sh         print calibration
        ├── solve_joint.py        joint solution helper
        └── derive_z.py           Z derivation helper
```

> Directory names are in Chinese for consistency with the original working tree;
> each has an English description above.

## License

This project is released under the **MIT License**.

```
MIT License

Copyright (c) 2026 Ruiheng Guo

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```

**In short:** you may use, copy, modify, merge, publish, distribute, sublicense,  
and sell copies of this software - provided you keep the copyright notice above.  
The software comes with no warranty of any kind.

> If you do **not** want others to reuse this code, remove this section, delete the  
> `LICENSE` file, and set the repository to **Private** instead.
