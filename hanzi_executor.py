#!/usr/bin/env python3
"""写汉字（古诗书法）轨迹执行器。

读 hanzi_grid_writer 生成的轨迹 JSON（table_frame 桌面坐标，单位米），
经 table_frame -> base_link 变换，补 Z 高度与笔尖姿态，
通过 ros2_cmd_server 的 RemoteCmdInterface 执行：笔画间 MoveJ/MoveL，
笔画内默认使用固定周期 ServoJ（也保留 legacy 与实验性 ServoCart 开关）。

用法：
    python3 hanzi_executor.py [--config ~/hanzi_task/hanzi_config.json] [--dry-run]
"""

import bisect
import json
import math
import sys
import time

# ROS 依赖惰性导入：dry-run 不需要 ROS 环境，真实运行时才需要。
# 这里先尝试导入，失败则用占位（dry-run 分支不会用到它们）。
try:
    import rclpy
    from rclpy.node import Node
except Exception:  # noqa: BLE001
    rclpy = None
    Node = object
try:
    from fairino_msgs.srv import RemoteCmdInterface
except Exception:  # noqa: BLE001
    RemoteCmdInterface = None
try:
    from sensor_msgs.msg import JointState
except Exception:  # noqa: BLE001
    JointState = None


HAND_NAMES = [
    'r_f_joint1_1','r_f_joint1_2','r_f_joint1_3','r_f_joint1_4',
    'r_f_joint2_1','r_f_joint2_2','r_f_joint2_3','r_f_joint2_4',
    'r_f_joint3_2','r_f_joint3_3','r_f_joint3_4',
    'r_f_joint4_1','r_f_joint4_2','r_f_joint4_3','r_f_joint4_4',
    'r_f_joint5_1','r_f_joint5_2','r_f_joint5_3','r_f_joint5_4',
]
HAND_GRASP = [
    1.894904, 0.245723, 0.674026, 0.674026,
    0.027925, 1.159577, 0.531259, 0.531259,
    1.245273, 0.655002, 0.655002,
    0.027925, 1.093080, 0.702650, 0.702650,
    0.027925, 0.864442, 0.674026, 0.674026,
]


# ---------------------------------------------------------------------------
# 坐标变换：table_frame -> base_link
# 2026-10-06 17:15 实机三点标定（笔尖对准格子中心读 GetActualTCPPose）：
#   白(第1行第1列)  table(0.457,-0.270)m -> base(373.2,-598.1)mm
#   流(第1行第10列) table(0.457, 0.470)m -> base(-370.2,-603.1)mm
#   河(第2行第1列)  table(0.349,-0.270)m -> base(369.3,-492.4)mm
# 最小二乘仿射拟合（含纸的轻微倾斜）：
#   base_x_m = -1.004609*table_y_m + 0.036778*table_x_m + 0.085175
#   base_y_m = -0.978352*table_x_m - 0.006776*table_y_m - 0.152822
# 主方向：base_x = -table_y（列/左右方向），base_y = -table_x（行/前后方向）。
# 注意：之前误用 workobject_tf.yaml（那是"标定板"的变换，与田字格纸无关）。
# ---------------------------------------------------------------------------
def _simplify_points(pts, eps):
    """Douglas-Peucker 化简笔画点（eps 单位米），保留首尾，可大幅减少 MoveL 次数。"""
    if len(pts) <= 2 or eps <= 0:
        return pts
    x1, y1 = pts[0]["x_m"], pts[0]["y_m"]
    x2, y2 = pts[-1]["x_m"], pts[-1]["y_m"]
    dx, dy = x2 - x1, y2 - y1
    L = math.hypot(dx, dy)
    dmax, idx = -1.0, 0
    for i in range(1, len(pts) - 1):
        x0, y0 = pts[i]["x_m"], pts[i]["y_m"]
        if L < 1e-12:
            d = math.hypot(x0 - x1, y0 - y1)
        else:
            d = abs(dy * x0 - dx * y0 + x2 * y1 - y2 * x1) / L
        if d > dmax:
            dmax, idx = d, i
    if dmax > eps:
        return _simplify_points(pts[:idx + 1], eps)[:-1] + _simplify_points(pts[idx:], eps)
    return [pts[0], pts[-1]]


def _simplify_keep_corners(pts, eps, corner_deg=150.0):
    """拐角感知简化：方向变化 > corner_deg 的点强制保留，其余段用 eps 化简。"""
    if len(pts) <= 2:
        return pts
    keep = [0]
    for i in range(1, len(pts) - 1):
        x0, y0 = pts[i - 1]["x_m"], pts[i - 1]["y_m"]
        x1, y1 = pts[i]["x_m"], pts[i]["y_m"]
        x2, y2 = pts[i + 1]["x_m"], pts[i + 1]["y_m"]
        v1 = (x1 - x0, y1 - y0)
        v2 = (x2 - x1, y2 - y1)
        n1 = math.hypot(*v1)
        n2 = math.hypot(*v2)
        if n1 < 1e-9 or n2 < 1e-9:
            continue
        c = max(-1.0, min(1.0, (v1[0]*v2[0] + v1[1]*v2[1]) / (n1 * n2)))
        ang = math.degrees(math.acos(c))
        if ang > corner_deg:
            keep.append(i)
    keep.append(len(pts) - 1)
    keep = sorted(set(keep))
    out = [pts[keep[0]]]
    for a, b in zip(keep[:-1], keep[1:]):
        seg = _simplify_points(pts[a:b + 1], eps)
        out.extend(seg[1:])
    return out


def build_transform(config=None):
    """返回函数：table坐标(m) -> base坐标(m)。优先用配置 table_transform（示教拟合），否则用实机三点标定。"""
    A = [
        [0.036778, -1.004609],   # base_x = 0.036778*Tx - 1.004609*Ty
        [-0.978352, -0.006776],  # base_y = -0.978352*Tx - 0.006776*Ty
    ]
    b = [0.085175, -0.152822]
    tt = (config or {}).get("table_transform")
    if tt:
        A = [[tt["a00"], tt["a01"]], [tt["a10"], tt["a11"]]]
        b = [tt["b0"], tt["b1"]]

    def transform(x_m, y_m):
        bx = A[0][0] * x_m + A[0][1] * y_m + b[0]
        by = A[1][0] * x_m + A[1][1] * y_m + b[1]
        return (bx, by, 0.0)  # z 由调用方传入的绝对高度决定

    return transform


class HanziExecutor(Node):
    def __init__(self, config):
        super().__init__("hanzi_executor")
        self.cfg = config
        self.cli = self.create_client(
            RemoteCmdInterface, "/fairino_remote_command_service"
        )
        self.transform = build_transform(config)

        # 笔尖姿态（度）：竖直朝下时由示教确定；未示教时默认法兰朝下
        rpy = config.get("pen_tcp_rpy_deg") or {"rx": None, "ry": None, "rz": None}
        self.rx = rpy["rx"] if rpy["rx"] is not None else 180.0
        self.ry = rpy["ry"] if rpy["ry"] is not None else 0.0
        self.rz = rpy["rz"] if rpy["rz"] is not None else 0.0

        # 书写姿态对应的旋转矩阵（用于把 TCP 偏移从法兰系旋转到 base 系）
        self._flange_R = self._rpy_to_R(self.rx, self.ry, self.rz)

        # 按列 rx 覆盖：第 1~10 字用 rx_by_column[0]，第 11~20 字用 [1]
        self.rx_by_column = config.get("rx_by_column") or [self.rx, self.rx]

        # 工具坐标系：启动时写入标定好的 TCP，全程用 tool_id
        tcp = config["tool_tcp"]
        self.tool_id = int(tcp["tool_id"])
        self.tool_tcp = tcp

        # Z 高度（mm）
        self.z_down = config["z_heights_mm"]["down"]
        self.z_up = config["z_heights_mm"]["up"]

        self.writing = config["writing"]
        self.flow = config["flow"]

        # 取笔/蘸墨（可选）
        self.ink = config.get("ink") or {}
        self.hand_pub = None
        if JointState is not None:
            self.hand_pub = self.create_publisher(
                JointState, "/right_hand/joint_commands", 10
            )

    # -- 指令封装 ------------------------------------------------------------
    @staticmethod
    def _rpy_to_R(rx_deg, ry_deg, rz_deg):
        """RPY(ZYX) -> 旋转矩阵。用于把 TCP 偏移从法兰坐标系旋转到 base 系。"""
        rx, ry, rz = math.radians(rx_deg), math.radians(ry_deg), math.radians(rz_deg)
        cx, cy, cz = math.cos(rx), math.cos(ry), math.cos(rz)
        sx, sy, sz = math.sin(rx), math.sin(ry), math.sin(rz)
        return [
            [cz * cy, cz * sy * sx - sz * cx, cz * sy * cx + sz * sx],
            [sz * cy, sz * sy * sx + cz * cx, sz * sy * cx - cz * sx],
            [-sy, cy * sx, cy * cx],
        ]

    def call(self, cmd, timeout=15.0):
        if not self.cli.wait_for_service(timeout_sec=3.0):
            return "NO_SERVICE"
        req = RemoteCmdInterface.Request()
        req.cmd_str = cmd
        fut = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        return fut.result().cmd_res if fut.done() else "TIMEOUT"

    def call_fast(self, cmd, timeout=5.0):
        """热路径专用：不做 wait_for_service 检查（实测每次省约 200ms）。

        只用于连续下发 ServoJ 这类高频调用；低频命令仍用 call() 保证连接健壮。
        """
        req = RemoteCmdInterface.Request()
        req.cmd_str = cmd
        fut = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        return fut.result().cmd_res if fut.done() else "TIMEOUT"

    def _pen_to_flange(self, pen_mm):
        """把笔尖(TCP)基坐标(mm)换算成法兰基坐标(mm)。
        关键：CARTPoint 存的位姿，MoveJ/MoveL 的逆解器(GetInverseKin)按"法兰位姿"处理，
        不使用 tool 偏移。所以必须把笔尖目标位姿转成法兰位姿。
        公式（已验证）：法兰 = 笔尖 - R(书写姿态) × TCP偏移。
        """
        tcp = self.tool_tcp
        t = (tcp["x"], tcp["y"], tcp["z"])  # 法兰坐标系下的 TCP 偏移(mm)
        R = self._flange_R
        off = [sum(R[i][j] * t[j] for j in range(3)) for i in range(3)]
        return (pen_mm[0] - off[0], pen_mm[1] - off[1], pen_mm[2] - off[2])

    def cart_point(self, idx, pen_x_m, pen_y_m, pen_z_mm):
        """把笔尖目标位姿换算成法兰位姿，存到 CART 容器 idx（tool=0 逆解语义）。"""
        pen_mm = (pen_x_m * 1000.0, pen_y_m * 1000.0, pen_z_mm)
        fx, fy, fz = self._pen_to_flange(pen_mm)
        return self.call(
            f"CARTPoint({idx},{fx:.3f},{fy:.3f},{fz:.3f},"
            f"{self.rx:.3f},{self.ry:.3f},{self.rz:.3f})"
        )

    def move_l(self, idx, speed_mm_s, acc=None):
        """笛卡尔直线移动到 CARTn。

        重要：v3.9.8 command_server 的字符串参数顺序是
        ``点位, speed, tool, user, 外部轴1..4``，并不接收逐条 acc。
        旧代码把 ``lin_acc_pct`` 填在第三项，实际会误选 tool 3，而不是 3% 加速度。
        这里固定使用 tool=0/user=0；真正的 MoveL 加速度由命令服务器参数
        ``MoveJLC_acc`` 决定。保留 acc 形参只为兼容旧调用，当前不会下发它。
        """
        _ = acc
        return self.call(
            f"MoveL(CART{idx},{int(speed_mm_s)},0,0,0,0,0,0)"
        )

    def move_j(self, flange_mm, speed):
        """PTP 接近改用 MoveJ(CART)：控制器规划关节路径、自动到位。
        rz=-156.86 自然手腕(无翻腕)，GetInverseKinExaxis 不会 112。
        speed 由 SDK 当作速度百分比（0~100）。"""
        fx, fy, fz = flange_mm
        current = self.read_joints()
        if current is None:
            self.get_logger().error("MoveJ 前读取当前关节角失败")
            return "READ_JOINT_FAILED"
        # 先用当前关节角作为参考选择最近的逆解分支，再下发 JNT 点。
        # 这样避免 command_server 对 CART 点使用 config=-1 时偶发换解/翻腕。
        target = self.ik_flange(fx, fy, fz, joint_ref=current)
        if target is None:
            self.get_logger().error("MoveJ 参考逆解失败")
            return "IK_FAILED"
        r = self.call(
            "JNTPoint(1," + ",".join(f"{v:.3f}" for v in target) + ")"
        )
        if r.strip() != "0":
            return r
        # command_server 第三个参数是 tool，不是加速度；目标本身是软件换算后的法兰位姿，
        # 所以必须固定 tool=0/user=0，避免重复叠加或误用其它工具坐标。
        return self.call(f"MoveJ(JNT1,{int(speed)},0,0,0,0,0,0)")

    def ik_flange(self, fx, fy, fz, joint_ref=None, fast=False):
        """法兰位姿逆解成关节角(度)。

        joint_ref 存在时使用 GetInverseKinRef，强制选择靠近上一姿态的连续解；
        这比每个点都用 config=-1 独立逆解更不容易翻腕。失败返回 None。
        """
        if joint_ref is None:
            cmd = (
                f"GetInverseKin(0,{fx:.3f},{fy:.3f},{fz:.3f},"
                f"{self.rx:.3f},{self.ry:.3f},{self.rz:.3f},-1)"
            )
        else:
            cmd = (
                f"GetInverseKinRef(0,{fx:.3f},{fy:.3f},{fz:.3f},"
                f"{self.rx:.3f},{self.ry:.3f},{self.rz:.3f},"
                + ",".join(f"{v:.3f}" for v in joint_ref) + ")"
            )
        # 固定周期轨迹会连续求解很多点；服务已建立后用 fast 路径，避免每点
        # 重复等待服务发现。普通单点动作仍走 self.call()，保留连接检查。
        res = self.call_fast(cmd) if fast else self.call(cmd)
        parts = res.split(",")
        if parts[0].strip() != "0" or len(parts) < 7:
            return None
        return [float(v) for v in parts[1:7]]

    def servo_goto(self, target, duration=None):
        """笔画间移动/垂直抬笔：逆解后用 ServoJ 关节伺服过去（不做到位轮询）。

        比 MoveJ/MoveL+wait_arrive 省掉每笔约 0.5~1 秒的固定开销。
        逆解失败 / 关节差过大 / 伺服失败时返回 False，调用方回退旧路径。
        """
        fx, fy, fz = self._pen_to_flange(target)
        cur = self.read_joints()
        if cur is None:
            return False
        j = self.ik_flange(fx, fy, fz, joint_ref=cur)
        if j is None:
            return False
        dmax = max(abs(j[k] - cur[k]) for k in range(6))
        if dmax > 170.0:
            self.get_logger().warn(f"servo_goto: 关节差 {dmax:.0f}° 过大 -> 回退")
            return False
        if duration is None:
            # 关节速度(度/秒)。注意 MoveJ 的 speed 参数是百分比(100=全速)，
            # 所以这里直接给一个接近/略高于它的关节角速度。
            spd = float(self.writing.get("servo_joint_speed_dps", 250.0))
            duration = min(max(dmax / max(spd, 1.0), 0.12), 1.2)
        _steps = max(6, min(40, int(duration / 0.02)))
        _t = time.time()
        _ok = bool(self.servo_move_j(j, duration=duration, steps=_steps))
        self.get_logger().info(
            f"伺服段: 关节差 {dmax:.0f}°, 时长 {duration:.2f}s, 实际 {time.time()-_t:.2f}s"
        )
        return _ok

    @staticmethod
    def _quintic(u):
        """五次平滑阶跃：两端速度、加速度都为 0。"""
        u = max(0.0, min(1.0, float(u)))
        return 10.0 * u**3 - 15.0 * u**4 + 6.0 * u**5

    @staticmethod
    def _quintic_velocity_integral(u):
        """五次速度斜坡的归一化位移积分，u=1 时等于 0.5。"""
        u = max(0.0, min(1.0, float(u)))
        return 2.5 * u**4 - 3.0 * u**5 + u**6

    @staticmethod
    def _polyline_point(points, cumulative, distance):
        """按弧长在折线上插值，避免原始 JSON 点距不均造成忽快忽慢。"""
        if not points:
            return None
        if distance <= 0.0:
            return tuple(points[0])
        if distance >= cumulative[-1]:
            return tuple(points[-1])
        i = max(1, bisect.bisect_left(cumulative, distance))
        seg = cumulative[i] - cumulative[i - 1]
        if seg <= 1e-9:
            return tuple(points[i])
        u = (distance - cumulative[i - 1]) / seg
        return tuple(
            points[i - 1][k] + u * (points[i][k] - points[i - 1][k])
            for k in range(3)
        )

    def _sample_stroke_fixed_period(self, targets, speed, period, ramp_time):
        """把笔画折线转换成固定周期、带 S 形起停的笛卡尔目标序列。

        长笔画：起步和收笔各使用 quintic 速度斜坡，中间匀速；总时长
        约为 ``长度/速度 + ramp_time``。短笔画容不下匀速段时，整段用
        quintic 位移曲线，峰值速度仍不超过 speed。
        """
        if not targets:
            return [], 0.0, 0.0
        clean = [tuple(float(v) for v in targets[0])]
        for p in targets[1:]:
            p = tuple(float(v) for v in p)
            if math.dist(clean[-1], p) > 1e-6:
                clean.append(p)
        if len(clean) == 1:
            return clean, 0.0, 0.0

        cumulative = [0.0]
        for i in range(1, len(clean)):
            cumulative.append(cumulative[-1] + math.dist(clean[i - 1], clean[i]))
        length = cumulative[-1]
        if length <= 1e-9:
            return [clean[0]], 0.0, 0.0

        speed = max(float(speed), 1e-3)
        period = max(float(period), 1e-3)
        ramp_time = max(float(ramp_time), period)
        long_stroke = length >= speed * ramp_time
        if long_stroke:
            total_time = length / speed + ramp_time
        else:
            # quintic 位移曲线最大归一化速度为 1.875；据此限制峰值速度。
            total_time = max(1.875 * length / speed, 2.0 * period)

        n = max(2, int(math.ceil(total_time / period)))
        samples = []
        for i in range(n + 1):
            t = min(i * period, total_time)
            if i == n:
                t = total_time
            if long_stroke:
                if t < ramp_time:
                    u = t / ramp_time
                    s = speed * ramp_time * self._quintic_velocity_integral(u)
                elif t <= total_time - ramp_time:
                    s = 0.5 * speed * ramp_time + speed * (t - ramp_time)
                else:
                    u = (total_time - t) / ramp_time
                    s = length - speed * ramp_time * self._quintic_velocity_integral(u)
            else:
                s = length * self._quintic(t / total_time)
            samples.append(self._polyline_point(clean, cumulative, s))
        samples[-1] = clean[-1]
        return samples, length, total_time

    def _build_fixed_servo_targets(self, targets, speed, period,
                                   skip_landing=False,
                                   append_release=True):
        """组合“软落笔 + 笔画 + 微抬笔”，所有输出点均按 period 执行。

        ``skip_landing`` 只用于 Bézier 跳笔：跳笔曲线已经在同一条伺服流中
        到达下一笔触纸点，因此不能再次从 z_up 重复落笔。
        """
        if not targets:
            return [], {"length_mm": 0.0, "planned_s": 0.0}

        targets = [tuple(float(v) for v in p) for p in targets]
        ramp_time = float(self.writing.get("stroke_ramp_time_s", 0.06))
        stroke, length, stroke_time = self._sample_stroke_fixed_period(
            targets, speed, period, ramp_time
        )
        out = []
        planned = 0.0

        # 不再只照顾“点”：每一笔都从起点上方以五次曲线落下，触纸瞬间速度为 0。
        do_land = bool(self.writing.get("soft_landing", True))
        all_strokes = bool(self.writing.get("soft_landing_all_strokes", True))
        max_len = float(self.writing.get("soft_landing_max_len_mm", 8.0))
        if (not skip_landing and do_land
                and (all_strokes or length <= max_len)):
            x0, y0, z0 = targets[0]
            z_start = max(float(self.z_up), z0)
            landing_time = max(
                float(self.writing.get("landing_time_s", 0.18)), period
            )
            n = max(2, int(math.ceil(landing_time / period)))
            for i in range(n + 1):
                u = i / n
                q = self._quintic(u)
                out.append((x0, y0, z_start + (z0 - z_start) * q))
            planned += n * period

        if not out:
            out.append(stroke[0])
        for p in stroke:
            if math.dist(out[-1], p) > 1e-6:
                out.append(p)
        planned += stroke_time

        # 收笔先在同一伺服流里抬离纸面，再结束 ServoMove；防止服务切换时笔尖
        # 仍压在纸上形成墨团。随后主流程仍可用 MoveL 抬到完整安全高度。
        release_mm = max(float(self.writing.get("inline_release_mm", 2.0)), 0.0)
        if append_release and release_mm > 0.0:
            x1, y1, z1 = out[-1]
            z_end = min(float(self.z_up), z1 + release_mm)
            if z_end > z1 + 1e-6:
                release_time = max(
                    float(self.writing.get("inline_release_time_s", 0.06)), period
                )
                n = max(2, int(math.ceil(release_time / period)))
                for i in range(1, n + 1):
                    q = self._quintic(i / n)
                    out.append((x1, y1, z1 + (z_end - z1) * q))
                planned += n * period

        return out, {"length_mm": length, "planned_s": planned}

    def _blend_stroke_contact(self, targets, soft_entry=False,
                              soft_exit=False):
        """在笔画首尾一小段内渐变笔压，避免“写完、停住、再抬笔”。

        ``soft_entry`` 令第一点保持在纸面上方很小的距离，再沿前几个毫米
        平滑压到正常书写高度；``soft_exit`` 在最后几个毫米内逐渐抬到
        ``inline_release_mm``。两段使用五次曲线，位置、速度和加速度在
        边界都连续。这里只改变瞬时笔压，不改变配置中的绝对书写/抬笔高度。

        返回 ``(新目标, 是否应用软落笔, 是否应用软抬笔)``。单点或极短笔画
        无法沿轨迹渐变，会自动保留原来的竖直软落笔/微抬笔方案。
        """
        clean = [tuple(float(v) for v in p) for p in targets]
        if len(clean) < 2:
            return clean, False, False

        cumulative = [0.0]
        for i in range(1, len(clean)):
            cumulative.append(
                cumulative[-1] + math.dist(clean[i - 1], clean[i])
            )
        length = cumulative[-1]
        min_length = max(float(self.writing.get(
            "contact_blend_min_stroke_mm", 1.0)), 0.0)
        if length < max(min_length, 1e-6):
            return clean, False, False

        # 首尾各不超过笔画长度的45%，确保中间仍有正常书写高度，二者不重叠。
        entry_distance = min(max(float(self.writing.get(
            "soft_entry_blend_mm", 2.0)), 0.0), 0.45 * length)
        exit_distance = min(max(float(self.writing.get(
            "soft_exit_blend_mm", 2.5)), 0.0), 0.45 * length)
        entry_offset = max(float(self.writing.get(
            "soft_entry_offset_mm", 0.6)), 0.0)
        release = max(float(self.writing.get(
            "inline_release_mm", 2.0)), 0.0)

        use_entry = bool(soft_entry and entry_distance > 1e-6
                         and entry_offset > 1e-6)
        use_exit = bool(soft_exit and exit_distance > 1e-6
                        and release > 1e-6)
        if not use_entry and not use_exit:
            return clean, False, False

        # 即使原轨迹只含首尾两点，也显式插入渐变边界；否则首尾偏移会被
        # 直接线性连接，整条短笔画可能始终没有达到正常书写高度。
        distances = list(cumulative)
        if use_entry:
            distances.append(entry_distance)
        if use_exit:
            distances.append(length - exit_distance)
        distances = sorted(set(round(v, 9) for v in distances))

        out = []
        for s in distances:
            p = self._polyline_point(clean, cumulative, s)
            dz = 0.0
            if use_entry and s < entry_distance:
                # 第一目标轻触/悬在纸面上方；到 entry_distance 时完全落下。
                dz += entry_offset * (1.0 - self._quintic(s / entry_distance))
            if use_exit and s > length - exit_distance:
                # 尚在沿笔画前进时就卸载笔压，不再到末点后停住再竖直抬。
                u = (s - (length - exit_distance)) / exit_distance
                dz += release * self._quintic(u)
            out.append((p[0], p[1], min(float(self.z_up), p[2] + dz)))
        return out, use_entry, use_exit

    def _build_bezier_jump_samples(self, start, end, period):
        """生成一条从上一笔微抬点到下一笔触纸点的三次 Bézier 跳笔。

        P1/P2 位于两个端点正上方，所以曲线开始时主要向上、结束时主要
        向下；它们只是控制点，不会让机器人停顿。高度随二维距离自适应。
        返回的曲线再按弧长和固定周期重采样，供 ServoJ/ServoCart 使用。
        """
        p0 = tuple(float(v) for v in start)
        p3 = tuple(float(v) for v in end)
        distance_xy = math.hypot(p3[0] - p0[0], p3[1] - p0[1])

        min_h = max(float(self.writing.get(
            "bezier_jump_min_height_mm", 6.0)), 2.0)
        max_h = max(float(self.writing.get(
            "bezier_jump_max_height_mm", 12.0)), min_h)
        base_h = float(self.writing.get("bezier_jump_base_height_mm", 3.0))
        ratio = max(float(self.writing.get("bezier_jump_height_ratio", 0.20)), 0.0)
        height = min(max(base_h + ratio * distance_xy, min_h), max_h)

        # 对 z0 != z3（纸面有倾斜/逐字 Z 补偿）也令 t=0.5 附近达到
        # max(z0,z3)+height。两个控制点同高，同时保持端点竖直切向。
        apex_z = max(p0[2], p3[2]) + height
        control_z = (apex_z - 0.125 * (p0[2] + p3[2])) / 0.75
        p1 = (p0[0], p0[1], control_z)
        p2 = (p3[0], p3[1], control_z)

        # 先生成足够密的几何曲线，再统一做固定周期时间参数化。
        raw_count = max(24, min(160, int(math.ceil(
            (distance_xy + 2.0 * height) / 1.5
        ))))
        raw = []
        for i in range(raw_count + 1):
            u = i / raw_count
            a = (1.0 - u) ** 3
            b = 3.0 * (1.0 - u) ** 2 * u
            c = 3.0 * (1.0 - u) * u ** 2
            d = u ** 3
            raw.append(tuple(
                a * p0[k] + b * p1[k] + c * p2[k] + d * p3[k]
                for k in range(3)
            ))

        speed = max(float(self.writing.get(
            "bezier_jump_speed_mm_s", 180.0)), 1.0)
        ramp = max(float(self.writing.get(
            "bezier_jump_ramp_time_s", 0.05)), period)
        samples, length, planned = self._sample_stroke_fixed_period(
            raw, speed, period, ramp
        )
        return samples, {
            "height_mm": height,
            "distance_xy_mm": distance_xy,
            "length_mm": length,
            "planned_s": planned,
        }

    def servo_write(self, targets, jump_from=None, smooth_takeoff=False):
        """书写伺服入口；fixed 为新方案，legacy 可一键恢复旧实现。"""
        mode = str(self.writing.get("servo_timing_mode", "fixed")).lower()
        if mode == "legacy":
            if jump_from is not None:
                self.get_logger().warn("legacy 模式不支持 Bézier 跳笔")
                return False
            return self._servo_write_legacy(targets)
        if mode != "fixed":
            self.get_logger().error(
                f"未知 servo_timing_mode={mode!r}，仅支持 fixed/legacy"
            )
            return False
        return self._servo_write_fixed(
            targets,
            jump_from=jump_from,
            smooth_takeoff=smooth_takeoff,
        )

    def _servo_write_fixed(self, targets, jump_from=None,
                           smooth_takeoff=False):
        """固定周期书写：笛卡尔等时重采样，再用 ServoJ 或 ServoCart 下发。"""
        t0 = time.time()
        speed = max(float(self.writing.get("speed_mm_s", 100.0)), 1.0)
        # 0.016s=62.5Hz，处在 ServoCart 文档允许的 0.001~0.016s 范围上限；
        # 对 ROS2 字符串服务比直接 SDK 的 1~8ms 更现实，也比旧代码最高 0.2s 稠密。
        period = float(self.writing.get("servo_period_s", 0.016))
        period = min(max(period, 0.004), 0.016)
        blended, entry_blended, exit_blended = self._blend_stroke_contact(
            targets,
            # 第一笔也先竖直落到“轻触高度”，再沿笔画前段逐渐完全落下；
            # 后续笔画则由 Bézier 曲线直接到同一个轻触高度。
            soft_entry=bool(
                self.writing.get("soft_landing", True)
                and self.writing.get("progressive_soft_entry", True)
            ),
            soft_exit=smooth_takeoff,
        )
        samples, info = self._build_fixed_servo_targets(
            blended, speed, period,
            # 跳笔本身已到达 blended[0]：长笔画是轻触高度，极短/单点
            # 笔画则直接到触纸点；两种情况都不能再插入一次 z_up 竖直落笔。
            skip_landing=(jump_from is not None),
            append_release=not exit_blended,
        )
        jump_info = None
        if jump_from is not None and targets:
            jump_samples, jump_info = self._build_bezier_jump_samples(
                jump_from, blended[0], period
            )
            if jump_samples:
                # 跳笔终点和书写首点相同；只保留一个，整段只启动一次伺服。
                samples = jump_samples + samples[1:]
                info["planned_s"] += jump_info["planned_s"]
        if not samples:
            return True

        command_mode = str(
            self.writing.get("servo_command_mode", "servoj")
        ).lower()
        if command_mode not in ("servoj", "servocart"):
            self.get_logger().error(
                f"未知 servo_command_mode={command_mode!r}，仅支持 servoj/servocart"
            )
            return False

        current = self.read_joints()
        if current is None:
            self.get_logger().warn("固定周期伺服: 无法读取当前关节角")
            return False

        # ServoJ 必须先为每个笛卡尔点求连续关节解；ServoCart 由控制器实时解算，
        # 但仍预检整条轨迹的 IK，防止运动到一半才发现不可达/换解。
        joints = []
        prev = current
        for (x, y, z) in samples:
            fx, fy, fz = self._pen_to_flange((x, y, z))
            j = self.ik_flange(fx, fy, fz, joint_ref=prev, fast=True)
            if j is None:
                self.get_logger().warn(
                    f"固定周期伺服: 逆解失败 @({x:.0f},{y:.0f},{z:.1f})"
                )
                return False
            dmax = max(
                abs((j[k] - prev[k] + 180.0) % 360.0 - 180.0)
                for k in range(6)
            )
            if dmax > 25.0:
                self.get_logger().warn(f"固定周期伺服: 关节跳变 {dmax:.1f}°")
                return False
            joints.append(j)
            prev = j

        entry_limit = float(self.writing.get("servo_entry_max_jump_deg", 8.0))
        entry_jump = max(
            abs((joints[0][k] - current[k] + 180.0) % 360.0 - 180.0)
            for k in range(6)
        )
        if entry_jump > entry_limit:
            self.get_logger().warn(
                f"固定周期伺服: 首点关节跳变 {entry_jump:.1f}° > "
                f"{entry_limit:.1f}°，拒绝执行"
            )
            return False

        ik_elapsed = time.time() - t0
        self.get_logger().info(
            f"固定周期{command_mode}: {len(samples)}点/{period*1000:.0f}ms，"
            f"笔画{info['length_mm']:.1f}mm，计划{info['planned_s']:.2f}s，"
            f"IK预检{ik_elapsed:.2f}s"
        )
        if jump_info is not None:
            self.get_logger().info(
                f"Bézier跳笔: 平面距离{jump_info['distance_xy_mm']:.1f}mm，"
                f"弧高{jump_info['height_mm']:.1f}mm，"
                f"曲线{jump_info['length_mm']:.1f}mm/"
                f"{jump_info['planned_s']:.2f}s"
            )
        if entry_blended or exit_blended:
            self.get_logger().info(
                "笔压渐变: "
                + ("软落笔 " if entry_blended else "")
                + ("软抬笔" if exit_blended else "")
            )
        if self.call("ServoMoveStart(0)").strip() != "0":
            self.get_logger().warn("固定周期伺服: ServoMoveStart 失败")
            return False

        tick_elapsed = []
        recovered_transients = 0
        for i, ((x, y, z), j) in enumerate(zip(samples, joints), 1):
            ts = time.perf_counter()
            if command_mode == "servoj":
                cmd = (
                    f"ServoJ({j[0]:.3f},{j[1]:.3f},{j[2]:.3f},{j[3]:.3f},"
                    f"{j[4]:.3f},{j[5]:.3f},0,0,0,0,{period:.4f})"
                )
            else:
                fx, fy, fz = self._pen_to_flange((x, y, z))
                # mode=0 绝对笛卡尔伺服；外部轴为0，增量增益在绝对模式不参与。
                cmd = (
                    f"ServoCart(0,{fx:.3f},{fy:.3f},{fz:.3f},"
                    f"{self.rx:.3f},{self.ry:.3f},{self.rz:.3f},"
                    "0,0,0,0,1,1,1,1,1,1,0,0,"
                    f"{period:.4f},0,0)"
                )
            # -4 是 SDK 的 ERR_XMLRPC_CMD_FAILED。高频 ROS 服务 -> command_server
            # -> SDK/XML-RPC 链路可能偶发返回它。重复“同一个”位置目标是幂等的：
            # 不跳点、不前进轨迹，只在很短时间窗内重试；其它错误绝不吞掉。
            retry_count = max(int(self.writing.get(
                "servo_transient_retry_count", 3)), 0)
            retry_wait = max(float(self.writing.get(
                "servo_transient_retry_wait_s", 0.002)), 0.0)
            retry_max_gap = max(float(self.writing.get(
                "servo_transient_retry_max_gap_s", 0.080)), period)
            r = self.call_fast(cmd)
            retries = 0
            while (r.strip() == "-4" and retries < retry_count
                   and time.perf_counter() - ts < retry_max_gap):
                retries += 1
                if retry_wait > 0.0:
                    time.sleep(retry_wait)
                r = self.call_fast(cmd)
            elapsed = time.perf_counter() - ts
            tick_elapsed.append(elapsed)
            if r.strip() == "0" and retries:
                recovered_transients += 1
                self.get_logger().warn(
                    f"固定周期伺服: 第{i}/{len(samples)}点发生-4，"
                    f"同点重试{retries}次后恢复（{elapsed*1000:.1f}ms）"
                )
            if r.strip() != "0":
                end_r = self.call("ServoMoveEnd(0)")
                # 失败后才查询，避免在正常高频流中增加任何额外 RPC 负载。
                com_state = self.call("GetSDKComState()")
                robot_error = self.call("GetRobotErrorCode()")
                self.get_logger().error(
                    f"固定周期伺服: 第{i}/{len(samples)}点返回 {r}；"
                    f"重试{retries}次/耗时{elapsed*1000:.1f}ms；"
                    f"ServoMoveEnd={end_r}，SDK通信={com_state}，"
                    f"机器人报警={robot_error}；已停止，禁止从笔画起点自动回放"
                )
                return None
            # 恢复点刚刚才被控制器接受，给它完整一个周期再发下一点；正常点仍
            # 保持固定的“起点到起点”周期，不额外拖慢整条轨迹。
            rest = period if retries else period - elapsed
            if rest > 0.0:
                time.sleep(rest)

        r = self.call("ServoMoveEnd(0)")
        if r.strip() != "0":
            self.get_logger().error(f"固定周期伺服: ServoMoveEnd 返回 {r}")
            return None
        overruns = sum(v > period for v in tick_elapsed)
        worst = max(tick_elapsed, default=0.0)
        if overruns:
            self.get_logger().warn(
                f"固定周期伺服: {overruns}/{len(tick_elapsed)} 次服务往返超过 "
                f"{period*1000:.0f}ms（最慢{worst*1000:.0f}ms）；"
                "若比例持续偏高请恢复 legacy 或改用直接 SDK"
            )
        self.get_logger().info(
            f"固定周期伺服完成: 实际下发{sum(max(period, v) for v in tick_elapsed):.2f}s，"
            f"总计{time.time()-t0:.2f}s，"
            f"已恢复瞬时-4 {recovered_transients}次"
        )
        return True

    def _servo_write_legacy(self, targets):
        """把一串笔尖目标(base mm)用 ServoJ 连续流回放。

        逐点 CARTPoint+MoveL+轮询 每个点有约 1 秒固定开销；这里先整段逆解，
        再用 ServoJ 连续下发，速度快很多。
        失败（逆解失败 / 关节跳变 / ServoJ 报错）返回 False，调用方回退逐点模式。
        """
        _t0 = time.time()
        speed = max(float(self.writing.get("speed_mm_s", 100.0)), 1.0)
        # ---- 软着陆：只对"点"（很短的笔画）做，避免砸毛；长笔画保持原速省时间 ----
        _len = 0.0
        for _k in range(1, len(targets)):
            _len += math.dist(targets[_k - 1], targets[_k])
        _max_len = float(self.writing.get("soft_landing_max_len_mm", 8.0))
        if (self.writing.get("soft_landing", True) and targets
                and _len <= _max_len):
            x0, y0, z0 = targets[0]
            zu = float(self.z_up)
            if zu > z0:
                # 三次方渐慢：越接近纸面，每一步的高度差越小 -> 落笔前明显减速
                pre = []
                for k in (5, 4, 3, 2, 1):
                    f = (k / 5.0) ** 3
                    pre.append((x0, y0, z0 + (zu - z0) * f))
                targets = pre + list(targets)
                self.get_logger().info(
                    f"软着陆: 短笔画 {_len:.1f}mm，分5步降落"
                )
        joints = []
        # 第一目标也要参考真实当前姿态；之后每一点参考上一点，保持同一逆解分支。
        current = self.read_joints()
        if current is None:
            self.get_logger().warn("ServoJ流: 无法读取当前关节角 -> 回退")
            return False
        prev = current
        for (x, y, z) in targets:
            fx, fy, fz = self._pen_to_flange((x, y, z))
            j = self.ik_flange(fx, fy, fz, joint_ref=prev)
            if j is None:
                self.get_logger().warn(f"ServoJ流: 逆解失败 @({x:.0f},{y:.0f},{z:.1f})")
                return False
            dmax = max(
                abs((j[k] - prev[k] + 180.0) % 360.0 - 180.0)
                for k in range(6)
            )
            if dmax > 25.0:
                self.get_logger().warn(f"ServoJ流: 关节跳变 {dmax:.1f}° -> 回退")
                return False
            joints.append(j)
            prev = j
        if not joints:
            return True
        # 不能只检查轨迹内部相邻点：还要检查“机器人当前姿态 -> 第一个伺服点”。
        # 否则第一帧可能发生很大的关节跳变，表现为猛砸纸面或触发控制器保护。
        entry_limit = float(self.writing.get("servo_entry_max_jump_deg", 8.0))
        entry_jump = max(
            abs((joints[0][k] - current[k] + 180.0) % 360.0 - 180.0)
            for k in range(6)
        )
        if entry_jump > entry_limit:
            self.get_logger().warn(
                f"ServoJ流: 首点相对当前姿态跳变 {entry_jump:.1f}° "
                f"> {entry_limit:.1f}° -> 回退"
            )
            return False
        L = 0.0
        for k in range(1, len(targets)):
            L += math.dist(targets[k - 1], targets[k])
        dt = L / speed / max(len(targets) - 1, 1)
        dt = min(max(dt, 0.010), 0.20)
        _tik = time.time()
        self.get_logger().info(
            f"ServoJ流: {len(joints)} 点, 逆解用时 {_tik-_t0:.2f}s, dt={dt:.3f}s"
        )
        if self.call("ServoMoveStart(0)").strip() != "0":
            self.get_logger().warn("ServoJ流: ServoMoveStart 失败")
            return False
        for _i, j in enumerate(joints, 1):
            _ts = time.time()
            r = self.call_fast(
                f"ServoJ({j[0]:.3f},{j[1]:.3f},{j[2]:.3f},{j[3]:.3f},"
                f"{j[4]:.3f},{j[5]:.3f},0,0,0,0,{dt:.4f})"
            )
            if r.strip() != "0":
                self.call("ServoMoveEnd(0)")
                # 已经开始运动后再从整条笔画起点回退，会拖着笔逆向划过纸面。
                # None 表示“运动中故障，只能停止，禁止自动回放整条笔画”。
                self.get_logger().error(f"ServoJ流: 运动中 ServoJ 返回 {r}，停止且禁止回退")
                return None
            # 节拍：默认 dt；可选加减速斜坡 / 拐角降速
            _n = len(joints)
            _dt_here = dt
            if self.writing.get("soft_ramp", False) and _n > 3:
                _w = float(self.writing.get("ramp_width", 0.15))
                _slow = float(self.writing.get("ramp_slowest", 0.50))
                _f = (_i - 1) / (_n - 1)
                _r = min(_f / _w, (1.0 - _f) / _w, 1.0)
                _dt_here = dt / max(_slow + (1.0 - _slow) * _r, _slow)
            # 拐角降速：方向变化大的点放慢，避免伺服跟随误差把拐角抹圆
            if (self.writing.get("corner_slow", True) and 2 <= _i < _n):
                _p0 = targets[_i - 2]; _p1 = targets[_i - 1]; _p2 = targets[_i]
                _v1 = (_p1[0] - _p0[0], _p1[1] - _p0[1])
                _v2 = (_p2[0] - _p1[0], _p2[1] - _p1[1])
                _n1 = math.hypot(*_v1); _n2 = math.hypot(*_v2)
                if _n1 > 0.2 and _n2 > 0.2:
                    _cos = (_v1[0]*_v2[0] + _v1[1]*_v2[1]) / (_n1 * _n2)
                    _cos = max(-1.0, min(1.0, _cos))
                    _ang = math.degrees(math.acos(_cos))
                    _thr = float(self.writing.get("corner_angle_deg", 60.0))
                    if _ang > _thr:
                        _dt_here *= float(self.writing.get("corner_slow_factor", 2.0))
            _rest = _dt_here - (time.time() - _ts)
            if _rest > 0:
                time.sleep(_rest)
        r = self.call("ServoMoveEnd(0)")
        _el = time.time() - _tik
        if r.strip() != "0":
            self.get_logger().warn(f"ServoJ流: ServoMoveEnd 返回 {r}")
            return None
        self.get_logger().info(
            f"ServoJ流: 下发用时 {_el:.2f}s ({_el/max(len(joints),1)*1000:.0f}ms/点), "
            f"总 {time.time()-_t0:.2f}s"
        )
        return True

    def read_flange(self):
        """读当前法兰位姿，返回 (x,y,z,rx,ry,rz) mm/deg，失败返回 None。
        关键：CARTPoint 存法兰位姿、MoveJ/MoveL 移动法兰，故到位检测读法兰。
        （GetActualTCPPose 在工具坐标生效时会返回笔尖 TCP，故统一用
        GetActualToolFlangePose，保证与 CARTPoint 里存的法兰位姿一致。）"""
        res = self.call("GetActualToolFlangePose(1)")
        parts = res.split(",")
        if parts[0].strip() != "0" or len(parts) < 7:
            return None
        try:
            return [float(v) for v in parts[1:7]]
        except ValueError:
            return None

    def wait_arrive(self, pen_target_mm, tol_mm=3.0, timeout=8.0, resend=None, tries=3):
        """轮询直到法兰到达「笔尖目标 pen_target_mm 对应的法兰位姿」附近，超时返回 False。
        pen_target_mm 为笔尖坐标(mm)，内部换算成法兰坐标再与当前法兰位姿对比。
        resend 非空时，超时会自动重发移动指令（最多 tries 次，防偶发指令丢失）。"""
        flange_target = self._pen_to_flange(pen_target_mm)
        for attempt in range(tries):
            t0 = time.time()
            while time.time() - t0 < timeout:
                cur = self.read_flange()
                if cur is None:
                    return False
                dist = math.sqrt(
                    (cur[0] - flange_target[0]) ** 2
                    + (cur[1] - flange_target[1]) ** 2
                    + (cur[2] - flange_target[2]) ** 2
                )
                if dist <= tol_mm:
                    return True
                time.sleep(0.05)
            if resend is not None and attempt < tries - 1:
                self.get_logger().warn(
                    f"到位超时（第 {attempt + 1} 次，偏差 > {tol_mm}mm），重发移动指令"
                )
                try:
                    resend()
                except Exception as e:  # noqa: BLE001
                    self.get_logger().error(f"重发失败: {e}")
                time.sleep(0.3)
        return False

    def install_tool(self):
        """清零 tool_id 坐标系（幂等）。

        执行器在软件里手工做「笔尖 <-> 法兰」换算（_pen_to_flange），
        控制器不能再叠加一次工具偏移，否则 GetActualTCPPose 会变成笔尖、
        和 CARTPoint 里存的法兰语义冲突（曾导致到位检测误判超时）。
        """
        r = self.call(f"SetToolCoord({self.tool_id},0,0,0,0,0,0)")
        self.get_logger().info(f"SetToolCoord(清零) -> {r}")
        return r.strip() == "0"

    # -- 主流程 --------------------------------------------------------------
    def read_joints(self):
        res = self.call("GetActualJointPosDegree(1)")
        parts = res.split(",")
        if parts[0].strip() != "0" or len(parts) < 7:
            return None
        try:
            return [float(v) for v in parts[1:7]]
        except ValueError:
            return None

    def servo_move_j(self, target, duration=1.5, steps=100):
        """ServoJ 关节插值回放（取笔/蘸墨示教点）。

        duration 决定整段动作时间；cmdT 只决定刷新周期。这里用五次曲线代替
        原来的线性关节插值，并保证每帧不长于 joint_servo_period_s，避免起停瞬间
        速度突变。缩短 cmdT 本身不会让整段更快，真正提速应调整 duration。
        """
        start = self.read_joints()
        if start is None:
            self.get_logger().error("读关节角失败")
            return False
        d = max(abs(target[k] - start[k]) for k in range(6))
        self.get_logger().info(f"ServoJ 移动: 最大关节差 {d:.1f}°")
        r = ""
        for attempt in range(4):
            r = self.call("ServoMoveStart(0)")
            if r.strip() == "0":
                break
            self.get_logger().warn(
                f"ServoMoveStart 失败({r})，等待后重试（第 {attempt + 1} 次）"
            )
            time.sleep(1.0)
        if r.strip() != "0":
            self.get_logger().error(f"ServoMoveStart 失败: {r}")
            return False
        max_period = float(self.writing.get("joint_servo_period_s", 0.016))
        max_period = min(max(max_period, 0.004), 0.016)
        # 传入的 steps 仍作为最低采样数；若它导致周期过长，则自动加密。
        steps = max(int(steps), int(math.ceil(duration / max_period)), 6)
        step_sleep = duration / steps
        for i in range(1, steps + 1):
            # 五次时间律：起点/终点的速度、加速度均为0，取笔和蘸墨不会突然启停。
            t = self._quintic(i / steps)
            j = [start[k] + t * (target[k] - start[k]) for k in range(6)]
            cmd = (f"ServoJ({j[0]:.3f},{j[1]:.3f},{j[2]:.3f},{j[3]:.3f},"
                   f"{j[4]:.3f},{j[5]:.3f},0,0,0,0,{step_sleep:.3f})")
            _ts = time.time()
            rr = self.call_fast(cmd)
            if rr.strip() != "0":
                self.get_logger().error(f"ServoJ step {i}/{steps}: {rr}，立即结束伺服")
                self.call("ServoMoveEnd(0)")
                return False
            _rest = step_sleep - (time.time() - _ts)
            if _rest > 0:
                time.sleep(_rest)
        r2 = self.call("ServoMoveEnd(0)")
        if r2.strip() != "0":
            self.get_logger().error(f"ServoMoveEnd 失败: {r2}")
            return False

        # ServoMoveEnd 返回 0 只表示指令被接受；再核对一次最终关节位置，防止“假成功”。
        tol = float(self.writing.get("joint_arrive_tol_deg", 1.5))
        deadline = time.time() + float(self.writing.get("joint_arrive_timeout_s", 2.0))
        while time.time() < deadline:
            actual = self.read_joints()
            if actual is not None:
                err = max(
                    abs((target[k] - actual[k] + 180.0) % 360.0 - 180.0)
                    for k in range(6)
                )
                if err <= tol:
                    return True
            time.sleep(0.05)
        self.get_logger().error(f"ServoJ 结束但未到目标关节位置（容差 {tol:.1f}°）")
        return False

    def grasp(self):
        """机械手握爪（抓笔）。"""
        if self.hand_pub is None:
            self.get_logger().error("机械手 publisher 未初始化")
            return False
        # ROS 发布本身没有“抓住笔”的反馈，但至少确认手节点已经订阅，
        # 避免在驱动未启动时把 60 次命令全部发到空气里却继续运行。
        deadline = time.time() + float(self.ink.get("hand_wait_subscriber_s", 5.0))
        while self.hand_pub.get_subscription_count() < 1 and time.time() < deadline:
            time.sleep(0.05)
        if self.hand_pub.get_subscription_count() < 1:
            self.get_logger().error("灵巧手命令话题没有订阅者，停止取笔流程")
            return False
        m = JointState()
        m.name = list(HAND_NAMES)
        m.position = list(HAND_GRASP)
        for _ in range(60):
            self.hand_pub.publish(m)
            time.sleep(0.05)
        self.get_logger().info("机械手握爪指令已发布")
        return True

    def dip_ink(self):
        """蘸墨：读悬停法兰位姿，逆解出下探关节，ServoJ 下探再抬起（全程伺服）。"""
        depth = float(self.ink.get("dip_depth_mm", 20.0))
        hover = self.read_joints()
        if hover is None:
            self.get_logger().error("蘸墨：读关节位置失败")
            return False
        # 优先使用实机示教过的触墨关节位，避免每次重新逆解选到另一条关节分支。
        down = self.ink.get("dip_touch_joint")
        if down:
            down = [float(v) for v in down]
        else:
            flange = self.read_flange()
            if flange is None:
                self.get_logger().error("蘸墨：读法兰位姿失败")
                return False
            down_flange = (flange[0], flange[1], flange[2] - depth,
                           flange[3], flange[4], flange[5])
            res = self.call(
                f"GetInverseKin(0,{down_flange[0]:.3f},{down_flange[1]:.3f},{down_flange[2]:.3f},"
                f"{down_flange[3]:.3f},{down_flange[4]:.3f},{down_flange[5]:.3f},-1)"
            )
            parts = res.split(",")
            if parts[0].strip() != "0" or len(parts) < 7:
                self.get_logger().error(f"蘸墨：下探逆解失败 {res}")
                return False
            down = [float(v) for v in parts[1:7]]
        dt = float(self.ink.get("dip_move_s", 1.0))
        if not self.servo_move_j(down, duration=dt, steps=30):
            return False
        if not self.servo_move_j(hover, duration=dt, steps=30):
            return False
        return True

    def set_posture(self, ai):
        """按字设置书写姿态。

        优先用 posture_by_char[ai] 指定的 (rx,ry,rz)（用于个别字需要不同姿态以避开
        逆解翻转分支的情况）；否则用 rx_by_column（第 1~10 字 / 第 11~20 字）+ 全局 ry/rz。
        """
        i = ai % 20        # 一首诗 = 2 行 × 10 字；连写两首时各按自己的列姿态
        ov = (self.cfg.get("posture_by_char") or {}).get(str(i))
        if ov:
            rx, ry, rz = float(ov["rx"]), float(ov["ry"]), float(ov["rz"])
        else:
            rx = self.rx_by_column[0 if i < 10 else 1]
            ry, rz = self.ry, self.rz
        if (rx, ry, rz) != (self.rx, self.ry, self.rz):
            self.rx, self.ry, self.rz = rx, ry, rz
            self._flange_R = self._rpy_to_R(rx, ry, rz)

    def to_base(self, x_m, y_m, z_mm):
        """桌面坐标 -> 基坐标，返回 (bx_m, by_m, z_mm)。z 直接用给定绝对高度。"""
        bx, by, _ = self.transform(x_m, y_m)
        return bx, by, z_mm

    def paper_z(self, x_m, y_m, ai=None):
        """纸面平面的书写高度(mm)。有 paper_plane 配置时按平面插值，否则用固定 z_down。"""
        pp = self.cfg.get("paper_plane")
        if pp:
            z = ((pp["z0"] + pp["a"] * x_m + pp["b"] * y_m) * 1000.0
                 + pp.get("offset_mm", 0.0))
        else:
            z = self.z_down
        if ai is not None:
            z += self.cfg.get("z_offsets_mm", {}).get(str(ai), 0.0)
        return z

    def run(self):
        cfg = self.cfg
        # 先安装工具坐标系，再干活
        if not self.install_tool():
            self.get_logger().error("SetToolCoord 失败，停止。")
            return
        spd = cfg["flow"].get("global_speed", 100)
        self.get_logger().info(
            f"SetSpeed({spd}) -> {self.call(f'SetSpeed({int(spd)})').strip()}"
        )

        traj_path = cfg["trajectory_json"]
        if traj_path.startswith("~"):
            import os
            traj_path = os.path.expanduser(traj_path)

        data = json.load(open(traj_path, encoding="utf-8"))
        assignments = data["assignments"]

        start = cfg["flow"].get("start_from_character", 0)
        stop = cfg["flow"].get("stop_after_character")
        if stop is None:
            stop = len(assignments)

        if self.z_down is None or self.z_up is None:
            self.get_logger().error("z_heights_mm 未填写（down/up），请先示教。")
            return

        _src = data.get("source_text", "")
        _no = cfg.get("poem_no")
        self.get_logger().info(
            f"共 {len(assignments)} 字，本次执行第 {start+1}~{stop} 字"
            + (f"；第 {_no} 首：{_src}" if _no else (f"；{_src}" if _src else ""))
        )
        t_all = time.time()

        # 取笔（完整任务必须执行；仅“笔仍在手上的续写”允许显式跳过）
        _pickup_lifted = False      # 取笔块内可能置 True；跳过取笔时必须为 False
        _skip_pickup = bool(cfg["flow"].get("skip_pickup", False))
        if not _skip_pickup and not self.ink.get("pickup_joint"):
            self.get_logger().error("本次要求取笔，但 ink.pickup_joint 缺失，停止。")
            return
        if not _skip_pickup:
            self.get_logger().info("取笔")
            if self.ink.get("pre_pickup_joint"):
                self.get_logger().info("取笔前先到「靠近笔前」过渡位")
                _pre_s = float(self.ink.get("pre_pickup_s", 1.5) or 1.5)
                if not self.servo_move_j(self.ink["pre_pickup_joint"], duration=_pre_s):
                    self.get_logger().error("到「靠近笔前」失败，停止。")
                    return
            _pickup_s = float(self.ink.get("pickup_move_s", 1.2) or 1.2)
            if not self.servo_move_j(self.ink["pickup_joint"], duration=_pickup_s):
                self.get_logger().error("取笔移动失败，停止。")
                return
            if not self.grasp():
                self.get_logger().error("抓笔命令未能可靠下发，停止。")
                return
            # 给灵巧手留足合拢时间：ROS 发布 60 条指令只花 3s，手实际合拢可能 4~6s。
            # 太早抬笔会导致"手还没抓住，机械臂已经走了"。
            time.sleep(float(self.ink.get("sleep_after_grasp_s", 3.0)))
            # 取笔后竖直上提（清空笔架开口）；ink.pickup_lift_mm 可配置，0 = 不抬直接去安全位
            lift_mm = float(self.ink.get("pickup_lift_mm", 150.0) or 0.0)
            if lift_mm > 0:
                flange = self.read_flange()
                if flange is None:
                    self.get_logger().error("取笔后无法读取法兰位姿，不能确认安全上提，停止。")
                    return
                res = self.call(
                    f"GetInverseKin(0,{flange[0]:.3f},{flange[1]:.3f},"
                    f"{flange[2]+lift_mm:.3f},"
                    f"{flange[3]:.3f},{flange[4]:.3f},{flange[5]:.3f},-1)"
                )
                parts = res.split(",")
                if parts[0].strip() != "0" or len(parts) < 7:
                    self.get_logger().error(f"取笔后上提逆解失败: {res}，停止。")
                    return
                lifted = [float(v) for v in parts[1:7]]
                _dur = min(max(lift_mm / 150.0 * 1.2, 0.4), 1.5)
                if not self.servo_move_j(lifted, duration=_dur,
                                         steps=max(10, int(_dur / 0.015))):
                    self.get_logger().error("取笔后上提失败，停止。")
                    return
                self.get_logger().info(f"取笔后竖直上提 {lift_mm:.0f}mm")
                _pickup_lifted = True
            else:
                self.get_logger().info("取笔后不抬升（pickup_lift_mm=0），直接去安全位")

        for ai in range(start, stop):
            a = assignments[ai]
            t_char = time.time()
            self.set_posture(ai)
            # 蘸墨（每字一次，可选）
            dip_n = int(cfg["flow"].get("dip_every_n", 2) or 2)
            if (cfg["flow"].get("dip_ink", True) and ai % dip_n == 0
                    and self.ink.get("dip_hover_joint")):
                self.get_logger().info(f"蘸墨（第 {ai+1} 字「{a['character']}」前）")
                _i = ai % 20
                _skip_safe = _pickup_lifted     # 取笔后已竖直抬升，第一次蘸墨不必再去安全位
                if (_i < 5 or 10 <= _i <= 13) and self.ink.get("safe_high_joint") \
                        and not _skip_safe:
                    self.get_logger().info("蘸墨前先到示教点")
                    if not self.servo_move_j(self.ink["safe_high_joint"]):
                        self.get_logger().error("到示教点失败，停止。")
                        return
                _pickup_lifted = False
                _dip_s = float(self.ink.get("dip_approach_s", 1.5) or 1.5)
                if not self.servo_move_j(self.ink["dip_hover_joint"], duration=_dip_s):
                    self.get_logger().error("蘸墨移动失败，停止。")
                    return
                if not self.dip_ink():
                    self.get_logger().error("蘸墨失败，停止。")
                    return
                # 本次运行的第一次蘸墨后停留一会儿（供人工换纸；笔悬在墨池上方，纸面是空的）
                pause = float(self.ink.get("dip_pause_s", 0.0) or 0.0)
                if pause > 0 and ai == start:
                    self.get_logger().info(f"蘸墨后停留 {pause:.0f} 秒（换纸时间）…")
                    time.sleep(pause)
                    self.get_logger().info("停留结束，继续写")
                # 原来这里有一绕：「蘸墨后抬到安全高位（避开笔架）」——
                # 笔架位置已变、不再需要避让，已按用户要求去掉。
            ch = a["character"]
            self.get_logger().info(f"开始写「{ch}」（第 {ai+1} 字）")

            # 只在同一个字内部连接笔画：跨字时可能切换姿态或执行蘸墨，仍走原来的
            # 安全抬笔 + MoveJ。空笔画先过滤，保证“下一笔”确实存在。
            _valid_strokes = [s for s in a["strokes"] if s.get("points")]
            _pending_jump_from = None
            for si, stroke in enumerate(_valid_strokes):
                pts = stroke["points"]
                eps = float(cfg["flow"].get("simplify_tol_mm", 0.0) or 0.0) / 1000.0
                if eps > 0:
                    _cd = float(cfg["flow"].get("corner_keep_deg", 150.0))
                    pts = _simplify_keep_corners(pts, eps, _cd)

                # 先生成本笔画的触纸目标。这里继续读取现有配置里的 paper_z、
                # z_offsets 等数据；Bézier 功能不会写回或覆盖任何标定高度。
                _targets = []
                for p in pts:
                    bx, by, _ = self.to_base(p["x_m"], p["y_m"], self.z_down)
                    _targets.append((bx * 1000.0, by * 1000.0,
                                     self.paper_z(p["x_m"], p["y_m"], ai)))
                _stroke_end = _targets[-1]
                _inline_lift = bool(self.writing.get("inline_lift", True))
                _has_next = si + 1 < len(_valid_strokes)
                _release_mm = max(float(self.writing.get(
                    "inline_release_mm", 2.0)), 0.0)
                _plan_smooth_takeoff = bool(
                    self.writing.get("progressive_soft_exit", True)
                    and str(self.writing.get(
                        "servo_timing_mode", "fixed")).lower() == "fixed"
                    and self.writing.get("use_servo_stream", True)
                    and not _inline_lift
                    and _release_mm > 0.0
                )
                _plan_jump_next = bool(
                    _has_next
                    and self.writing.get("use_bezier_jump", True)
                    and str(self.writing.get(
                        "servo_timing_mode", "fixed")).lower() == "fixed"
                    and self.writing.get("use_servo_stream", True)
                    and not _inline_lift
                    and _release_mm > 0.0
                )
                if _inline_lift and _targets:
                    _lx, _ly, _lz = _targets[-1]
                    _zu = float(self.z_up)
                    if _zu > _lz:
                        for _k in range(1, 6):
                            _targets.append((_lx, _ly,
                                             _lz + (_zu - _lz) * _k / 5.0))

                # 默认开启：只要上一笔已在固定周期流中可靠地微抬离纸面，下一笔就
                # 与它合成“一条 Bézier 跳笔 + 下一笔书写”的连续伺服流。
                _jump_from = _pending_jump_from
                _pending_jump_from = None
                _use_jump = bool(
                    _jump_from is not None
                    and self.writing.get("use_bezier_jump", True)
                    and str(self.writing.get(
                        "servo_timing_mode", "fixed")).lower() == "fixed"
                    and self.writing.get("use_servo_stream", True)
                    and not _inline_lift
                )

                # -- 非跳笔时：保持原来的安全方案，先到起点上方 --
                x0, y0 = pts[0]["x_m"], pts[0]["y_m"]
                bx, by, _ = self.to_base(x0, y0, self.z_up)
                _start_mm = (bx * 1000.0, by * 1000.0, float(self.z_up))
                if _use_jump:
                    self.get_logger().info(
                        f"  笔画{si+1}: Bézier 跳笔后直接书写"
                    )
                else:
                    flange = self._pen_to_flange(_start_mm)
                    _moved = False
                    # 注意：ServoJ 关节插值绕过控制器规划与碰撞保护，默认关闭。
                    if self.writing.get("use_servo_goto", False):
                        _moved = self.servo_goto(_start_mm)
                    if not _moved:
                        self.get_logger().info(
                            f"  笔画{si+1}: PTP -> base({bx*1000:.0f},"
                            f"{by*1000:.0f},{self.z_up:.1f})"
                        )
                        r = self.move_j(
                            flange, self.writing["approach_speed_mm_s"]
                        )
                        if r.strip() not in ("0", "14"):
                            self.get_logger().error(
                                f"PTP 接近被拒: {r}，终止执行。"
                            )
                            return
                        if not self.wait_arrive(
                            _start_mm, tol_mm=5.0, timeout=30.0,
                            resend=lambda f=flange: self.move_j(
                                f, self.writing["approach_speed_mm_s"]
                            ),
                        ):
                            self.get_logger().error(
                                "PTP 接近到位超时（控制器可能报错停止），终止执行。"
                            )
                            return

                # -- Bézier跳笔（若有）和本笔画在一次 ServoMoveStart/End 内完成 --
                _ok = False
                if self.writing.get("use_servo_stream", True):
                    _ok = self.servo_write(
                        _targets,
                        jump_from=_jump_from if _use_jump else None,
                        smooth_takeoff=_plan_smooth_takeoff,
                    )
                    if _ok is None:
                        self.get_logger().error(
                            "ServoJ 流在运动中失败；为防止带笔回划，终止而不自动回退。"
                        )
                        return
                    if _use_jump and _ok is False:
                        # 跳笔在预检/启动前失败，机器人仍停在上一笔微抬点。
                        # 先竖直抬到原 z_up，再恢复旧的 PTP 接近和普通书写。
                        self.get_logger().warn(
                            "Bézier 跳笔预检失败 -> 安全抬高后回退原方案"
                        )
                        px, py, _ = _jump_from
                        self.cart_point(1, px / 1000.0, py / 1000.0,
                                        self.z_up)
                        rr = self.move_l(1, self.writing["lift_speed_mm_s"])
                        if rr.strip() not in ("0", "14") or not self.wait_arrive(
                            (px, py, self.z_up), tol_mm=5.0, timeout=10.0,
                            resend=lambda x=px, y=py: (
                                self.cart_point(1, x / 1000.0, y / 1000.0,
                                                self.z_up),
                                self.move_l(1, self.writing["lift_speed_mm_s"]),
                            ),
                        ):
                            self.get_logger().error(
                                "跳笔回退时无法安全抬高，终止执行。"
                            )
                            return
                        flange = self._pen_to_flange(_start_mm)
                        r = self.move_j(
                            flange, self.writing["approach_speed_mm_s"]
                        )
                        if r.strip() not in ("0", "14") or not self.wait_arrive(
                            _start_mm, tol_mm=5.0, timeout=30.0,
                            resend=lambda f=flange: self.move_j(
                                f, self.writing["approach_speed_mm_s"]
                            ),
                        ):
                            self.get_logger().error(
                                "跳笔回退时无法到达下一笔上方，终止执行。"
                            )
                            return
                        _ok = self.servo_write(
                            _targets, smooth_takeoff=_plan_smooth_takeoff
                        )
                        if _ok is None:
                            self.get_logger().error(
                                "回退书写在运动中失败，终止执行。"
                            )
                            return
                    if _ok is False:
                        self.get_logger().warn("ServoJ 流失败 -> 回退逐点 MoveL")
                if not _ok:
                    for (tx, ty, tz) in _targets:
                        self.cart_point(1, tx / 1000.0, ty / 1000.0, tz)
                        rr = self.move_l(1, self.writing["speed_mm_s"])
                        if rr.strip() not in ("0", "14"):
                            self.get_logger().error(f"书写点被拒: {rr}，终止。")
                            return
                        if not self.wait_arrive(
                            (tx, ty, tz), tol_mm=3.0, timeout=10.0,
                            resend=lambda x=tx, y=ty, z=tz: (
                                self.cart_point(1, x / 1000.0, y / 1000.0, z),
                                self.move_l(1, self.writing["speed_mm_s"]),
                            ),
                        ):
                            self.get_logger().error(
                                "书写点到位超时，终止（避免拖着笔在纸上划）。"
                            )
                            return

                # -- 本笔画结束：有下一笔时保留微抬位置供 Bézier 跳笔 --
                _can_jump_next = bool(
                    _ok is True
                    and _plan_jump_next
                )
                _lifted = False
                if _can_jump_next:
                    _lx, _ly, _lz = _stroke_end
                    _release_z = min(float(self.z_up), _lz + _release_mm)
                    if _release_z > _lz + 1e-6:
                        _pending_jump_from = (_lx, _ly, _release_z)
                        _lifted = True

                # 最后一笔、禁用跳笔或回退书写时，保持原来的完整垂直抬笔。
                bx, by, _ = self.to_base(pts[-1]["x_m"], pts[-1]["y_m"], self.z_up)
                _lifted = _lifted or (_ok and _inline_lift)
                if (not _lifted
                        and self.writing.get("use_servo_goto", False)):
                    _lifted = self.servo_goto((bx * 1000, by * 1000, self.z_up),
                                              duration=0.15)
                if not _lifted:
                    self.cart_point(1, bx, by, self.z_up)
                    r = self.move_l(1, self.writing["lift_speed_mm_s"])
                    if r.strip() not in ("0", "14"):
                        self.get_logger().error(f"抬笔被拒: {r}，终止执行。")
                        return
                    if not self.wait_arrive(
                        (bx * 1000, by * 1000, self.z_up), tol_mm=5.0, timeout=10.0,
                        resend=lambda x=bx, y=by: (
                            self.cart_point(1, x, y, self.z_up),
                            self.move_l(1, self.writing["lift_speed_mm_s"]),
                        ),
                    ):
                        self.get_logger().error("抬笔到位超时，终止执行。")
                        return

            self.get_logger().info(
                f"「{ch}」完成（本字 {time.time() - t_char:.1f}s，累计 {time.time() - t_all:.1f}s）"
            )

        self.get_logger().info(
            f"全部书写完成，总用时 {time.time() - t_all:.1f}s"
        )
        return True


def main():
    dry = "--dry-run" in sys.argv
    cfg_path = "~/hanzi_task/hanzi_config.json"
    if "--config" in sys.argv:
        cfg_path = sys.argv[sys.argv.index("--config") + 1]
    import os
    cfg_path = os.path.expanduser(cfg_path)
    config = json.load(open(cfg_path, encoding="utf-8"))

    # 直接指定轨迹文件（连写两首时用 dual_trajectories.json）
    if "--traj" in sys.argv:
        traj = sys.argv[sys.argv.index("--traj") + 1]
        config["trajectory_json"] = traj
        # 连写两首：一个文件里就是全部字，取消单首 20 字的范围限制
        config.setdefault("flow", {})["start_from_character"] = 0
        config["flow"]["stop_after_character"] = None
        print(f"指定轨迹文件: {traj}（写完整文件，不限 20 字）")

    # 选诗：--poem 1 / 2 / 3
    if "--poem" in sys.argv:
        poem = str(sys.argv[sys.argv.index("--poem") + 1]).strip()
        poems = config.get("poems") or {}
        if poem not in poems:
            print(f"未知的诗编号 '{poem}'，可选：{sorted(poems)}")
            return
        config["trajectory_json"] = poems[poem]
        config["poem_no"] = poem
        print(f"已选择第 {poem} 首 -> {poems[poem]}")

    if dry:
        print("=== DRY RUN ===")
        print(f"轨迹文件: {config['trajectory_json']}")
        print(f"Z: down={config['z_heights_mm']['down']} "
              f"up={config['z_heights_mm']['up']}")
        print(f"速度: 书写{config['writing']['speed_mm_s']}mm/s "
              f"抬笔{config['writing']['lift_speed_mm_s']}mm/s")

        # 样例：验证坐标变换（用实机三点标定验证 cell0/cell9/cell10 中心）
        tf = build_transform()
        print("\n样例变换验证（实机三点标定）:")
        samples = [
            ("白 cell0", 0.457, -0.270, (373.2, -598.1)),
            ("流 cell9", 0.457, 0.470, (-370.2, -603.1)),
            ("河 cell10", 0.349, -0.270, (369.3, -492.4)),
        ]
        for name, tx, ty, expect in samples:
            bx, by, _ = tf(tx, ty)
            print(f"  {name}: table({tx},{ty}) -> base({bx*1000:.1f},{by*1000:.1f})mm "
                  f"(实测{expect})")
        return

    # 真实运行才 import ROS（dry-run 无需 ROS 环境）
    import rclpy as _rclpy
    from rclpy.node import Node as _Node
    from fairino_msgs.srv import RemoteCmdInterface as _RemoteCmdInterface
    global rclpy, Node, RemoteCmdInterface
    rclpy = _rclpy
    Node = _Node
    RemoteCmdInterface = _RemoteCmdInterface

    rclpy.init()
    node = HanziExecutor(config)
    success = False
    try:
        success = node.run() is True
    finally:
        node.destroy_node()
        rclpy.shutdown()
    # 过去 run() 中途 return 后进程仍返回0，外层脚本会出现“退出码0但未完成”。
    # 现在只有打印“全部书写完成”并正常走到末尾才返回0，其余失败统一返回1。
    if not success:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
