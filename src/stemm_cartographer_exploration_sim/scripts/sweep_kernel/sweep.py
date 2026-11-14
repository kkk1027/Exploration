"""sweep_kernel —— 扫掠式可通行性内核（S5 M2）。

要修的既有缺陷
--------------
`docs/P1_候选生成缺陷清单.md` D2：走廊宽度按**轴向法向**测量，斜向通道会
量出 `真实宽度 ÷ cos45° ≈ 1.41 倍`。根因是 BFS 父格差只能给出 4 连通方向
（`(±1,0)` / `(0,±1)`），法向必然轴对齐 —— 这不是参数没调好，是**信息不足**。

做法
----
不算"宽度"，改算**能不能放下车**：把 footprint 多边形沿路径扫过去，
逐个位姿检查多边形内是否落进障碍格。

    输入: 路径点序列 P、footprint 多边形 F、距离场 D
    1. 沿 P 按 step_m 重采样，位姿 yaw = 路径切向
    2. 把 F 的顶点按位姿变换到世界系
    3. 逐行扫描填充（crossing number）→ 多边形覆盖的栅格行区间
    4. 区间内 dist 的最小值 == 0 ⇒ 有多边形内有障碍格 ⇒ 放不下
    5. swept_min_clearance = 多边形**边界**采样点到最近障碍的距离最小值

为什么第 4 步用 `dist == 0` 判碰撞
------------------------------------
`clearance_field()` 的距离场由 `scipy.ndimage.distance_transform_edt(~blocked)`
算出，障碍格上**精确为 0**，相邻自由格为 1 格（= res）。
所以"多边形内部存在 dist==0 的格" ⟺ "多边形与障碍重叠"，
不需要再单独拿一张占据掩码，也不会因为浮点误差误判。

构型无关性（S5 §3 M2）
----------------------
同一份代码，换 profile 就能服务三种构型：

| 构型 | 差异 | 分支 |
|---|---|---|
| 差速 | 扫掠多边形 = 矩形，yaw = 切向 | 无 |
| 全向 | 允许横移 ⇒ 车头不必对齐切向 | 由 `yaw_source` 决定位姿口径 |
| 阿克曼 | 扫掠沿转向弧而非直线 | 由 `min_turning_radius` 决定采样曲线 |

本文件实现差速/全向的直线扫掠；阿克曼的弧线采样接口留了 `yaw_source`
参数位，S5 Step 6 才实现（S5 §6 R3：不能原地转的分支只允许在 Step 6 做）。

性能
----
* 单个位姿的成本 ≈ 多边形包围盒行数 × (边数 + 列数)，5 cm 栅格下
  0.5×0.47 m 的车大约 10×10 格 ⇒ 约 20 µs。
* 一条 5 m 路径按 5 cm 采样 = 100 个位姿 ⇒ 约 2 ms。
* 内核自带 `budget_ms` 硬预算：超了就置 `exceeded=True` 并提前返回，
  **不静默变慢**（S5 §3 M2 的约定）。
"""

from __future__ import annotations

import math
import time

try:
    import numpy as np
except ImportError:                                    # pragma: no cover
    np = None


class SweepResult:
    """一次扫掠的结果。"""

    __slots__ = ('clear', 'min_clearance', 'at_index', 'poses', 'exceeded',
                 'reason', 'at_pose')

    def __init__(self, clear, min_clearance, at_index, poses, exceeded,
                 reason='', at_pose=None):
        self.clear = clear
        self.min_clearance = min_clearance
        self.at_index = at_index
        self.poses = poses
        self.exceeded = exceeded
        self.reason = reason
        # ★ 失败位姿的完整 (x, y, yaw)。
        #   为什么必需：归因函数要把 footprint **旋转到实际 yaw** 才能判定
        #   "撞的是 /map 障碍还是视觉障碍"。只给 (x, y) 时只能按 yaw=0 摆一个
        #   轴对齐矩形，判到的格子可能压根不压在车体上 —— 归因会系统性失真。
        self.at_pose = at_pose

    def as_dict(self):
        return {'swept_clear': bool(self.clear),
                'swept_min_clearance': (None if self.min_clearance is None
                                        else round(float(self.min_clearance), 4)),
                'swept_at': self.at_index,
                'swept_at_pose': self.at_pose,
                'swept_poses': self.poses,
                'swept_exceeded': bool(self.exceeded),
                'swept_reason': self.reason}

    def __repr__(self):
        return ('<Sweep clear=%s min_clr=%s at=%s poses=%d%s>'
                % (self.clear, self.min_clearance, self.at_index, self.poses,
                   ' EXCEEDED' if self.exceeded else ''))


def _unit(v):
    n = math.hypot(v[0], v[1])
    return (v[0] / n, v[1] / n) if n > 0.0 else (0.0, 0.0)


def resample_path(path_xy, step_m, yaw_lookahead_m=None):
    """把路径重采样成等距点列，并给出每点的切向 yaw。

    `path_xy` 是 [(x, y), ...]（世界坐标，米）。返回 [(x, y, yaw), ...]。
    首点与末点一定保留（末点位姿的切向取最后一段方向）。

    ⚠️ **yaw 必须沿"前后弦"算，不能取逐段瞬时方向**
    ---------------------------------------------
    `path_and_width` 的 BFS 是 4 连通的，路径是**曼哈顿阶梯**：每段的
    方向只可能是 0°/90°/180°/270°。若按逐段方向给 yaw，车会在拐角处
    **瞬间转 90°**，而 0.5 m 长的车身在狭处一转就剮墙 ——
    这不是车过不去，是"假想了一个会瞬移拐弯的车"。

    2026-11-14 实测踩到：按逐段方向扫掠时 40 条记录里 25 条报
    obstacle_inside（62.5%），包括 `min_w=1.8 m` 的宽阔处 —— 一眼就假。

    所以改用 `yaw_lookahead_m`（默认半个车长量级）在当前点前后各取
    `lookahead/2` 的弦方向，等价于纯追踪控制器的实际车头方向。
    直线路径上这个平滑**不改变任何结果**（前向与后向弦同向），
    只在拐角处把"瞬转 90°"换成"逐渐转向"。
    """
    pts = [(float(x), float(y)) for x, y in path_xy]
    if len(pts) < 2:
        if not pts:
            return []
        return [(pts[0][0], pts[0][1], 0.0)]
    step = max(1e-3, float(step_m))
    out = []
    # 逐段插值，段内按 step 取点（不做全局重采样，避免累积误差）
    for k in range(len(pts) - 1):
        ax, ay = pts[k]
        bx, by = pts[k + 1]
        seg = math.hypot(bx - ax, by - ay)
        if seg <= 0.0:
            continue
        yaw = math.atan2(by - ay, bx - ax)
        n = max(1, int(math.ceil(seg / step)))
        for i in range(n):
            t = i / n
            out.append((ax + (bx - ax) * t, ay + (by - ay) * t, yaw))
    lx, ly = pts[-1]
    px, py = pts[-2]
    out.append((lx, ly, math.atan2(ly - py, lx - px)))
    if yaw_lookahead_m and yaw_lookahead_m > 0.0:
        out = smooth_yaw(out, yaw_lookahead_m)
    return out


def smooth_yaw(poses, lookahead_m):
    """把位姿序列的 yaw 换成"前后弦"方向（纯追踪口径）。

    对每个点取弧长前后各 `lookahead_m/2` 处的两点连线方向。
    首尾不足时向端点夹取。
    """
    n = len(poses)
    if n < 2 or lookahead_m <= 0.0:
        return poses
    half = lookahead_m * 0.5
    # 累积弧长
    cum = [0.0] * n
    for i in range(1, n):
        cum[i] = cum[i - 1] + math.hypot(poses[i][0] - poses[i - 1][0],
                                         poses[i][1] - poses[i - 1][1])
    def idx_at(s):
        # 线性查找足够快（n 通常 < 200，且只在影子模式用）
        lo, hi = 0, n - 1
        if s <= 0.0:
            return 0
        if s >= cum[-1]:
            return n - 1
        while lo < hi:
            mid = (lo + hi) // 2
            if cum[mid] < s:
                lo = mid + 1
            else:
                hi = mid
        return lo
    out = []
    for i in range(n):
        a = idx_at(cum[i] - half)
        b = idx_at(cum[i] + half)
        dx = poses[b][0] - poses[a][0]
        dy = poses[b][1] - poses[a][1]
        if abs(dx) < 1e-9 and abs(dy) < 1e-9:
            yaw = poses[i][2]
        else:
            yaw = math.atan2(dy, dx)
        out.append((poses[i][0], poses[i][1], yaw))
    return out


def transform_footprint(footprint, x, y, yaw):
    """把车身系多边形顶点变换到世界系。"""
    c, s = math.cos(yaw), math.sin(yaw)
    return [(x + vx * c - vy * s, y + vx * s + vy * c) for vx, vy in footprint]


def _row_spans(verts, yc):
    """水平线 y=yc 与多边形的交点对（crossing number，支持非凸）。

    返回 [(x0, x1), ...]，按 x 升序、两两成对。
    """
    xs = []
    n = len(verts)
    for i in range(n):
        ax, ay = verts[i]
        bx, by = verts[(i + 1) % n]
        # half-open 判据避免顶点重复计数
        if (ay <= yc < by) or (by <= yc < ay):
            t = (yc - ay) / (by - ay)
            xs.append(ax + t * (bx - ax))
    if len(xs) < 2:
        return ()
    xs.sort()
    return [(xs[i], xs[i + 1]) for i in range(0, len(xs) - 1, 2)]


def point_in_polygon(px, py, verts):
    """点是否在多边形内。

    ★ 刻意复用 `_row_spans()`（与 `sweep_poses()` 判碰撞时**同一套**扫描线），
      而不是另写一份射线法。理由：`_sweep_cause()` 要判"失败位姿下车体压到
      了哪些格"，如果它用的内点判据与内核不一致，归因结论和碰撞结论就会
      互相矛盾 —— 归因只用于落盘，但结论不一致会让人无法信任数据。
    """
    if not verts:
        return False
    for x0, x1 in _row_spans(verts, py):
        if x0 <= px <= x1:
            return True
    return False


def _boundary_samples(verts, step_m):
    """沿多边形边界按步长采样（用于量"多边形边界到最近障碍"的距离）。"""
    out = []
    n = len(verts)
    for i in range(n):
        ax, ay = verts[i]
        bx, by = verts[(i + 1) % n]
        seg = math.hypot(bx - ax, by - ay)
        k = max(1, int(math.ceil(seg / max(1e-3, step_m))))
        for j in range(k):
            t = j / k
            out.append((ax + (bx - ax) * t, ay + (by - ay) * t))
    return out


def sweep_poses(footprint, poses, field, budget_ms=None, unknown_blocks=False,
                sample_m=None):
    """对一串**位姿**做扫掠。

    参数
    ----
    footprint   : [(x, y), ...] 车身系多边形（来自 `RobotProfile.footprint`）
    poses       : [(x, y, yaw), ...] 世界系位姿
    field       : `(dist_m, ox, oy, res, w, h, unknown)` —— 与
                  `frontier_arbiter.clearance_field()` 返回的元组同构
    budget_ms   : 硬预算（毫秒）。None = 不限。超了置 `exceeded` 并提前返回。
    unknown_blocks : True 时把未知格也当障碍（与 `width_unknown_block` 对齐）

    返回 `SweepResult`。
    """
    if np is None:                                     # pragma: no cover
        return SweepResult(True, None, None, 0, False, 'no_numpy')
    if not footprint or not poses:
        return SweepResult(True, None, None, 0, False, 'empty_input')
    dist_m, ox, oy, res, w, h, unknown = field
    if res <= 0.0:
        return SweepResult(True, None, None, 0, False, 'bad_res')
    smp = res if sample_m is None else max(1e-3, float(sample_m))

    # 预计算：障碍格掩码（dist 精确为 0）
    occ = dist_m <= 0.0

    t0 = time.perf_counter()
    deadline = None if not budget_ms else t0 + budget_ms / 1000.0
    min_clr = None
    at_idx = None
    checked = 0

    for idx, (x, y, yaw) in enumerate(poses):
        if deadline is not None and (idx & 0x0F) == 0:
            if time.perf_counter() > deadline:
                return SweepResult(True, min_clr, at_idx, idx, True,
                                   'budget_exceeded')
        verts = transform_footprint(footprint, x, y, yaw)
        ys = [v[1] for v in verts]
        y_lo, y_hi = min(ys), max(ys)

        # 多边形覆盖的行范围（格心在 [y_lo, y_hi] 内）
        j0 = int(math.ceil((y_lo - oy) / res - 0.5))
        j1 = int(math.floor((y_hi - oy) / res - 0.5))
        j0 = max(0, j0)
        j1 = min(h - 1, j1)

        for j in range(j0, j1 + 1):
            yc = oy + (j + 0.5) * res
            spans = _row_spans(verts, yc)
            if not spans:
                continue
            for x0, x1 in spans:
                i0 = int(math.ceil((x0 - ox) / res - 0.5))
                i1 = int(math.floor((x1 - ox) / res - 0.5))
                i0 = max(0, i0)
                i1 = min(w - 1, i1)
                if i1 < i0:
                    continue
                checked += 1
                seg = dist_m[j, i0:i1 + 1]
                if unknown_blocks and unknown is not None:
                    if unknown[j, i0:i1 + 1].any():
                        return SweepResult(False, 0.0, (x, y), idx, False,
                                           'unknown_inside', (x, y, yaw))
                if occ[j, i0:i1 + 1].any():
                    return SweepResult(False, 0.0, (x, y), idx, False,
                                       'obstacle_inside', (x, y, yaw))

        # 边界到最近障碍的距离
        for bx, by in _boundary_samples(verts, smp):
            ci = int((bx - ox) / res)
            cj = int((by - oy) / res)
            if ci < 0 or cj < 0 or ci >= w or cj >= h:
                continue
            d = float(dist_m[cj, ci])
            if min_clr is None or d < min_clr:
                min_clr = d
                at_idx = (bx, by)

    return SweepResult(True, min_clr, at_idx, len(poses), False, 'ok')


def sweep_path(footprint, path_xy, field, step_m=None, budget_ms=None,
               unknown_blocks=False, yaw_lookahead_m=None):
    """对一条**路径**做扫掠（内部重采样 + 平滑后的切向 yaw）。

    `step_m` 默认取一格（`res`）：车长 0.5 m = 10 格，1 格步长已经足够密，
    比 `res/2` 少一半位姿、少一半耗时。要更保守可显式给 `res/2`。

    `yaw_lookahead_m` 默认 0.5 m（与车长同量级）：见 `resample_path` 里
    关于"曼哈顿阶梯不能按逐段瞬时方向给 yaw"的说明。
    """
    if field is None:
        return SweepResult(True, None, None, 0, False, 'no_field')
    res = field[3]
    step = res if step_m is None else max(1e-3, float(step_m))
    look = 0.5 if yaw_lookahead_m is None else float(yaw_lookahead_m)
    poses = resample_path(path_xy, step, yaw_lookahead_m=look)
    return sweep_poses(footprint, poses, field, budget_ms=budget_ms,
                       unknown_blocks=unknown_blocks, sample_m=res * 0.5)


# --------------------------------------------------------------------------
# 对照实现：轴向法向测宽（改造前的口径，仅用于测试里证明它高估 1.41 倍）
# --------------------------------------------------------------------------
def lateral_width_axis_aligned(blocked, res, ci, cj):
    """沿**轴对齐法向**测宽 —— 这就是 D2 缺陷的口径。

    只沿 x 和 y 两个方向各扫一次，取较小者。斜向通道会被高估 1/cos45 ≈ 1.41 倍。
    `blocked` 是 2D 布尔数组（True=障碍）。
    """
    h, w = blocked.shape

    def span(dx, dy):
        # 半宽：向两侧各走到第一个障碍
        pos = 0
        i, j = ci, cj
        while True:
            i += dx
            j += dy
            if not (0 <= i < w and 0 <= j < h) or blocked[j, i]:
                break
            pos += 1
        neg = 0
        i, j = ci, cj
        while True:
            i -= dx
            j -= dy
            if not (0 <= i < w and 0 <= j < h) or blocked[j, i]:
                break
            neg += 1
        return pos + neg + 1

    wx = span(1, 0)
    wy = span(0, 1)
    return min(wx, wy) * res
