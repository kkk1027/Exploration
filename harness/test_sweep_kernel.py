#!/usr/bin/env python3
"""test_sweep_kernel.py —— S5 M2 扫掠内核的合成栅格断言（< 2 s）。

为什么用合成栅格而不是真实地图
------------------------------
真实地图上没有"真值"：没人知道某条走廊到底多宽。合成栅格可以**构造已知真值**，
才能判断内核是对是错。真实地图留给影子模式的观测数据去评估。

四个合成用例（S5 §4 Step 2 验收 ①）
-----------------------------------
    A 直道      宽度 1.2 m，车宽 0.47 → 必须能过
    B 斜道      45° 走廊，真实宽 0.62 m → 必须能过
    C 窄口      真实宽 0.40 m（< 车宽 0.47）→ 必须过不去
    D L 形      两段直道 + 一个直角 → 车必须能整体转过去

验收 ②（斜道不再高估 1.41 倍）
-----------------------------
构造 45° 走廊，真实净宽依次取 0.40 ~ 0.70 m：

* **轴向法向测宽**（改造前口径）在 45° 下读数 = 真实宽 × 1/cos45 ≈ ×1.414。
  所以真实 0.40 m 会被读成 ≈ 0.57 m —— 看起来"能过"（> 0.47），实际过不去。
* **扫掠**直接回答"放不放得下"，0.40 m 必须判过不去。

这条测试就是"斜道高估"从缺陷变成可执行断言的证据。

用法
----
    python3 harness/test_sweep_kernel.py
    python3 harness/test_sweep_kernel.py -v
"""
import argparse
import math
import os
import sys

BASE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SIM_PKG = os.path.join(BASE, 'src', 'stemm_cartographer_exploration_sim')
SCRIPTS = os.path.join(SIM_PKG, 'scripts')
sys.path.insert(0, SCRIPTS)

from robot_model import load_profile                       # noqa: E402
from sweep_kernel import (sweep_path, sweep_poses,          # noqa: E402
                          lateral_width_axis_aligned)

_ok = []
_bad = []

RES = 0.05


def chk(cond, label, detail=''):
    (_ok if cond else _bad).append((label, detail))
    return cond


# --------------------------------------------------------------------------
# 合成栅格工具
# --------------------------------------------------------------------------
class Grid:
    """一张合成占据栅格 + 对应的距离场（与 clearance_field 同构）。"""

    def __init__(self, w, h, res=RES, ox=0.0, oy=0.0):
        try:
            import numpy as np
        except ImportError:                            # pragma: no cover
            raise SystemExit('需要 numpy')
        self.np = np
        self.w, self.h, self.res, self.ox, self.oy = w, h, res, ox, oy
        self.blocked = np.zeros((h, w), dtype=bool)

    # -- 世界坐标 ↔ 栅格 ------------------------------------------------
    def cell(self, x, y):
        return (int((x - self.ox) / self.res), int((y - self.oy) / self.res))

    # -- 构造 ------------------------------------------------------------
    def fill_all(self):
        self.blocked[:, :] = True

    def carve_rect(self, x0, y0, x1, y1):
        """把世界坐标矩形 [x0,x1]×[y0,y1] 切成自由。"""
        i0 = int(math.ceil((x0 - self.ox) / self.res))
        i1 = int(math.floor((x1 - self.ox) / self.res))
        j0 = int(math.ceil((y0 - self.oy) / self.res))
        j1 = int(math.floor((y1 - self.oy) / self.res))
        self.blocked[max(0, j0):max(0, j1 + 1),
                     max(0, i0):max(0, i1 + 1)] = False

    def carve_band(self, x0, y0, x1, y1, half_width):
        """沿线段 (x0,y0)-(x1,y1) 挖一条**半宽 half_width** 的自由通道。

        按格心到线段的垂直距离判据，而不是按整数格刷帧 ——
        后者会把 0.40 m 的通道挖成 0.45 m（`2*floor(hw/res)+1` 格），
        让"窄口应该过不去"的用例失效。
        """
        L = math.hypot(x1 - x0, y1 - y0)
        if L <= 0:
            return
        dx, dy = (x1 - x0) / L, (y1 - y0) / L
        pad = half_width + self.res * 2
        i0 = max(0, int((min(x0, x1) - pad - self.ox) / self.res))
        i1 = min(self.w - 1, int((max(x0, x1) + pad - self.ox) / self.res))
        j0 = max(0, int((min(y0, y1) - pad - self.oy) / self.res))
        j1 = min(self.h - 1, int((max(y0, y1) + pad - self.oy) / self.res))
        for j in range(j0, j1 + 1):
            py = self.oy + (j + 0.5) * self.res
            for i in range(i0, i1 + 1):
                px = self.ox + (i + 0.5) * self.res
                t = (px - x0) * dx + (py - y0) * dy
                t = max(0.0, min(L, t))
                cx, cy = x0 + dx * t, y0 + dy * t
                if math.hypot(px - cx, py - cy) <= half_width:
                    self.blocked[j, i] = False

    # -- 距离场 ----------------------------------------------------------
    def field(self):
        from scipy import ndimage
        np = self.np
        dist_cells = ndimage.distance_transform_edt(~self.blocked)
        dist_m = dist_cells * self.res
        unknown = np.zeros((self.h, self.w), dtype=bool)
        return (dist_m, self.ox, self.oy, self.res, self.w, self.h, unknown)


def body_footprint(prof):
    """profile 的 footprint 顶点点列（车身系，米）。"""
    return [(float(x), float(y)) for x, y in prof.footprint]


def straight_path(x0, y0, x1, y1, n=40):
    return [(x0 + (x1 - x0) * k / n, y0 + (y1 - y0) * k / n)
            for k in range(n + 1)]


# --------------------------------------------------------------------------
# A/B/C/D + 验收 ②
# --------------------------------------------------------------------------
def test_A_straight(prof, verbose):
    """A 直道：宽 1.2 m，车宽 0.47 → 必须能过。"""
    g = Grid(80, 60)                                # 4.0 m × 3.0 m
    g.fill_all()
    g.carve_rect(0.00, 0.90, 4.00, 2.10)            # 1.2 m 宽的横向直道（贯穿全宽）
    f = g.field()
    # 路径端点必须留出**车体后半长** 0.18 m，否则车尾会顶在挖槽边界上
    r = sweep_path(body_footprint(prof), straight_path(0.40, 1.50, 3.60, 1.50), f)
    chk(r.clear and not r.exceeded,
        'A 直道 1.20 m（车宽 0.47）→ 能过',
        'clear=%s min_clr=%s %s' % (r.clear, r.min_clearance, r.reason))
    chk(r.min_clearance is not None and r.min_clearance > 0.30,
        'A 直道净空 ≈ (1.20−0.47)/2 = 0.365 m',
        '测得 %s' % r.min_clearance)


def test_B_diagonal(prof, verbose):
    """B 斜道：45° 走廊，真实净宽 0.62 m → 必须能过。"""
    g = Grid(140, 140)                              # 7.0 m × 7.0 m
    g.fill_all()
    g.carve_band(0.60, 0.60, 6.20, 6.20, 0.31)     # 45°, 宽 0.62 m
    f = g.field()
    # 路径两端各留 ~0.8 m 余量，避开斜道端帽处的采样
    path = [(1.60, 1.60), (5.20, 5.20)]
    r = sweep_path(body_footprint(prof), path, f)
    chk(r.clear and not r.exceeded,
        'B 斜道 45° 宽 0.62 m（车宽 0.47）→ 能过',
        'clear=%s min_clr=%s %s' % (r.clear, r.min_clearance, r.reason))


def test_C_narrow(prof, verbose):
    """C 窄口：真实宽 0.40 m < 车宽 0.47 → 必须过不去。"""
    g = Grid(80, 60)
    g.fill_all()
    g.carve_rect(0.00, 1.30, 4.00, 1.70)            # 0.40 m 宽
    f = g.field()
    r = sweep_path(body_footprint(prof), straight_path(0.40, 1.50, 3.60, 1.50), f)
    chk(not r.clear,
        'C 窄口 0.40 m < 车宽 0.47 → 过不去',
        'clear=%s %s' % (r.clear, r.reason))


def test_D_L_shape(prof, verbose):
    """D L 形：两段直道 + 直角转弯 → 车必须能整体过去。"""
    g = Grid(120, 120)                              # 6.0 m × 6.0 m
    g.fill_all()
    g.carve_rect(0.20, 0.80, 4.20, 2.20)            # 横向段（1.4 m 宽）
    g.carve_rect(2.70, 0.80, 4.10, 4.40)            # 纵向段（1.4 m 宽）
    f = g.field()
    path = [(0.70, 1.50), (3.40, 1.50), (3.40, 3.90)]
    r = sweep_path(body_footprint(prof), path, f)
    chk(r.clear and not r.exceeded,
        'D L 形（直角转弯）→ 能整体过去',
        'clear=%s min_clr=%s %s' % (r.clear, r.min_clearance, r.reason))


def test_E_diagonal_no_overestimate(prof, verbose):
    """验收 ②：斜道测量不再高估 1.41 倍。

    轴向法向测宽在 45° 下读数 = 真实宽 × 1.414。构造真实宽 0.40 m 的斜道：
      轴向读数 ≈ 0.57 m > 车宽 0.47 ⇒ **误判"能过"**
      扫掠 ⇒ 正确地判"过不去"

    ⚠️ 合成栅格的分辨率限制（必须如实登记）
    --------------------------------------
    `carve_band` 按**格心**判据挖槽，实际自由宽会被量化到约 ±1 格（5 cm）。
    所以标称 0.44 m 的槽实际自由宽约 0.50 m，车（0.47）在里面确实装得下 ——
    扫掠判 `clear=True` 是**对的**，不是内核的错。

    因此本用例对 `|true_w − 车宽| < 0.03 m` 的刀锋值**跳过可达性断言**
    （只断言高估倍数）。判断"内核能否分辨 5 cm 以内的差异"需要真实地图，
    那不是合成栅格能回答的问题，留给影子模式的实测数据。
    """
    bw = prof.body_width_m
    # 避开与车宽相差不足 1 格（5 cm）的刀锋值
    for true_w in (0.36, 0.40, 0.44, 0.56, 0.62, 0.70):
        g = Grid(160, 160)                          # 8.0 m × 8.0 m
        g.fill_all()
        g.carve_band(0.80, 0.80, 7.20, 7.20, true_w / 2.0)
        f = g.field()
        ci, cj = g.cell(4.0, 4.0)
        w_axis = lateral_width_axis_aligned(g.blocked, g.res, ci, cj)
        r = sweep_path(body_footprint(prof), [(2.20, 2.20), (5.80, 5.80)], f)
        ratio = w_axis / true_w
        if verbose:
            print('      真实 %.2f m → 轴向读数 %.3f m (×%.2f)  扫掠 clear=%s'
                  % (true_w, w_axis, ratio, r.clear))
        if true_w < bw - 0.03:
            chk(w_axis > bw,
                'E 真实 %.2f m：轴向读数 %.3f m **误判能过**（> 车宽 %.2f）'
                % (true_w, w_axis, bw))
            chk(not r.clear,
                'E 真实 %.2f m：扫掠正确地判过不去' % true_w,
                'clear=%s' % r.clear)
        elif true_w > bw + 0.03:
            chk(r.clear,
                'E 真实 %.2f m（> 车宽 %.2f）：扫掠判能过' % (true_w, bw),
                'clear=%s %s' % (r.clear, r.reason))
        # 高估倍数落在 1.15~1.55 之间（理论 1.414，离散化会略偏低）
        chk(1.15 <= ratio <= 1.55,
            'E 真实 %.2f m：轴向高估 ×%.2f（理论 1.414）' % (true_w, ratio))


def test_F_budget(prof):
    """预算硬约束：budget_ms 必须真的截断并置 exceeded（不静默变慢）。"""
    g = Grid(200, 200)
    g.fill_all()
    g.carve_band(1.0, 1.0, 8.0, 8.0, 1.0)
    f = g.field()
    long_path = [(1.0 + 7.0 * k / 400, 1.0 + 7.0 * k / 400) for k in range(401)]
    r = sweep_path(body_footprint(prof), long_path, f, budget_ms=0.001)
    chk(r.exceeded or r.poses < 100,
        'F 极小预算 → 置 exceeded 并提前返回（不静默变慢）',
        'exceeded=%s poses=%d reason=%s' % (r.exceeded, r.poses, r.reason))
    r2 = sweep_path(body_footprint(prof), long_path, f, budget_ms=5000.0)
    chk(not r2.exceeded,
        'F 充足预算 → exceeded=False（预算不是永久截断）',
        'poses=%d reason=%s' % (r2.poses, r2.reason))


def test_G_resample():
    """重采样：等距、保留首末点、切向正确。"""
    from sweep_kernel import resample_path
    path = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)]
    poses = resample_path(path, 0.25)
    chk(len(poses) > 8, 'G 重采样点数合理', '%d' % len(poses))
    chk(abs(poses[0][0]) < 1e-9 and abs(poses[0][1]) < 1e-9,
        'G 首点保留', str(poses[0][:2]))
    chk(abs(poses[-1][0] - 1.0) < 1e-9 and abs(poses[-1][1] - 1.0) < 1e-9,
        'G 末点保留', str(poses[-1][:2]))
    # 第一段沿 +x ⇒ yaw=0；第二段沿 +y ⇒ yaw=pi/2
    chk(abs(poses[0][2]) < 1e-6, 'G 首段切向 yaw=0', '%.4f' % poses[0][2])
    chk(abs(poses[-1][2] - math.pi / 2) < 1e-6,
        'G 末段切向 yaw=π/2', '%.4f' % poses[-1][2])


def test_H_capability_flags(prof):
    """构型无关：footprint 从 profile 取，代码里没有机型名分支。"""
    import inspect
    from sweep_kernel import sweep as sk
    src = inspect.getsource(sk).lower()
    for word in ('ackermann', 'mecanum', 'omni', 'differential'):
        # 只允许出现在注释/docstring 里；这里做的是"没有 if 机型名 分支"的粗筛
        chk(('if %s' % word) not in src and ('== %s' % word) not in src,
            'H 内核里没有 %r 的机型分支' % word)
    chk(prof.body_width_m > 0,
        'H footprint 来自 profile（宽 %.3f）' % prof.body_width_m)


def test_I_arbiter_binding():
    """集成断言：影子扫掠必须核对"这条路径属于哪个目标"。

    为什么这条必须存在
    ------------------
    `_last_path_w` 在同一拍里会被 `max_candidates`(=14) 次 `path_and_width`
    **反复覆盖**。若 `probe_candidate` 直接读它而不核对 `_last_path_goal`，
    就会拿**别的候选的路径**去做扫掠。

    2026-11-14 实测踩到：不核对时 220 条记录里 186 条报
    `obstacle_inside`（84.5%），而对应 `min_w` 高达 1.8 m —— 一眼就假。
    这类"接错线"的 bug 不会抛异常，只会污染整批观测数据。

    本测试用源码级断言把它钉死（不做运行期模拟，因为要构造整个 Node）。
    """
    path = os.path.join(SIM_PKG, 'scripts', 'frontier_arbiter')
    src = open(path, encoding='utf-8').read()
    chk('_last_path_goal' in src,
        'I frontier_arbiter 记录了 _last_path_goal（路径与目标的配对标记）')
    # 找到 _sweep_shadow_probe 的调用点，检查同一语句块里引用了 _last_path_goal
    idx = src.find('_sweep_shadow_probe(self._last_path_w')
    chk(idx > 0, 'I 影子扫掠调用 _sweep_shadow_probe')
    if idx > 0:
        window = src[max(0, idx - 1200):idx]
        chk('_last_path_goal' in window,
            'I 调用影子扫掠前核对了 _last_path_goal',
            '调用点前 1200 字符内未见配对检查')
        chk('same' in window or 'pg' in window,
            'I 存在"路径属于本候选"的判定变量')
    # 超预算必须记 None（未知），不能沿用内核的安全默认 clear=True
    chk("out['swept_clear'] = None" in src or 'out["swept_clear"] = None' in src,
        'I 扫掠超预算时 swept_clear 记为 None（未知），不是 True')
    # 影子开关默认必须是关闭（S5 §2 原则 4）
    chk("p('sweep_shadow', False)" in src,
        'I sweep_shadow 参数默认 False（默认关闭，逐位等价）')


def test_K_sweep_quota():
    """集成断言：影子扫掠的每拍配额必须**真的生效**。

    为什么这条必须存在
    ------------------
    第一版把 `sweep_top_n` 声明了、也写进了启动日志，**却从未参与任何限制**。
    `sweep_budget_ms` 则直接透传给 `sweep_path()`，是"单次调用"上限。
    两者叠加 = 每拍最坏 `14 × 30 = 420 ms`，而 S5 §4 Step 2 的验收是
    "单拍 P95 增量 ≤ 10 ms"。6 轮 A/B 实测 P95 增量 +50~65 ms，正是这个。

    这类 bug 的特征：参数看着存在、日志看着正常、单元测试全绿，
    只有拿真实 tick 耗时去比才暴露。所以本测试直接断言源码里
    **存在使用该参数的比较**，而不是只断言参数被声明。
    """
    path = os.path.join(SIM_PKG, 'scripts', 'frontier_arbiter')
    src = open(path, encoding='utf-8').read()

    # 1) sweep_top_n 必须出现在一个比较里（不只是被赋值/打印）
    chk(('self._sweep_tick_n >= self.sweep_top_n' in src)
        or ('self.sweep_top_n <= self._sweep_tick_n' in src),
        'K sweep_top_n 参与了每拍次数限制（不是只声明不生效）',
        '未找到 >= sweep_top_n 的比较；参数可能只是装饰')

    # 2) 预算必须是"每拍共享"，必须存在按拍重置
    chk('_sweep_ms_left' in src,
        'K 存在每拍共享的毫秒预算池 _sweep_ms_left')
    chk('_sweep_ms_left = self.sweep_budget_ms' in src,
        'K 预算池按拍重置回 sweep_budget_ms',
        '每拍必须重新充满，否则整轮只够扫一次')
    chk('_sweep_ms_left -= ' in src,
        'K 每次扫掠后从共享池扣掉真实耗时（含超预算那次）')

    # 3) 重置必须绑定拍号，而不是"每 N 次调用"之类
    chk('_sweep_tick_id' in src,
        'K 预算池用拍号 _sweep_tick_id 判定拍边界')

    # 4) 传给内核的预算必须是**剩余额度**，不能是原始全量
    idx = src.find('res = sweep_path(fp, path_w, field,')
    chk(idx > 0, 'K 找到 sweep_path 调用点')
    if idx > 0:
        seg = src[idx:idx + 400]
        chk('budget_ms=self._sweep_ms_left' in seg,
            'K 传给内核的是剩余额度，不是每拍全量',
            '若传 self.sweep_budget_ms，每拍总开销仍可无界')

    # 5) 配额拒绝要计数，否则无法从日志验证是否生效
    for k in ('sweep_skipped_quota', 'sweep_calls', 'sweep_ms_left_prev'):
        chk(k in src, 'K 计数器 %s 已落盘到 timeline' % k,
            '没有观测项就无法事后验证配额')


def test_L_cause_attribution():
    """集成断言：碰撞归因必须用**真实 yaw**，不能轴对齐替换。

    为什么这条必须存在
    ------------------
    第一版 `_sweep_cause()` 里写着 `yaw = 0.0`（注释还自我安慰"只影响归因
    不影响判据"）。实际上归因时用轴对齐矩形去查图，查到的格子可能压根不在
    旋转后的车体上。

    2026-11-14 实测后果：`p14vP` 三批 58 条有效扫掠里 33 条与轴向口径分歧，
    **33/33 全报 `vision_or_other`**，而对应 `min_w` 高达 0.90~1.15 m。
    100% 单一归因 + 反直觉的宽度，是"归因本身坏了"的典型特征，
    而不是"视觉层真的在误报"。

    另外，内点判据必须与扫掠本体**同一套**（`point_in_polygon` 复用
    `_row_spans`），否则归因结论与碰撞结论会自相矛盾。
    """
    path = os.path.join(SIM_PKG, 'scripts', 'frontier_arbiter')
    src = open(path, encoding='utf-8').read()
    idx = src.find('def _sweep_cause')
    chk(idx > 0, 'L 找到 _sweep_cause')
    if idx > 0:
        seg = src[idx:idx + 4000]
        chk('yaw = 0.0' not in seg,
            'L 归因没有把 yaw 写死成 0.0',
            '轴对齐归因：查到的格子可能不在车体上')
        chk('at_pose' in seg,
            'L 归因从 res.at_pose 取真实 yaw')
        chk('point_in_polygon' in seg,
            'L 归因用内核同一套内点判据（point_in_polygon）')
        # 三类来源必须分开报，不然看不出该改哪里
        for tag in ('map_obstacle', 'vision_obstacle', 'unknown_blocked'):
            chk(tag in seg, 'L 归因能区分 %s' % tag)

    # 内核必须把 yaw 一起带回来
    sk = os.path.join(SIM_PKG, 'scripts', 'sweep_kernel', 'sweep.py')
    ssrc = open(sk, encoding='utf-8').read()
    chk('at_pose' in ssrc, 'L SweepResult 携带失败位姿 at_pose（含 yaw）')
    chk('(x, y, yaw))' in ssrc,
        'L 碰撞返回处把 (x, y, yaw) 一起塞进结果')

    # point_in_polygon 必须与扫描线同源，不是另写一份射线法
    pi = ssrc.find('def point_in_polygon')
    chk(pi > 0, 'L 存在 point_in_polygon')
    if pi > 0:
        chk('_row_spans' in ssrc[pi:pi + 900],
            'L point_in_polygon 复用 _row_spans（与碰撞判据同源）')


def test_J_staircase(yaw_smoothing=True):
    """曼哈顿阶梯路径：不平滑会假报碰撞，平滑后不报。

    BFS 是 4 连通的，真实路径是阶梯。按"逐段瞬时方向"给 yaw 时，车会在每个
    拐角**瞬间转 90°**（0.5 m 车身一转就剐墙），实测让 62.5% 的记录假报
    `obstacle_inside`，包括 `min_w=1.8 m` 的宽阔处。

    本用例在一条**笔直 0.75 m 宽的走廊**里构造一条阶梯路径（锯齿幅度
    ±1 格）。车宽 0.47 ⇒ 两侧各 0.14 m，真值是"肯定能过"，
    所以 `clear=False` 就是内核的假报。

    实测（离线，合成 0.75/0.90/1.05/1.20 m 四档）：
      0.75 m → 不平滑 False（假报）/ 平滑 True（正确）
      0.90 m 及以上 → 两种都 True（余量够，掩盖问题）
    所以用例必须用 0.75 m，用 1.2 m 复现不出来。
    """
    from sweep_kernel import resample_path, sweep_poses
    g = Grid(80, 60)
    g.fill_all()
    # 0.75 m 宽：车 0.47 能过（两侧各 0.14），但**容不下瞬间转 90°**
    g.carve_rect(0.00, 1.125, 4.00, 1.875)
    f = g.field()
    # 阶梯路径：沿 +x 前进，y 在 ±1 格之间锯齿（模拟曼哈顿 BFS 的产物）
    stair = []
    for k in range(60):
        x = 0.40 + k * 0.05
        stair.append((x, 1.50 + (0.05 if k % 2 else -0.05)))
    fp = body_footprint(load_profile('wheeltec_diff'))

    raw = resample_path(stair, 0.05, yaw_lookahead_m=0.0)   # 不平滑
    r_raw = sweep_poses(fp, raw, f)
    sm = resample_path(stair, 0.05, yaw_lookahead_m=0.5)    # 平滑
    r_sm = sweep_poses(fp, sm, f)
    chk(r_sm.clear,
        'J 阶梯路径（平滑后）在 0.75 m 窄道里判能过',
        'clear=%s reason=%s' % (r_sm.clear, r_sm.reason))
    chk(not r_raw.clear,
        'J 阶梯路径（不平滑）会假报碰撞 —— 证明平滑是必需的',
        'clear=%s' % r_raw.clear)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('-v', '--verbose', action='store_true')
    a = ap.parse_args()
    try:
        prof = load_profile('wheeltec_diff')
    except Exception as exc:                           # noqa: BLE001
        print('✗ 无法加载 profile：%r' % (exc,))
        return 2
    chk(abs(prof.body_width_m - 0.470) < 1e-9,
        'profile 可用（车宽 0.470，非 0.400）', '%.4f' % prof.body_width_m)

    for fn in (test_A_straight, test_B_diagonal, test_C_narrow,
               test_D_L_shape, test_E_diagonal_no_overestimate):
        try:
            fn(prof, a.verbose)
        except Exception as exc:                       # noqa: BLE001
            chk(False, '%s 抛异常' % fn.__name__, repr(exc))
    for fn in (test_F_budget,):
        try:
            fn(prof)
        except Exception as exc:                       # noqa: BLE001
            chk(False, '%s 抛异常' % fn.__name__, repr(exc))
    for fn in (test_G_resample,):
        try:
            fn()
        except Exception as exc:                       # noqa: BLE001
            chk(False, '%s 抛异常' % fn.__name__, repr(exc))
    try:
        test_H_capability_flags(prof)
    except Exception as exc:                           # noqa: BLE001
        chk(False, 'test_H_capability_flags 抛异常', repr(exc))
    try:
        test_I_arbiter_binding()
    except Exception as exc:                           # noqa: BLE001
        chk(False, 'test_I_arbiter_binding 抛异常', repr(exc))
    try:
        test_J_staircase()
    except Exception as exc:                           # noqa: BLE001
        chk(False, 'test_J_staircase 抛异常', repr(exc))
    try:
        test_K_sweep_quota()
    except Exception as exc:                           # noqa: BLE001
        chk(False, 'test_K_sweep_quota 抛异常', repr(exc))
    try:
        test_L_cause_attribution()
    except Exception as exc:                           # noqa: BLE001
        chk(False, 'test_L_cause_attribution 抛异常', repr(exc))

    if a.verbose:
        for label, detail in _ok:
            print('  ✓ %s  %s' % (label, detail))
    for label, detail in _bad:
        print('  ✗ %s  %s' % (label, detail))
    print('test_sweep_kernel: %d 通过, %d 失败' % (len(_ok), len(_bad)))
    return 0 if not _bad else 1


if __name__ == '__main__':
    sys.exit(main())
