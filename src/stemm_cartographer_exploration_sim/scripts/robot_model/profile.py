"""RobotProfile —— 车体画像，以及**只算不许配**的阈值派生量。

设计要点
--------
* 配置里只允许出现：外廓多边形、碰撞几何、安装高度、能力 flag、净空余量。
* 一切"阈值"（`pass_width_m` / `narrow_floor_m` / `turn_corridor_m`）都是派生量，
  在 `__init__` 里算出来并**只读**。配置里若出现同名键，构造时直接报错。
* 所有派生量都可以从 `as_dict()` 拿到，用于等价性闸门与日志。

两个容易混的半径（S5 §3 M1 遗留矛盾 #2，**两个数不同是对的**）
-------------------------------------------------------------
* `circumscribed_radius_m`  = 0.397 —— **规划口径**。外廓多边形顶点的最大半径。
  给 Nav2 膨胀/外接圆用。相当于"整块板子转起来扫过多大圆"。
* `collision_radius_m`      = 0.370 —— **取证口径**。由 collision_geometry
  （车身矩形 + 含轮半宽）反推，给 `collision_probe` 判"是不是真的碰到了"。

后者比前者小 2.7 cm，因为 Nav2 的 footprint 比实测碰撞盒前后各留了约 2 cm 余量。
把两者强行统一，要么让碰撞取证误报（用 0.397 会把"差 2 cm 没碰上"判成碰上），
要么让规划过度保守（用 0.370 会让膨胀圈套不住车角）。所以**故意分开**，
并在字段名上区分口径。

`inscribed_radius_m` 与 `min_origin_edge_m` 同理（见各自 docstring）。
"""

from __future__ import annotations

import glob
import math
import os

try:
    import yaml
except ImportError:                                    # pragma: no cover
    yaml = None


PROFILE_DIR_ENV = 'STEMM_ROBOT_PROFILE_DIR'

#: 允许的构型枚举。**不参与判据**，只用于校验配置是否写错。
KINEMATICS = ('differential', 'omni', 'ackermann', 'four_wheel_skid')


class ProfileError(ValueError):
    """profile 内容不合法。宁可启动即报错，也不要拿半套几何跑整个会话。"""


# --------------------------------------------------------------------------
# 几何工具
# --------------------------------------------------------------------------

def _dist_point_seg(p, a, b):
    """点到线段距离。"""
    ax, ay = a
    bx, by = b
    px, py = p
    dx, dy = bx - ax, by - ay
    den = dx * dx + dy * dy
    if den <= 0.0:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / den))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def _unit(v):
    n = math.hypot(v[0], v[1])
    return (v[0] / n, v[1] / n) if n > 0.0 else (0.0, 0.0)


def _rect_side_lengths(fp, tol=1e-6):
    """若是矩形（含旋转矩形）返回 (边a, 边b)，否则 None。

    判据：4 个顶点、相邻边两两垂直、对边等长。容忍 1e-6 的浮点误差
    （YAML 里写的是十进制小数，反解不保证逐位精确）。
    """
    if len(fp) != 4:
        return None
    a, b, c, d = fp
    ab = (b[0] - a[0], b[1] - a[1])
    bc = (c[0] - b[0], c[1] - b[1])
    cd = (d[0] - c[0], d[1] - c[1])
    da = (a[0] - d[0], a[1] - d[1])
    lab, lbc = math.hypot(*ab), math.hypot(*bc)
    lcd, lda = math.hypot(*cd), math.hypot(*da)
    if min(lab, lbc, lcd, lda) <= 0.0:
        return None
    if abs(lab - lcd) > tol or abs(lbc - lda) > tol:
        return None
    uab, ubc = _unit(ab), _unit(bc)
    if abs(uab[0] * ubc[0] + uab[1] * ubc[1]) > tol:
        return None
    return (lab, lbc)


def _r(v, nd=9):
    return round(float(v), nd)


# --------------------------------------------------------------------------
# 画像
# --------------------------------------------------------------------------

_DERIVED_KEYS = (
    'body_width_m', 'body_length_m',
    'inscribed_radius_m', 'min_origin_edge_m', 'circumscribed_radius_m',
    'collision_radius_m',
    'pass_width_m', 'narrow_floor_m', 'turn_corridor_m',
)


class RobotProfile:
    """车体画像。构造即校验；派生量只读。"""

    def __init__(self, data, source=None):
        if not isinstance(data, dict):
            raise ProfileError('profile 必须是映射（YAML dict），收到 %r'
                               % type(data).__name__)
        self.source = source
        self.raw = data

        # ---- 派生量不许出现在配置里（S5 §2 原则 1 的结构化保证） ----
        clash = [k for k in _DERIVED_KEYS if k in data]
        if clash:
            raise ProfileError(
                'profile 里出现派生量 %s —— 这些只能由几何算出，不许配'
                '（S5 §2 原则 1：阈值不得配置）。请从 %s 删除。'
                % (', '.join(clash), source or '<profile>'))

        # ---- 必填项 ----
        self.profile_name = self._get(data, 'profile_name')
        self.kinematics = self._get(data, 'kinematics')
        if self.kinematics not in KINEMATICS:
            raise ProfileError('kinematics=%r 不在允许集合 %s 内'
                               % (self.kinematics, list(KINEMATICS)))

        fp = self._get(data, 'footprint')
        if not isinstance(fp, (list, tuple)) or len(fp) < 3:
            raise ProfileError('footprint 至少要有 3 个顶点，收到 %r' % (fp,))
        try:
            self.footprint = [(float(x), float(y)) for x, y in fp]
        except Exception as exc:                       # noqa: BLE001
            raise ProfileError('footprint 顶点必须是 [x, y] 数字对：%s' % exc)

        cg = self._get(data, 'collision_geometry')
        for k in ('x_min', 'x_max', 'y_half'):
            if k not in cg:
                raise ProfileError('collision_geometry 缺少 %s' % k)
        self.collision_x_min = float(cg['x_min'])
        self.collision_x_max = float(cg['x_max'])
        self.collision_y_half = float(cg['y_half'])
        if self.collision_x_max <= self.collision_x_min:
            raise ProfileError('collision_geometry: x_max 必须 > x_min')
        if self.collision_y_half <= 0.0:
            raise ProfileError('collision_geometry: y_half 必须 > 0')

        caps = self._get(data, 'capabilities')
        self.can_rotate_in_place = bool(caps.get('in_place_rotation', False))
        self.min_turning_radius_m = float(caps.get('min_turning_radius_m', 0.0))
        self.reverse_allowed = bool(caps.get('reverse_allowed', False))
        self.max_reverse_step_m = float(caps.get('max_reverse_step_m', 0.0))
        self.can_move_lateral = bool(caps.get('lateral_motion', False))
        if not self.can_rotate_in_place and self.min_turning_radius_m <= 0.0:
            raise ProfileError(
                'in_place_rotation=false 时必须给出 min_turning_radius_m > 0，'
                '否则 turn_corridor_m 无意义')

        budget = self._get(data, 'clearance_budget')
        for k in ('side_open_m', 'side_tight_m', 'goal_disc_m'):
            if k not in budget:
                raise ProfileError('clearance_budget 缺少 %s' % k)
        self.side_open_m = float(budget['side_open_m'])
        self.side_tight_m = float(budget['side_tight_m'])
        self.goal_disc_m = float(budget['goal_disc_m'])
        if self.side_tight_m < 0.0 or self.side_open_m < 0.0:
            raise ProfileError('clearance_budget 的余量不能为负')
        if self.side_tight_m > self.side_open_m:
            raise ProfileError(
                'side_tight_m(%.3f) 不能大于 side_open_m(%.3f)：'
                '极限挤过必须比宽松行进更窄'
                % (self.side_tight_m, self.side_open_m))

        # 安装高度：不参与 Step 1 判据，但"哪些障碍 2D 雷达看不见"要靠它
        self.body_height_m = float(data.get('body_height_m', 0.0))
        self.scan_plane_height_m = float(data.get('scan_plane_height_m', 0.0))

        # ---- 派生量（只算，不许配） ----
        self._derive()

    # -- 内部工具 -----------------------------------------------------------

    @staticmethod
    def _get(data, key):
        if key not in data or data[key] is None:
            raise ProfileError('profile 缺少必填项 %s' % key)
        return data[key]

    def _derive(self):
        xs = [x for x, _ in self.footprint]
        ys = [y for _, y in self.footprint]

        self.body_width_m = max(ys) - min(ys)
        self.body_length_m = max(xs) - min(xs)

        # 外接圆：规划口径
        self.circumscribed_radius_m = max(
            math.hypot(x, y) for x, y in self.footprint)

        # Nav2 语义的 "inscribed_radius"：原点到最近外廓边的距离。
        # 本仓库 footprint 不关于原点对称（x∈[-0.18,0.32]），所以这个值
        # 是 min(0.18, 0.32, 0.235, 0.235) = 0.18，**比半宽小**。
        self.min_origin_edge_m = min(
            _dist_point_seg((0.0, 0.0), a, b)
            for a, b in zip(self.footprint,
                            self.footprint[1:] + self.footprint[:1]))

        # 几何内切圆：整块板子里能放下的最大圆。对矩形 = min(边长)/2。
        # 这是 S5 §3 M1 闸门要的 0.235（车体半宽），也是 Step 2/3 判
        # "这条走廊/这个区域装得下车吗"用的量。它与上面的 0.18 不是一回事。
        sides = _rect_side_lengths(self.footprint)
        if sides is not None:
            self.inscribed_radius_m = min(sides) / 2.0
            self.inscribed_radius_exact = True
        else:
            # 非矩形：取质心到各边最小距离，是真实内切半径的**保守下界**
            cx = sum(xs) / len(xs)
            cy = sum(ys) / len(ys)
            self.inscribed_radius_m = min(
                _dist_point_seg((cx, cy), a, b)
                for a, b in zip(self.footprint,
                                self.footprint[1:] + self.footprint[:1]))
            self.inscribed_radius_exact = False

        # 取证口径：由碰撞盒反推。与 circumscribed_radius_m 不同是故意的。
        self.collision_radius_m = math.hypot(
            max(abs(self.collision_x_min), abs(self.collision_x_max)),
            self.collision_y_half)

        # ---- 阈值派生（S5 §3 M1 表） ----
        #
        # 量化到 1 µm 是为了让等价性闸门做到**逐位相等**（S5 §5-A "0 容差"）：
        #   0.470 + 2×0.215 的浮点和是 0.89999999999999996669，
        #   最近的双精度是 0.9000000000000000222（比字面量 0.9 **高 1 ulp**）。
        # 不量化就会出现 `min_w < pass_width` 在 1e-16 量级上翻转的可能，
        # 虽然物理上无意义，但"0 容差"的门槛就不成立，历史上 S3 的 A/B 也就
        # 无法声称逐位可比。
        #
        # 1 µm 相比 5 cm 地图栅格低 50000 倍，远低于任何传感器/执行器分辨率，
        # 因此这一步只有收益、没有精度损失。
        _Q = 6                                            # 量化到 1e-6 m
        self.pass_width_m = round(
            self.body_width_m + 2.0 * self.side_open_m, _Q)
        self.narrow_floor_m = round(
            self.body_width_m + 2.0 * self.side_tight_m, _Q)

        # 掉头所需走廊宽度：能原地转就是 0（一维自由度，不需要走廊）。
        # 否则取"绕转弯圆心扫过的最大半径 × 2"，保守上界（S5 §3 M1）。
        # 直接对外廓顶点做扫描，等价于 S5 的闭式
        #   hypot(min_turning_radius + body_width/2, max(|x_min|,|x_max|))
        # 在横向对称车体上的结果；对非对称车体更强（闭式会低估一侧）。
        if self.can_rotate_in_place:
            self.turn_corridor_m = 0.0
            self.turn_outer_radius_m = 0.0
        else:
            r = self.min_turning_radius_m
            # 左转圆心在车体系 (0, +r)，右转在 (0, -r)；两种都要扫。
            self.turn_outer_radius_m = max(
                max(math.hypot(x, y - r) for x, y in self.footprint),
                max(math.hypot(x, y + r) for x, y in self.footprint))
            self.turn_corridor_m = 2.0 * self.turn_outer_radius_m

    # -- 只读别名（判据里用语义名，不用机型名） -----------------------------

    @property
    def can_pass(self):
        """占位：能力判定的语义入口，Step 2 起由扫掠内核使用。"""
        return True

    def as_dict(self):
        """全部派生量（等价性闸门与日志用）。"""
        return {
            'profile_name': self.profile_name,
            'kinematics': self.kinematics,
            'body_width_m': _r(self.body_width_m),
            'body_length_m': _r(self.body_length_m),
            'inscribed_radius_m': _r(self.inscribed_radius_m),
            'inscribed_radius_exact': self.inscribed_radius_exact,
            'min_origin_edge_m': _r(self.min_origin_edge_m),
            'circumscribed_radius_m': _r(self.circumscribed_radius_m),
            'collision_radius_m': _r(self.collision_radius_m),
            'pass_width_m': _r(self.pass_width_m),
            'narrow_floor_m': _r(self.narrow_floor_m),
            'turn_corridor_m': _r(self.turn_corridor_m),
            'turn_outer_radius_m': _r(self.turn_outer_radius_m),
            'side_open_m': _r(self.side_open_m),
            'side_tight_m': _r(self.side_tight_m),
            'goal_disc_m': _r(self.goal_disc_m),
            'can_rotate_in_place': self.can_rotate_in_place,
            'reverse_allowed': self.reverse_allowed,
            'can_move_lateral': self.can_move_lateral,
            'min_turning_radius_m': _r(self.min_turning_radius_m),
            'scan_plane_height_m': _r(self.scan_plane_height_m),
            'source': self.source,
        }

    def describe(self):
        return ('profile=%s(%s) 宽%.3f 长%.3f 内切%.3f 外接%.3f 碰撞%.3f '
                'pass=%.3f floor=%.3f turn=%.3f'
                % (self.profile_name, self.kinematics, self.body_width_m,
                   self.body_length_m, self.inscribed_radius_m,
                   self.circumscribed_radius_m, self.collision_radius_m,
                   self.pass_width_m, self.narrow_floor_m,
                   self.turn_corridor_m))

    def __repr__(self):
        return '<RobotProfile %s %s w=%.3f l=%.3f pass=%.3f floor=%.3f>' % (
            self.profile_name, self.kinematics, self.body_width_m,
            self.body_length_m, self.pass_width_m, self.narrow_floor_m)


# --------------------------------------------------------------------------
# 加载
# --------------------------------------------------------------------------

def profile_search_dirs():
    """profile 目录搜索顺序。

    运行期 `frontier_arbiter` 是 install 里的**符号链接**，`__file__` 指向
    install。所以先用 `realpath` 解析回 src，再退回 install/share 布局。
    """
    here = os.path.dirname(os.path.realpath(__file__))   # .../<pkg>/scripts/robot_model
    scripts = os.path.dirname(here)                      # .../<pkg>/scripts
    pkg = os.path.dirname(scripts)                       # .../<pkg>
    out = []
    env = os.environ.get(PROFILE_DIR_ENV)
    if env:
        out.append(env)
    out.append(os.path.join(pkg, 'config', 'robot_profiles'))
    # install 非 symlink 布局：<prefix>/lib/<pkg>/robot_model
    out.append(os.path.join(pkg, os.pardir, 'share',
                            'stemm_cartographer_exploration_sim',
                            'config', 'robot_profiles'))
    seen, uniq = set(), []
    for d in out:
        d = os.path.abspath(d)
        if d not in seen:
            seen.add(d)
            uniq.append(d)
    return uniq


def _resolve(name_or_path):
    """把 profile 名或路径解析成真实文件路径。"""
    if not name_or_path:
        raise ProfileError('profile 名不能为空')
    cand = name_or_path
    if os.path.isfile(cand):
        return os.path.abspath(cand)
    name = name_or_path
    if name.endswith('.yaml') or name.endswith('.yml'):
        name = os.path.splitext(name)[0]
    tried = []
    for d in profile_search_dirs():
        for ext in ('.yaml', '.yml'):
            p = os.path.join(d, name + ext)
            tried.append(p)
            if os.path.isfile(p):
                return p
    raise ProfileError('找不到 profile %r，已尝试：%s'
                       % (name_or_path, ', '.join(tried) or '<无搜索目录>'))


def available_profiles():
    """列出可加载的 profile 名（按搜索目录去重）。"""
    names = []
    for d in profile_search_dirs():
        for p in sorted(glob.glob(os.path.join(d, '*.yaml')) +
                        glob.glob(os.path.join(d, '*.yml'))):
            n = os.path.splitext(os.path.basename(p))[0]
            if n not in names:
                names.append(n)
    return names


def load_profile(name_or_path='wheeltec_diff'):
    """加载并校验 profile。失败一律抛 ProfileError（消息里带路径）。"""
    if yaml is None:                                   # pragma: no cover
        raise ProfileError('需要 PyYAML（ros2 环境自带）')
    path = _resolve(name_or_path)
    try:
        with open(path, encoding='utf-8') as f:
            data = yaml.safe_load(f)
    except OSError as exc:
        raise ProfileError('读取 profile 失败 %s：%s' % (path, exc))
    except yaml.YAMLError as exc:
        raise ProfileError('profile YAML 解析失败 %s：%s' % (path, exc))
    try:
        return RobotProfile(data, source=path)
    except ProfileError as exc:
        raise ProfileError('%s（%s）' % (exc, path))
