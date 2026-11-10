#!/usr/bin/env python3
"""test_verdict_propagation.py —— P13-6「判决传递」的离线验证。

背景（p13d A/B 实测，B 组 5 轮均值）：
    raw_seen        165.6 次/轮   （仲裁器看到的 RRT 原始目标）
    raw_deadend     189.8 次/轮   （其中被判"无逃生方向"）
    raw_publish       1.6 次/轮   （判决后真正放行）
    merge_raw        35~93 条/轮  （frontier_merge 实际转发给 manager）
⇒ 98.6% 的 raw 被判死胡同/窄缝/不可达，**判决却从未离开仲裁器**；
  merge 的 raw 兜底等于"绕过否决再送进 nav2"。

本测试锁住五件事：
  1) 两个模块的"负判决"集合必须完全一致（跨模块契约，不一致会静默失效）；
  2) parse_verdict 对坏消息容错，绝不抛异常；
  3) lookup_verdict 的半径/时效/取最新三条语义；
  4) _verdict_gate 的四条分支（关闭/负判决/正判决/无判决）；
  5) 饿死保护 force_after_sec 能在负判决全满时放行；
  6) 仲裁器侧 _record_verdict 只在判决翻转时发布（不刷屏）+ 量化网格一致。

用法：python3 harness/test_verdict_propagation.py [-v]
"""
import importlib.machinery
import importlib.util
import json
import math
import os
import sys
import types

_pkgs = ('rclpy', 'rclpy.node', 'rclpy.qos', 'action_msgs', 'action_msgs.msg',
         'geometry_msgs', 'geometry_msgs.msg', 'nav_msgs', 'nav_msgs.msg',
         'std_msgs', 'std_msgs.msg', 'wheeltec_rrt_msg', 'wheeltec_rrt_msg.msg')
for name in _pkgs:
    if name not in sys.modules:
        mod = types.ModuleType(name)
        mod.__getattr__ = lambda attr: object  # type: ignore
        sys.modules[name] = mod
sys.modules['rclpy'].node = sys.modules['rclpy.node']
sys.modules['std_msgs'].msg = sys.modules['std_msgs.msg']
sys.modules['geometry_msgs'].msg = sys.modules['geometry_msgs.msg']
sys.modules['nav_msgs'].msg = sys.modules['nav_msgs.msg']
sys.modules['action_msgs'].msg = sys.modules['action_msgs.msg']
sys.modules['wheeltec_rrt_msg'].msg = sys.modules['wheeltec_rrt_msg.msg']


class _Node:
    def __init__(self, *a, **k):
        pass


sys.modules['rclpy.node'].Node = _Node
sys.modules['rclpy.qos'].QoSProfile = lambda *a, **k: None
sys.modules['rclpy.qos'].DurabilityPolicy = type('D', (), {'TRANSIENT_LOCAL': 1})
sys.modules['rclpy.qos'].ReliabilityPolicy = type('R', (), {'RELIABLE': 1})
sys.modules['wheeltec_rrt_msg.msg'].PointArray = type('P', (), {})
sys.modules['action_msgs.msg'].GoalStatusArray = type('G', (), {})
sys.modules['geometry_msgs.msg'].PoseStamped = type('PS', (), {})
sys.modules['geometry_msgs.msg'].Twist = type('T', (), {})
sys.modules['nav_msgs.msg'].OccupancyGrid = type('O', (), {})
sys.modules['nav_msgs.msg'].Odometry = type('Od', (), {})
sys.modules['std_msgs.msg'].String = type('S', (), {'__init__': lambda s: None})
sys.modules['std_msgs.msg'].Bool = type('B', (), {})

HERE = os.path.dirname(os.path.abspath(__file__))
SCRIPTS = os.path.join(HERE, '..', 'src', 'stemm_cartographer_exploration_sim',
                       'scripts')


def _load(mod_name, filename):
    path = os.path.join(SCRIPTS, filename)
    loader = importlib.machinery.SourceFileLoader(mod_name, path)
    spec = importlib.util.spec_from_loader(mod_name, loader)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[mod_name] = mod
    loader.exec_module(mod)
    return mod


arb = _load('frontier_arbiter', 'frontier_arbiter')
mrg = _load('frontier_merge', 'frontier_merge')

FAILS = []
VERBOSE = '-v' in sys.argv


def check(name, cond, detail=''):
    if cond:
        if VERBOSE:
            print('  ok   %s  %s' % (name, detail))
        return True
    FAILS.append('%s  %s' % (name, detail))
    print('  FAIL %s  %s' % (name, detail))
    return False


# ---------------------------------------------------------------- 门控桩
class Stub:
    """只提供 _verdict_gate 需要的字段，用来跑真实的 FrontierMerge 方法。"""

    def __init__(self, enabled=True, force_after=15.0, last_out=0.0):
        self.verdict_enabled = enabled
        self.force_after = force_after
        self.verdict_r = 0.45
        self.verdict_ttl = 6.0
        self.verdicts = {}
        self._last_out = last_out
        self.stats = {'merge_verdict_block': 0, 'merge_verdict_ok': 0,
                      'merge_verdict_none': 0, 'merge_force_pass': 0}


def gate(stub, px, py, now):
    return mrg.FrontierMerge._verdict_gate(stub, px, py, now)


def put(stub, x, y, v, t, score=0.0):
    k = (int(math.floor(x / 0.30)), int(math.floor(y / 0.30)))
    stub.verdicts[k] = (t, v, score, x, y)


def main():
    # ============================================================
    print('\n[1] 跨模块契约：负判决集合必须完全一致')
    a_set = set(arb.Arbiter.VERDICT_NEGATIVE)
    m_set = set(mrg.NEGATIVE_VERDICTS)
    check('仲裁器与 merge 的负判决集合相等', a_set == m_set,
          'arb=%s merge=%s' % (sorted(a_set), sorted(m_set)))
    check('feasible 不属于负判决', 'feasible' not in m_set)
    check('四类负判决齐全',
          m_set == {'dead_end', 'unreachable', 'too_narrow', 'over_dist'},
          repr(sorted(m_set)))

    # ============================================================
    print('\n[2] parse_verdict 容错')
    good = json.dumps({'k': [3, -2], 'x': 1.05, 'y': -0.45, 'v': 'dead_end',
                       's': 2.0})
    got = mrg.parse_verdict(good)
    check('正常消息可解析', got is not None and got[0] == (3, -2)
          and got[3] == 'dead_end', repr(got))
    check('坐标原样保留', got is not None and abs(got[1] - 1.05) < 1e-9
          and abs(got[2] + 0.45) < 1e-9, repr(got[1:3] if got else None))
    check('坏 JSON 返回 None', mrg.parse_verdict('{not json') is None)
    check('缺 v 字段返回 None',
          mrg.parse_verdict(json.dumps({'x': 0, 'y': 0})) is None)
    check('v 为空串返回 None',
          mrg.parse_verdict(json.dumps({'v': '', 'x': 0, 'y': 0})) is None)
    # 无 k 时退回自己量化
    nok = mrg.parse_verdict(json.dumps({'x': 0.95, 'y': -0.31, 'v': 'too_narrow'}))
    check('无 k 字段时按 0.30 m 网格自行量化',
          nok is not None and nok[0] == (3, -2), repr(nok[0] if nok else None))
    check('x/y 非数字且无 k 时返回 None',
          mrg.parse_verdict(json.dumps({'v': 'dead_end', 'x': 'a', 'y': 'b'}))
          is None)
    check('s 非数字时降级为 0 而不是失败',
          mrg.parse_verdict(json.dumps({'x': 0, 'y': 0, 'v': 'dead_end',
                                        's': 'x'}))[4] == 0.0)

    # ============================================================
    print('\n[3] lookup_verdict 的半径 / 时效 / 取最新')
    now = 100.0
    tbl = {}
    check('空表返回 (None, None)', mrg.lookup_verdict(tbl, 0, 0, 0.45, 6.0, now)
          == (None, None))
    tbl = {(-9, -9): (now - 1.0, 'dead_end', 0.0, 5.00, 5.00)}
    check('半径内命中', mrg.lookup_verdict(tbl, 5.20, 5.10, 0.45, 6.0, now)[0]
          == 'dead_end')
    check('半径外不命中', mrg.lookup_verdict(tbl, 5.90, 5.00, 0.45, 6.0, now)[0]
          is None)
    tbl2 = {(-9, -9): (now - 7.0, 'dead_end', 0.0, 5.00, 5.00)}
    check('过期判决不命中', mrg.lookup_verdict(tbl2, 5.00, 5.00, 0.45, 6.0, now)[0]
          is None)
    tbl3 = {(-9, -9): (now - 3.0, 'dead_end', 0.0, 5.00, 5.00),
            (-9, -8): (now - 0.5, 'feasible', 0.0, 5.20, 5.10)}
    v, age = mrg.lookup_verdict(tbl3, 5.10, 5.05, 0.45, 6.0, now)
    check('多条命中时取最新（判决可翻转）', v == 'feasible',
          'v=%s age=%.2f' % (v, age if age is not None else -1))
    check('返回的 age 是最新那条的', age is not None and abs(age - 0.5) < 1e-6,
          'age=%s' % age)

    # ============================================================
    print('\n[4] _verdict_gate 的四条分支')
    s = Stub(enabled=False)
    put(s, 2.0, 2.0, 'dead_end', 99.5)
    check('verdict_enabled=False 时一律放行', gate(s, 2.0, 2.0, 100.0) is False)

    s = Stub(enabled=True, force_after=15.0, last_out=99.0)
    put(s, 2.0, 2.0, 'dead_end', 99.5)
    check('新鲜负判决 → 拦截', gate(s, 2.0, 2.0, 100.0) is True)
    check('block 计数 +1', s.stats['merge_verdict_block'] == 1)

    s = Stub(enabled=True, last_out=99.0)
    put(s, 2.0, 2.0, 'feasible', 99.5)
    check('正判决 → 放行', gate(s, 2.0, 2.0, 100.0) is False)
    check('ok 计数 +1', s.stats['merge_verdict_ok'] == 1)

    s = Stub(enabled=True, last_out=99.0)
    check('无任何判决 → 放行（不误杀）', gate(s, 2.0, 2.0, 100.0) is False)
    check('none 计数 +1', s.stats['merge_verdict_none'] == 1)

    s = Stub(enabled=True, last_out=99.0)
    put(s, 2.0, 2.0, 'unreachable', 99.5)
    check('unreachable 同样拦截', gate(s, 2.0, 2.0, 100.0) is True)
    s2 = Stub(enabled=True, last_out=99.0)
    put(s2, 2.0, 2.0, 'too_narrow', 99.5)
    check('too_narrow 同样拦截', gate(s2, 2.0, 2.0, 100.0) is True)
    s3 = Stub(enabled=True, last_out=99.0)
    put(s3, 2.0, 2.0, 'over_dist', 99.5)
    check('over_dist 同样拦截', gate(s3, 2.0, 2.0, 100.0) is True)

    # ============================================================
    print('\n[5] 饿死保护 force_after_sec')
    s = Stub(enabled=True, force_after=15.0, last_out=80.0)   # 20 s 没输出
    put(s, 2.0, 2.0, 'dead_end', 99.5)
    check('超过 force_after 后无视负判决放行', gate(s, 2.0, 2.0, 100.0) is False)
    check('force 计数 +1', s.stats['merge_force_pass'] == 1)

    s = Stub(enabled=True, force_after=15.0, last_out=90.0)   # 10 s 没输出
    put(s, 2.0, 2.0, 'dead_end', 99.5)
    check('未超过 force_after 时仍然拦截', gate(s, 2.0, 2.0, 100.0) is True)

    # 边界：正好等于阈值应放行（>= 判定，避免浮点抖动导致的永久拦截）
    s = Stub(enabled=True, force_after=15.0, last_out=85.0)
    put(s, 2.0, 2.0, 'dead_end', 99.5)
    check('恰等于 force_after 时放行（>= 而非 >）',
          gate(s, 2.0, 2.0, 100.0) is False)

    # 饿死保护必须能真正"救活"：连续 N 条全被拦后应放行一条
    s = Stub(enabled=True, force_after=15.0, last_out=100.0)
    put(s, 2.0, 2.0, 'dead_end', 99.5)
    blocked = 0
    for i in range(40):                      # 模拟 40 条 raw，时间推进 0.5 s/条
        t = 100.0 + i * 0.5
        if gate(s, 2.0, 2.0, t):
            blocked += 1
        else:
            s._last_out = t                  # 放行后刷新
    check('全负判决下不会永久阻塞（40 条里最终有放行）', blocked < 40,
          'blocked=%d/40' % blocked)

    # ============================================================
    print('\n[6] 仲裁器侧：量化网格一致 + 只在翻转时发布')

    class PubStub(arb.Arbiter):
        """继承真实 Arbiter，但跳过 __init__（避免拉起 rclpy）。
        这样 _record_verdict / _verdict_key / publish_verdict_snapshot
        跑的都是线上那一份代码，不是复制品。"""

        def __init__(self):
            self.sent = []
            self.verdict_enabled = True
            self.verdict_period = 2.0
            self.verdict_ttl = 6.0
            self.verdicts = {}
            self._t_verdict = 0.0
            self.stats = {}
            self.verdict_pub = self

        def publish(self, m):
            self.sent.append(m.data)

    ps = PubStub()
    # 直接调用真实方法（未绑定 self 的实例字段由 PubStub 提供）
    arb.Arbiter._record_verdict(ps, 1.05, -0.45, 'dead_end', 2.0)
    check('首次判决会发布', len(ps.sent) == 1, 'n=%d' % len(ps.sent))
    arb.Arbiter._record_verdict(ps, 1.08, -0.44, 'dead_end', 2.0)
    check('同格同判决不重复发布', len(ps.sent) == 1, 'n=%d' % len(ps.sent))
    arb.Arbiter._record_verdict(ps, 1.08, -0.44, 'feasible', 90.0)
    check('判决翻转时立即发布', len(ps.sent) == 2, 'n=%d' % len(ps.sent))

    key = arb.Arbiter._verdict_key(ps, 1.05, -0.45)
    # merge 侧对同一坐标的量化必须一致
    mkey = (int(math.floor(1.05 / 0.30)), int(math.floor(-0.45 / 0.30)))
    check('arbiter 与 merge 的 0.30 m 量化结果一致', key == mkey,
          'arb=%s merge=%s' % (key, mkey))

    # 发布内容必须能被 merge 解析（端到端契约）
    parsed = mrg.parse_verdict(ps.sent[0])
    check('arbiter 发出的消息能被 merge 正确解析',
          parsed is not None and parsed[3] == 'dead_end', repr(parsed))
    check('解析出的键与发送端一致', parsed is not None and parsed[0] == key,
          'sent_key=%s parsed=%s' % (key, parsed[0] if parsed else None))

    # ============================================================
    print('\n[7] 快照重发（丢包/后启动自愈）')
    ps2 = PubStub()
    arb.Arbiter._record_verdict(ps2, 1.0, 1.0, 'dead_end', 1.0)
    n0 = len(ps2.sent)
    ps2._t_verdict = 0.0                  # 强制认为已到期
    arb.Arbiter.publish_verdict_snapshot(ps2)
    check('周期快照会重发存活判决', len(ps2.sent) > n0,
          '%d → %d' % (n0, len(ps2.sent)))
    ps2.verdicts[(-99, -99)] = (1.0, 'dead_end', 0.0)   # 极旧（时间戳 1.0）
    ps2._t_verdict = 0.0
    before = len(ps2.sent)
    arb.Arbiter.publish_verdict_snapshot(ps2)
    check('快照不会重发已过期判决', len(ps2.sent) - before <= 1,
          '新增 %d 条' % (len(ps2.sent) - before))

    # ============================================================
    print('\n[8] 各层默认值一致（否则单跑与 A/B 跑批行为不同）')
    here = os.path.dirname(os.path.abspath(__file__))
    launch_src = open(os.path.join(here, '..', 'src',
                                   'stemm_cartographer_exploration_sim',
                                   'launch', 'sim_explore.launch.py'),
                      encoding='utf-8').read()
    round_src = open(os.path.join(here, 'round.sh'), encoding='utf-8').read()
    arb_src = open(os.path.join(SCRIPTS, 'frontier_arbiter'), encoding='utf-8').read()
    mrg_src = open(os.path.join(SCRIPTS, 'frontier_merge'), encoding='utf-8').read()
    check('launch 声明 verdict 默认 true',
          "DeclareLaunchArgument('verdict', default_value='true'" in launch_src)
    check('round.sh 的 VERDICT 默认与 launch 一致（=1 → true）',
          '"${VERDICT:-1}"' in round_src, '应为 ${VERDICT:-1}')
    check('arbiter 参数默认键名为 verdict_enabled/verdict_ttl_sec',
          "'verdict_enabled'" in arb_src and "'verdict_ttl_sec'" in arb_src)
    check('merge 参数默认键名为 verdict_ttl/verdict_radius/force_after_sec',
          "'verdict_ttl', 6.0" in mrg_src and "'verdict_radius', 0.45" in mrg_src
          and "'force_after_sec', 15.0" in mrg_src)
    check('launch 的 verdict_ttl 默认与两端一致（6.0）',
          "DeclareLaunchArgument('verdict_ttl', default_value='6.0'" in launch_src)
    check('launch 的 verdict_radius 默认与 merge 一致（0.45）',
          "DeclareLaunchArgument('verdict_radius', default_value='0.45'"
          in launch_src)
    check('launch 的 verdict_force_after 默认与 merge 一致（15.0）',
          "DeclareLaunchArgument('verdict_force_after', default_value='15.0'"
          in launch_src)
    # 判决必须打在原始坐标上（回拉点会让 merge 永远匹配不上）
    pick = arb_src.split('def pick_raw_goal')[1].split('def ')[0]
    check('pick_raw_goal 把判决打在原始坐标 (rx,ry) 而非回拉点 (gx,gy)',
          '_record_verdict(rx, ry,' in pick,
          'found=%d' % pick.count('_record_verdict(rx, ry,'))
    check('pick_raw_goal 的五个判决分支都有打点',
          all(("'%s'" % v) in pick for v in mrg.NEGATIVE_VERDICTS)
          and "'feasible'" in pick,
          repr([v for v in mrg.NEGATIVE_VERDICTS + ('feasible',)
                if ("'%s'" % v) not in pick]) or 'all present')

    # ============================================================
    print('\n' + '=' * 60)
    if FAILS:
        print('FAIL —— %d 项未通过：' % len(FAILS))
        for f in FAILS:
            print('  - %s' % f)
        return 1
    print('全部通过（P13-6 判决传递不变量成立）')
    return 0


if __name__ == '__main__':
    sys.exit(main())
