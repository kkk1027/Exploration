#!/usr/bin/env python3
"""param_audit_probe.py —— 运行期"参数是否真落地"自检（rclpy 直连，不依赖 ros2cli daemon）。

为什么重写（2026-10-26 事故）：
  原实现用 `ros2 param dump`。而本机在手机热点 + `ROS_LOCALHOST_ONLY=1` 下，
  ros2cli **daemon 发现不到任何节点**（`ros2 node list` 输出为空，但 rclpy 节点之间
  订阅/服务一切正常）。结果是 `dump` 永远空 → 自检恒 0/8 → 每轮被判
  invalid(param_audit)，纯假失败；而且每个节点重试 90 s，一轮白掉 4.5 分钟。
  现在直接用 rclpy 建节点、调目标节点的 `<node>/get_parameters` 服务取值。

用法（round.sh → harness/param_audit.sh → 本脚本）：
  python3 param_audit_probe.py --out <json 路径|空> --backstep 1 --rrt-wait 4 --front-stop 0
退出码：0=全部通过；1=有不通过项（会把本轮标 invalid，但继续跑）。
"""
import argparse
import json
import sys
import time

import rclpy
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters, ListParameters

CANDIDATE_NODES = ['/stemm_cartographer_nav2_manager',
                   '/local_costmap/local_costmap',
                   '/global_costmap/global_costmap']

# 需要取值的参数名（服务调用是"点名取"，不像 dump 能整树拉）
WANTED = ['use_sim_time', 'footprint', 'return_home_retreat_enabled',
          'return_home_orientation_grace_sec', 'rrt_fallback_wait_sec',
          'front_stop_distance', 'exploration_retreat_max_distance_m',
          'exploration_retreat_speed_mps', 'blocked_retreat_max_per_session',
          'exploration_retreat_enabled']


def to_str(v):
    t = v.type
    if t == ParameterType.PARAMETER_BOOL:
        return 'True' if v.bool_value else 'False'
    if t == ParameterType.PARAMETER_INTEGER:
        return str(v.integer_value)
    if t == ParameterType.PARAMETER_DOUBLE:
        return repr(v.double_value)
    if t == ParameterType.PARAMETER_STRING:
        return v.string_value
    if t == ParameterType.PARAMETER_BOOL_ARRAY:
        return '[' + ', '.join('True' if b else 'False' for b in v.bool_array_value) + ']'
    if t == ParameterType.PARAMETER_INTEGER_ARRAY:
        return '[' + ', '.join(str(i) for i in v.integer_array_value) + ']'
    if t == ParameterType.PARAMETER_DOUBLE_ARRAY:
        return '[' + ', '.join(repr(d) for d in v.double_array_value) + ']'
    if t == ParameterType.PARAMETER_STRING_ARRAY:
        return '[' + ', '.join(v.string_array_value) + ']'
    if t == ParameterType.PARAMETER_BYTE_ARRAY:
        return '[' + ', '.join(str(b) for b in v.byte_array_value) + ']'
    return ''


def call(node, cli, req, timeout=4.0):
    fut = cli.call_async(req)
    t0 = time.time()
    while not fut.done() and time.time() - t0 < timeout:
        rclpy.spin_once(node, timeout_sec=0.05)
    if not fut.done():
        return None
    return fut.result()


def fetch(node, name, names, timeout=4.0):
    """先 list_parameters 拿到真正声明过的名字，再按名字取值。

    2026-10-26 实测坑：直接 `get_parameters` 请求未声明的名字时，回应里
    可能**整个空掉**（实测 10 个名字里混着未声明项 → values 为空 → 判成"节点连不上"，
    而单独请求已声明项立刻返回 22.0）。所以必须先用 list_parameters 求交集。
    """
    lcli = node.create_client(ListParameters, name + '/list_parameters')
    if not lcli.wait_for_service(timeout_sec=timeout):
        return None
    lreq = ListParameters.Request()
    lres = call(node, lcli, lreq, timeout)
    if lres is None:
        return None
    declared = set(lres.result.names or [])
    want = [n for n in names if n in declared]
    if not want:
        return {}
    cli = node.create_client(GetParameters, name + '/get_parameters')
    if not cli.wait_for_service(timeout_sec=timeout):
        return None
    req = GetParameters.Request()
    req.names = want
    res = call(node, cli, req, timeout)
    if res is None:
        return None
    out = {}
    for nm, val in zip(want, res.values):
        if val.type == ParameterType.PARAMETER_NOT_SET:
            continue
        out[nm] = to_str(val)
    out['_declared'] = len(declared)
    return out


def norm(s):
    return str(s).replace(' ', '').lower()


def matches(got, want):
    g, w = norm(got), norm(want)
    try:
        return abs(float(g) - float(w)) <= 1e-6
    except (TypeError, ValueError):
        return w in g


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default='')
    ap.add_argument('--backstep', default='1')
    ap.add_argument('--rrt-wait', default='0')
    ap.add_argument('--front-stop', default='0')
    a = ap.parse_args()

    rclpy.init()
    node = rclpy.create_node('stemm_param_audit_probe')
    per_node = {}
    t_start = time.time()
    try:
        # 服务可能还没起：整体最多等 15 s 把它们都摸一遍
        deadline = t_start + 15.0
        pending = list(CANDIDATE_NODES)
        while pending and time.time() < deadline:
            for nm in list(pending):
                got = fetch(node, nm, WANTED, timeout=1.5)
                if got is not None:
                    got.pop('_declared', None)
                    per_node[nm] = got
                    pending.remove(nm)
            if pending:
                time.sleep(0.5)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    seen = list(per_node)

    def pick(param):
        for nm in CANDIDATE_NODES:
            v = per_node.get(nm, {}).get(param)
            if v is not None:
                return v
        return None

    # ---- 硬门槛：这两项错了整轮数据就没意义（都真出过事故）----
    # use_sim_time：mapping_tf_guard 曾因没有 sim time 判"TF 不健康"把整栈拆掉；
    # footprint  ：车体轮廓没打补丁 → 通行判据/膨胀全都按原厂算。
    items, bad = [], 0

    def record(param, want, got, hard=True):
        nonlocal bad
        if got is None:
            if hard:
                bad += 1
                print("  [audit] ✗ %s 取不到值（节点未声明或服务不通）" % param)
            items.append({'param': param, 'ok': (not hard), 'hard': hard, 'want': want,
                          'got': None if got is None else str(got)})
            return
        ok = matches(got, want)
        if not ok and hard:
            bad += 1
            print("  [audit] ✗ %s 期望含 '%s'，实际 '%s'" % (param, want, got))
        elif not ok:
            print("  [audit] (info) %s 实际 '%s'（期望 '%s'，非硬门槛）" % (param, got, want))
        items.append({'param': param, 'ok': ok, 'hard': hard, 'want': want, 'got': str(got)})

    # use_sim_time：能摸到的节点必须全是 True
    sims = {nm: per_node[nm].get('use_sim_time') for nm in seen}
    bad_sim = {nm: v for nm, v in sims.items() if v is not None and norm(v) != 'true'}
    if not any(v is not None for v in sims.values()):
        bad += 1
        print('  [audit] ✗ 没有任何节点报 use_sim_time')
        items.append({'param': 'use_sim_time', 'ok': False, 'hard': True, 'got': None})
    else:
        ok = not bad_sim
        if not ok:
            bad += 1
            print('  [audit] ✗ use_sim_time 非 True 的节点：%s' % bad_sim)
        items.append({'param': 'use_sim_time', 'ok': ok, 'hard': True,
                      'got': json.dumps(sims, ensure_ascii=False)})

    fp = pick('footprint')
    want_fp = '[[-0.18,-0.235]'
    record('footprint', want_fp, fp, hard=True)
    # ---- 非硬门槛：车端参数，注入了就要对得上；没声明只提示 ----
    record('return_home_retreat_enabled', 'False', pick('return_home_retreat_enabled'), hard=False)
    record('return_home_orientation_grace_sec', '22.0',
           pick('return_home_orientation_grace_sec'), hard=False)
    record('rrt_fallback_wait_sec', a.rrt_wait, pick('rrt_fallback_wait_sec'), hard=False)
    if a.front_stop not in ('0', '0.0', ''):
        record('front_stop_distance', a.front_stop, pick('front_stop_distance'), hard=False)
    if a.backstep == '1':
        record('exploration_retreat_max_distance_m', '0.15',
               pick('exploration_retreat_max_distance_m'), hard=False)
        record('exploration_retreat_speed_mps', '0.05',
               pick('exploration_retreat_speed_mps'), hard=False)
        record('blocked_retreat_max_per_session', '2',
               pick('blocked_retreat_max_per_session'), hard=False)
        info = pick('exploration_retreat_enabled')
        if info is not None:
            print('  [audit] (info) exploration_retreat_enabled=%s —— 由四向闸动态掌管' % info)
    else:
        record('exploration_retreat_enabled', 'False', pick('exploration_retreat_enabled'),
               hard=False)
    if not seen:
        print('  [audit] ✗ 三个目标节点一个都没连上（服务未就绪或 DDS 不通）')

    total = len(items)
    print('  [audit] 参数落地自检：%d/%d 通过（硬门槛失败 %d 项；backstep=%s，节点=%d，耗时 %.1fs）'
          % (total - bad, total, bad, a.backstep, len(seen), time.time() - t_start))
    if a.out and a.out != '/dev/null':
        try:
            with open(a.out, 'w', encoding='utf-8') as fh:
                json.dump({'check': 'param_audit', 'ok': bad == 0, 'pass': total - bad,
                           'total': total, 'nodes': seen, 'items': items},
                          fh, ensure_ascii=False)
        except OSError as exc:
            print('  [audit] ✗ 写 JSON 失败：%s' % exc)
    return 0 if bad == 0 else 1


if __name__ == '__main__':
    sys.exit(main())
