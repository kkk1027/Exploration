#!/usr/bin/env python3
"""analyze_dispatch_risk.py —— 派发质量归因（只读，不启停任何 ROS/Gazebo 进程）。

为什么单独一套
--------------
现有两套指标都回答不了当前最大的问题：

  * `candidate_metrics.py` 看"这一轮跑得合不合格"（PASS/接触/覆盖）；
  * `analyze_candidates.py` 看"落点在地图上的复杂度"。

它们都不回答：**派出去的目标，为什么会被取消 / 卡死？**

而实测（10 轮 306 次派发）显示这才是瓶颈：中位派发距离 5.30 m（81% >4 m、
31% >6 m），中位转角 135°（71% >90°、40% >150°）。本脚本把"派发决策本身"量化：

  1) 派发账本：每条 `navigation goal dispatched` 按目标坐标配对
     `reached / canceled / failed`，得到真实的取消率；
  2) 到达误差：`reached` 行里的 `error=(xy=…)` 分布；
  3) **幻影成功（phantom success）**：`reached` 但"派发→终态"耗时短到物理不可能
     （隐含速度 > PHANTOM_SPEED，车最快 ≈0.5 m/s）。实测 v2 组 83~98% 的 reached
     属于这一类：`controller_server` 在 30 ms 内对 5.456 m 的目标报
     `Reached the goal!`。成因是 `goal_checker.stateful: true` 锁存成功后不再复位
     （详见 docs/P13_基线.md）。这一项与候选评分完全无关，但会直接造成长时静止；
  4) 分桶取消率：按**派发时就能拿到的物理量**（距离 / 转角 / esc_dirs /
     comp_r2 / min_w）分桶，看哪一类候选更容易被打断；
  5) 评分覆盖率：有多少派发真的经过仲裁器评分（probe 配对 + merge.jsonl 计数）。

  ★ 分桶刻意使用**物理特征**而不是新的风险分：这样改造前后的数字可直接对比。

特征来源（两级，脚本会标明用了哪一级）
  * probe  —— candidate_probe.jsonl（**派发时刻**的地图，正确口径）
  * final  —— 本轮结束时的 map.pgm（回退用，已知面积更大，esc/comp 偏乐观）
  * none   —— 连地图都没有，只参与"距离/转角"分组

用法
----
  python3 harness/analyze_dispatch_risk.py logs/round_*            # 全量
  python3 harness/analyze_dispatch_risk.py logs/round_xxx -v      # 单轮明细
  python3 harness/analyze_dispatch_risk.py logs/round_* --json /tmp/x.json
"""
import argparse
import glob
import json
import math
import os
import re
import statistics as st
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)

TS = r'\[(\d{9,}\.\d+)\]'
RE_DISP = re.compile(
    TS + r'.*navigation goal dispatched: source=(\w+), '
         r'target=\(([-\d.]+), ([-\d.]+)\), '
         r'robot=\(([-\d.]+), ([-\d.]+), yaw=([-\d.]+)deg\), '
         r'dist=([\d.]+)m, turn=(-?\d+)deg')
RE_REACH = re.compile(
    TS + r'.*navigation goal reached; goal=\(([-\d.]+), ([-\d.]+), yaw=[-\d.]+\), '
         r'current=\(([-\d.]+), ([-\d.]+), yaw=[-\d.]+\), '
         r'error=\(xy=([\d.]+)m')
RE_TERM = re.compile(
    TS + r'.*navigation goal (canceled|failed|aborted); '
         r'goal=\(([-\d.]+), ([-\d.]+)')

GOAL_TOL = 0.06          # 目标坐标匹配容差（日志两位小数）
PAIR_WINDOW = 90.0       # 派发 → 终态 的最大配对时窗（秒）
DEDUPE_SEC = 3.0         # 同一目标在该秒数内重复派发只算一次
FALSE_REACH_M = 0.50     # reached 但 error 超过此值 → 记"带误差到达"
# 幻影成功：派发→reached 的隐含速度超过这个值就物理上不可能（车最快 ~0.5 m/s）
PHANTOM_SPEED = 1.50     # m/s
PHANTOM_MIN_DIST = 1.00  # m，低于这个距离的快速到达可能只是原地微调，不判幻影
PROBE_TOL = 0.80         # m，probe 配对容差（含 manager 的"投影到已知自由格"位移）
PROBE_WINDOW = 3.0       # s，probe 配对时窗


def _round_name(d):
    return os.path.basename(d.rstrip('/'))


def _jl(path):
    out = []
    if not os.path.exists(path):
        return out
    for line in open(path, encoding='utf-8', errors='replace'):
        line = line.strip()
        if not line.startswith('{'):
            continue
        try:
            out.append(json.loads(line))
        except Exception:                              # noqa: BLE001
            pass
    return out


def parse_ledger(round_dir):
    """返回 (dispatches, reach_errors, counters)。"""
    lp = os.path.join(round_dir, 'launch.log')
    if not os.path.exists(lp):
        return [], [], {}
    disp, term = [], []
    for line in open(lp, encoding='utf-8', errors='replace'):
        m = RE_DISP.search(line)
        if m:
            disp.append({
                't': float(m.group(1)), 'src': m.group(2),
                'gx': float(m.group(3)), 'gy': float(m.group(4)),
                'rx': float(m.group(5)), 'ry': float(m.group(6)),
                'dist': float(m.group(8)), 'turn': abs(float(m.group(9))),
            })
            continue
        m = RE_REACH.search(line)
        if m:
            term.append({'t': float(m.group(1)), 'kind': 'reached',
                         'gx': float(m.group(2)), 'gy': float(m.group(3)),
                         'cx': float(m.group(4)), 'cy': float(m.group(5)),
                         'err': float(m.group(6))})
            continue
        m = RE_TERM.search(line)
        if m:
            kind = 'canceled' if m.group(2) == 'canceled' else 'failed'
            term.append({'t': float(m.group(1)), 'kind': kind,
                         'gx': float(m.group(3)), 'gy': float(m.group(4)),
                         'err': None})
    term.sort(key=lambda e: e['t'])

    # 同一目标在 DEDUPE_SEC 内重复派发 → 只留第一条（避免把一次派发数成多次）
    kept = []
    for d in disp:
        if kept:
            p = kept[-1]
            if (d['t'] - p['t'] <= DEDUPE_SEC
                    and math.hypot(d['gx'] - p['gx'], d['gy'] - p['gy']) <= GOAL_TOL):
                continue
        kept.append(d)
    disp = kept

    counters = {'disp': len(disp), 'reached': 0, 'canceled': 0,
                'failed': 0, 'unknown': 0, 'false_reach': 0,
                'phantom': 0, 'real_reach': 0}
    reach_errors = []
    for d in disp:
        hit = None
        for e in term:
            if e['t'] < d['t']:
                continue
            if e['t'] - d['t'] > PAIR_WINDOW:
                break
            if (abs(e['gx'] - d['gx']) <= GOAL_TOL
                    and abs(e['gy'] - d['gy']) <= GOAL_TOL):
                hit = e
                break
        if hit is None:
            counters['unknown'] += 1
            d['label'] = 'unknown'
            continue
        d['label'] = hit['kind']
        counters[hit['kind']] += 1
        d['dt'] = round(hit['t'] - d['t'], 3)
        if hit['kind'] != 'reached':
            continue
        reach_errors.append(hit['err'])
        if hit['err'] > FALSE_REACH_M:
            counters['false_reach'] += 1
            d['false_reach'] = True
        # 幻影成功：距离够远但耗时短到物理不可能
        dt = max(1e-3, hit['t'] - d['t'])
        d['implied_speed'] = round(d['dist'] / dt, 2)
        if d['dist'] >= PHANTOM_MIN_DIST and d['implied_speed'] > PHANTOM_SPEED:
            counters['phantom'] += 1
            d['phantom'] = True
        else:
            counters['real_reach'] += 1
    return disp, reach_errors, counters


def attach_features(round_dir, disp):
    """给派发补上"派发时刻"的物理特征。

    优先用 candidate_probe.jsonl（正确口径）；缺失的记录再用本轮最终地图
    （map.pgm，借助 analyze_candidates 的距离场）补齐，并标记 feature_source。
    返回 (n_probe_join, n_final_join, n_none)。
    """
    probes = _jl(os.path.join(round_dir, 'candidate_probe.jsonl'))
    keys = ('esc_dirs', 'comp_r2', 'min_w', 'occ_frac_r2',
            'dist_to_robot', 'turn_deg', 'unk_2m', 'free_frac_2m',
            'risk_total', 'risk_pre', 'risk_route')
    # 坐标容差刻意放宽到 0.80 m：manager 在派发前会把候选"投影"到最近的
    # 已知自由格（日志 `projected RRT boundary sample from A to known-free B`），
    # 实测位移 0.3~0.5 m。用 0.12 m 会把真正经过评分的派发全部漏掉
    # （首版就是 0% 的配对率，把"评分覆盖率"误读成 0）。
    n_probe = 0
    for d in disp:
        best, best_key = None, None
        for p in probes:
            g = p.get('goal')
            if not g or len(g) < 2:
                continue
            dd = math.hypot(g[0] - d['gx'], g[1] - d['gy'])
            if dd > PROBE_TOL:
                continue
            dt = abs(float(p.get('t', 0.0)) - d['t'])
            if dt > PROBE_WINDOW:
                continue
            key = (dt, dd)          # 时间优先、距离次之
            if best_key is None or key < best_key:
                best, best_key = p, key
        if best is not None:
            d['feature_source'] = 'probe'
            d['probe_dt'] = round(best_key[0], 2)
            d['probe_dxy'] = round(best_key[1], 3)
            for k in keys:
                if best.get(k) is not None:
                    d[k] = best[k]
            n_probe += 1
        else:
            d['feature_source'] = 'none'

    n_final = 0
    missing = [d for d in disp if d['feature_source'] == 'none']
    if missing:
        try:
            import analyze_candidates as ac
            field = ac.build_field(round_dir)
        except Exception:                              # noqa: BLE001
            field = None
        if field is not None:
            for d in missing:
                try:
                    f = ac.features(field, d['gx'], d['gy'])
                except Exception:                      # noqa: BLE001
                    f = None
                if f:
                    d.update(f)
                    d['feature_source'] = 'final'
                    n_final += 1
    n_none = sum(1 for d in disp if d['feature_source'] == 'none')
    return n_probe, n_final, n_none


def _pct(a, b):
    return (100.0 * a / b) if b else 0.0


def _stat(vals):
    if not vals:
        return None, None, None
    s = sorted(vals)
    p90 = s[min(len(s) - 1, int(round(0.9 * (len(s) - 1))))]
    return st.median(s), p90, max(s)


def _merge_ratio(round_dir):
    """仲裁评分覆盖率与后置纠偏次数。

    merge.jsonl 是 frontier_merge 的累计计数：
      merge_validated = 仲裁器评分通过的转发数
      merge_raw       = 未经评分的 RRT 原始目标转发数
    两者相加就是 manager 实际能看到的候选供给，所以
      validated / (validated + raw) = **评分覆盖率**。
    基线实测该值约 0.35~0.47，即一半以上的派发绕过评分。
    """
    out = {}
    rows = _jl(os.path.join(round_dir, 'merge.jsonl'))
    if rows:
        last = rows[-1]
        v = int(last.get('merge_validated') or 0)
        r = int(last.get('merge_raw') or 0)
        out = {'validated': v, 'raw': r,
               'score_cover': round(_pct(v, v + r), 1)}
    # 后置纠偏（inplace 原地改目标）次数
    lp = os.path.join(round_dir, 'launch.log')
    if os.path.exists(lp):
        n = 0
        for line in open(lp, encoding='utf-8', errors='replace'):
            if '后置纠偏' in line:
                n += 1
        out['inplace'] = n
    return out


def count_escape_goals(round_dir):
    """改轮内"前沿全灭 → 派脱困目标"的次数（arbiter 日志）。

    这是 P13-4a 的直接观测值：基线多轮为 0（严格轮全空直接返回 None），
    改造后应 >=1/轮。
    """
    lp = os.path.join(round_dir, 'launch.log')
    if not os.path.exists(lp):
        return 0
    n = 0
    for line in open(lp, encoding='utf-8', errors='replace'):
        if '派脱困目标' in line:
            n += 1
    return n


def _bucket_report(rows, key, buckets, title):
    """按 key 分桶统计取消率。rows 需含 label 与非 None 的 key。

    ★ 幻影成功必须从分母里剔除：那类派发根本未被执行，永远不会被取消，
      留在分母会把"远端/大转角"的取消率稀释到很低（首版就出现了
      ">150° 取消率 6%" 的假结论）。这里同时给出 phantom 列供对照。
    """
    print('\n%s' % title)
    print('  %-18s %6s %7s %8s %8s %9s'
          % ('bucket', 'n', 'phantom', 'cancel', 'fail', 'cancel%*'))
    print('  %-18s %6s %7s %8s %8s %9s'
          % ('', '', '', '', '', '(*已剔除幻影)'))
    out = []
    for lo, hi, name in buckets:
        sub = [r for r in rows
               if r.get(key) is not None and lo <= float(r[key]) <= hi
               and r.get('label') in ('reached', 'canceled', 'failed')]
        if not sub:
            continue
        ph = sum(1 for r in sub if r.get('phantom'))
        eff = [r for r in sub if not r.get('phantom')]
        c = sum(1 for r in eff if r['label'] == 'canceled')
        f = sum(1 for r in eff if r['label'] == 'failed')
        k = sum(1 for r in eff if r['label'] == 'reached')
        print('  %-18s %6d %7d %8d %8d %8.0f%%'
              % (name, len(sub), ph, c, f, _pct(c, len(eff))))
        out.append({'key': key, 'bucket': name, 'n': len(sub),
                    'phantom': ph, 'n_eff': len(eff),
                    'canceled': c, 'failed': f, 'reached': k,
                    'phantom_rate': round(_pct(ph, len(sub)), 2),
                    'cancel_rate': round(_pct(c, len(eff)), 2)})
    return out


def main():
    ap = argparse.ArgumentParser(description='派发质量归因（只读）')
    ap.add_argument('dirs', nargs='+', help='logs/round_* 目录或通配符')
    ap.add_argument('--json', default='', help='把汇总写成 JSON')
    ap.add_argument('-v', '--verbose', action='store_true', help='打印每轮明细')
    a = ap.parse_args()

    dirs, seen = [], set()
    for pat in a.dirs:
        ds = sorted(glob.glob(pat))
        for p in (ds if ds else [pat]):
            # 注意：'*v2r*' 也会命中 'offv2r*'，必须按真实路径去重，否则会重复计数
            rp = os.path.realpath(p)
            if os.path.isdir(p) and rp not in seen:
                seen.add(rp)
                dirs.append(p)
    if not dirs:
        print('没有匹配到任何 round 目录', file=sys.stderr)
        return 2

    all_rows = []
    summary = {'rounds': [], 'buckets': {}}
    print('=' * 78)
    print('派发质量归因 · %d 轮' % len(dirs))
    print('=' * 78)
    hdr = ('%-34s %5s %5s %5s %5s %6s %6s %6s %6s %7s'
           % ('round', 'disp', 'real', 'canc', 'phan', 'unk', 'dist50',
              'turn50', '>90%', 'probe%'))
    print(hdr)
    print('-' * len(hdr))

    for d in dirs:
        disp, reach_err, cnt = parse_ledger(d)
        if not disp:
            continue
        n_probe, n_final, n_none = attach_features(d, disp)
        all_rows.extend(disp)
        dists = [x['dist'] for x in disp]
        turns = [x['turn'] for x in disp]
        dm, _dp90, _dmax = _stat(dists)
        tm, _tp90, _tmax = _stat(turns)
        n_known = cnt['reached'] + cnt['canceled'] + cnt['failed']
        merge = _merge_ratio(d)
        rec = {'round': _round_name(d), 'disp': cnt['disp'],
               'reached': cnt['reached'], 'real_reach': cnt['real_reach'],
               'phantom': cnt['phantom'],
               'canceled': cnt['canceled'],
               'failed': cnt['failed'], 'unknown': cnt['unknown'],
               'false_reach': cnt['false_reach'],
               'dist_med': dm, 'turn_med': tm,
               'probe_join': n_probe, 'final_join': n_final,
               'no_feature': n_none,
               'probe_rate': round(_pct(n_probe, cnt['disp']), 1),
               'cancel_rate': round(_pct(cnt['canceled'], n_known), 1),
               'merge': merge,
               'reach_errors': [round(x, 3) for x in reach_err]}
        summary['rounds'].append(rec)
        print('%-34s %5d %5d %5d %5d %6d %6.2f %6.0f %5.0f%% %6.0f%%'
              % (_round_name(d)[-34:], cnt['disp'], cnt['real_reach'],
                 cnt['canceled'], cnt['phantom'], cnt['unknown'],
                 dm if dm is not None else -1,
                 tm if tm is not None else -1,
                 _pct(sum(1 for x in turns if x > 90), len(turns)),
                 _pct(n_probe, cnt['disp'])))
        if a.verbose:
            for x in disp:
                if x['label'] in ('canceled', 'failed') or x.get('false_reach'):
                    print('    %-9s (%.2f,%.2f) dist=%.2f turn=%.0f src=%s '
                          'feat=%s%s'
                          % (x['label'], x['gx'], x['gy'], x['dist'], x['turn'],
                             x['src'], x['feature_source'],
                             ' FALSE_REACH' if x.get('false_reach') else ''))

    # ---------------- 汇总 ----------------
    n = len(all_rows)
    if not n:
        print('没有可用派发记录', file=sys.stderr)
        return 2
    dists = [x['dist'] for x in all_rows]
    turns = [x['turn'] for x in all_rows]
    dm, dp90, dmax = _stat(dists)
    tm, tp90, tmax = _stat(turns)
    known = [x for x in all_rows if x['label'] in ('reached', 'canceled', 'failed')]
    n_known = len(known)
    n_cancel = sum(1 for x in known if x['label'] == 'canceled')
    n_fail = sum(1 for x in known if x['label'] == 'failed')
    n_false = sum(1 for x in all_rows if x.get('false_reach'))
    n_phantom = sum(1 for x in all_rows if x.get('phantom'))
    n_real = len(known) - n_cancel - n_fail - n_phantom
    src_count = {}
    for x in all_rows:
        src_count[x['src']] = src_count.get(x['src'], 0) + 1
    feat_count = {}
    for x in all_rows:
        feat_count[x['feature_source']] = feat_count.get(x['feature_source'], 0) + 1

    print('\n' + '=' * 78)
    print('汇总 · %d 轮 / %d 次派发（去重后）' % (len(dirs), n))
    print('=' * 78)
    print('  终态配对： reached %d / canceled %d / failed %d / 未配对 %d'
          % (len(known) - n_cancel - n_fail, n_cancel, n_fail, n - n_known))
    print('  其中【幻影成功】%d 条（%.1f%% of reached）—— 隐含速度 > %.1f m/s，'
          '物理不可能'
          % (n_phantom, _pct(n_phantom, max(1, len(known) - n_cancel - n_fail)),
             PHANTOM_SPEED))
    print('  ⇒ 真实到达 %d 条；带误差到达（error>%.1fm）%d 条'
          % (n_real, FALSE_REACH_M, n_false))
    print('  取消率：%.1f%%（分母=已配对 %d）' % (_pct(n_cancel, n_known), n_known))
    print('  派发距离：中位 %.2f m / p90 %.2f m / max %.2f m'
          % (dm, dp90, dmax))
    print('      >4 m %.0f%%   >6 m %.0f%%'
          % (_pct(sum(1 for x in dists if x > 4), n),
             _pct(sum(1 for x in dists if x > 6), n)))
    print('  派发转角：中位 %.0f° / p90 %.0f° / max %.0f°' % (tm, tp90, tmax))
    print('      >90° %.0f%%   >150° %.0f%%'
          % (_pct(sum(1 for x in turns if x > 90), n),
             _pct(sum(1 for x in turns if x > 150), n)))
    print('  派发来源：%s' % ', '.join('%s=%d' % kv for kv in
                                   sorted(src_count.items(), key=lambda z: -z[1])))
    print('  特征来源：%s（probe=派发时刻口径，final=最终地图回退）'
          % ', '.join('%s=%d' % kv for kv in sorted(feat_count.items())))

    summary['total'] = {
        'rounds': len(dirs), 'disp': n, 'paired': n_known,
        'canceled': n_cancel, 'failed': n_fail,
        'phantom': n_phantom, 'real_reach': n_real,
        'phantom_rate': round(_pct(n_phantom, max(1, n_known)), 2),
        'inplace_corrections': sum(
            int(r.get('merge', {}).get('inplace', 0)) for r in summary['rounds']),
        'cancel_rate': round(_pct(n_cancel, n_known), 2),
        'false_reach': n_false,
        'dist_med': dm, 'dist_p90': dp90, 'dist_max': dmax,
        'dist_gt4_pct': round(_pct(sum(1 for x in dists if x > 4), n), 1),
        'dist_gt6_pct': round(_pct(sum(1 for x in dists if x > 6), n), 1),
        'turn_med': tm, 'turn_p90': tp90, 'turn_max': tmax,
        'turn_gt90_pct': round(_pct(sum(1 for x in turns if x > 90), n), 1),
        'turn_gt150_pct': round(_pct(sum(1 for x in turns if x > 150), n), 1),
        'sources': src_count, 'feature_sources': feat_count,
    }

    # ---------------- 分桶（物理特征，跨版本可比） ----------------
    b = {}
    b['dist'] = _bucket_report(all_rows, 'dist', [
        (0.0, 3.0, '<3 m'), (3.0, 4.0, '3-4 m'),
        (4.0, 6.0, '4-6 m'), (6.0, 999.0, '>6 m')],
        '按派发距离分桶（取消率）')
    b['turn'] = _bucket_report(all_rows, 'turn', [
        (0.0, 45.0, '<45°'), (45.0, 90.0, '45-90°'),
        (90.0, 150.0, '90-150°'), (150.0, 181.0, '>150°')],
        '按派发转角分桶（取消率）')
    b['esc_dirs'] = _bucket_report(all_rows, 'esc_dirs', [
        (0.0, 2.0, 'esc<=2'), (3.0, 4.0, 'esc=3-4'),
        (5.0, 6.0, 'esc=5-6'), (7.0, 8.0, 'esc=7-8')],
        '按落点可逃脱方向数分桶（仅 features≠none 的子集）')
    b['comp_r2'] = _bucket_report(all_rows, 'comp_r2', [
        (0.0, 400.0, 'comp<=400'), (401.0, 1200.0, 'comp 401-1200'),
        (1201.0, 3000.0, 'comp 1201-3000'), (3001.0, 99999.0, 'comp>3000')],
        '按落点连通自由格数分桶（死口袋检测）')
    b['min_w'] = _bucket_report(all_rows, 'min_w', [
        (0.0, 0.90, '<0.90 m'), (0.90, 1.00, '0.90-1.00 m'),
        (1.00, 1.30, '1.00-1.30 m'), (1.30, 99.0, '>=1.30 m')],
        '按路径最窄净宽分桶')
    if any(x.get('risk_total') is not None for x in all_rows):
        b['risk_total'] = _bucket_report(all_rows, 'risk_total', [
            (0.0, 100.0, 'risk<=100'), (100.0, 200.0, 'risk 100-200'),
            (200.0, 300.0, 'risk 200-300'), (300.0, 9999.0, 'risk>300')],
            '按 P13 风险总分分桶（改造后才可用）')
    summary['buckets'] = b

    if a.json:
        os.makedirs(os.path.dirname(a.json) or '.', exist_ok=True)
        with open(a.json, 'w', encoding='utf-8') as f:
            json.dump(summary, f, ensure_ascii=False, indent=1)
        print('\nJSON → %s' % a.json)
    return 0


if __name__ == '__main__':
    sys.exit(main())
