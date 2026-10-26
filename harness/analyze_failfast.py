#!/usr/bin/env python3
"""analyze_failfast.py —— L1「确定过不去」判定的**离线复算**（只读数据，不改任何东西）。

用户 2026-10-26 要的行为：车"确定过不去"、又长时间在同一小范围里缓慢挪动/静止 → 立刻判失败/换目标。

L1 四条判据（滑窗 W 秒，全部成立才算命中）：
  1) 净位移：窗口内位姿包围盒对角线 <= drift_m
  2) 累计里程：窗口内路径长度 >= path_m（区分"反复蹭"与"完全静止"）
  3) 前进意图：cmd_vx > 0.02 的采样占比 >= intent_frac
  4) 净空证据：窗口内 front 中位数 < front_m （或 front 缺失时看 plan 侧，这里只用地形证据）

输出：
  * 每轮命中次数、首次命中时刻（相对首个派发）、以及比"真卡死事件"早/晚多少；
  * 误报：命中后 10 s 内净位移 > 0.5 m 的次数（说明其实还能走 → 判得太急）；
  * 与 digest.md 里"卡住(cmd非0)"事件的覆盖情况。
用法：
  python3 harness/analyze_failfast.py --glob 'logs/round_*_nosp*' --window 20
  python3 harness/analyze_failfast.py --glob 'logs/round_*' --sweep      # 扫参数组合
"""
import argparse
import glob
import json
import math
import os
import re
import statistics
import sys

W = '/home/kbf/stemm_exploration_sim_ws'
RE_EV = re.compile(r'\|\s*\d+\s*\|\s*(\d+)\s*\|\s*(\d+)\s*\|\s*(\d+)\s*\|\s*\(([-\d.]+),([-\d.]+)\)'
                   r'\s*\|\s*([^|]+)\|')


def load_timeline(d):
    p = os.path.join(d, 'timeline.jsonl')
    rows = []
    if not os.path.exists(p):
        return rows
    for line in open(p, encoding='utf-8', errors='ignore'):
        line = line.strip()
        if not line:
            continue
        try:
            rows.append(json.loads(line))
        except Exception:                       # noqa: BLE001
            pass
    rows = [r for r in rows if r.get('x') is not None and r.get('y') is not None]
    rows.sort(key=lambda r: r['t'])
    return rows


def digest_stuck(d):
    """digest.md 里的卡死事件（含"卡住(cmd非0)"与"空等"），返回 [(t0,t1,kind,x,y)]。"""
    p = os.path.join(d, 'digest.md')
    out = []
    if not os.path.exists(p):
        return out
    txt = open(p, encoding='utf-8', errors='ignore').read()
    for m in RE_EV.finditer(txt):
        t0, t1, dur, x, y, kind = int(m.group(1)), int(m.group(2)), int(m.group(3)), \
            float(m.group(4)), float(m.group(5)), m.group(6).strip()
        out.append((t0, t1, dur, kind, x, y))
    return out


def path_len(pts):
    return sum(math.hypot(pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1])
               for i in range(1, len(pts)))


# 实测标定（2026-10-26，17 个 ≥15 s 事件的特征分布）：
#   A 类「真过不去/被按住」：漂移 ≤0.05 m、里程 ≤0.09 m、意图 ≈0、
#       front 中位 0.15~0.56 m、nav=blocked_waiting_clearance 或 mode=safety_stop
#   B 类「在等」：front 1.5~5.2 m 或 None、mode/nav ∈ {waiting_for_rrt_frontier,
#       warming_up, arrived, returning_home+wait_cartographer, canceled}
#   ⇒ 只有 1) 小范围 2) 净空/状态证据 3) 非"等"状态 同时成立才判——否则会大量误报。
WEDGE_MODES = ('safety_stop', 'navigating', 'returning_home')
WAIT_MODES = ('waiting_for_rrt_frontier', 'warming_up', 'arrived', 'stopped', 'finalizing_map')
WEDGE_NAV = ('blocked_waiting_clearance', 'blocked_precheck_pending', 'executing',
             'recovery', 'turning_out_of_blocked_place', 'sending_goal')
WAIT_NAV = ('waiting_for_rrt_frontier', 'warming_up', 'canceled',
            'return_home_waiting_cartographer_finish', 'succeeded', 'clearance_restored')


def detect(rows, window=15.0, drift_m=0.20, path_m=None, intent_frac=0.0, front_m=0.60,
           cooldown=20.0, need_state=True):
    """L1-v2：「确定过不去」判定。返回命中列表 [(t_hit, x, y, drift, plen, front_med, state_share)]。"""
    hits = []
    last_hit = -1e9
    n = len(rows)
    i = 0
    while i < n:
        t0 = rows[i]['t']
        j = i
        while j + 1 < n and rows[j + 1]['t'] - t0 <= window:
            j += 1
        if j - i < 3 or rows[j]['t'] - t0 < window * 0.8:
            i += 1
            continue
        win = rows[i:j + 1]
        xs = [r['x'] for r in win]
        ys = [r['y'] for r in win]
        drift = math.hypot(max(xs) - min(xs), max(ys) - min(ys))
        plen = path_len([(r['x'], r['y']) for r in win])
        fronts = [r['front'] for r in win if isinstance(r.get('front'), (int, float))]
        front_med = statistics.median(fronts) if fronts else None
        # 非"等"状态占比：mode 不在等待态、nav 不在等待态
        ok_state = 0
        for r in win:
            md = (r.get('mode') or '').lower()
            nv = (r.get('nav') or '').lower()
            if any(w in nv for w in WAIT_NAV) or any(w in md for w in WAIT_MODES):
                continue
            if (any(w in nv for w in WEDGE_NAV) or any(w in md for w in WEDGE_MODES)):
                ok_state += 1
        state_share = ok_state / len(win)
        blocked_nav = sum(1 for r in win if 'blocked_' in (r.get('nav') or '')) / len(win)
        near = drift <= drift_m
        clear = (front_med is not None and front_med < front_m) or blocked_nav >= 0.6
        state_ok = (state_share >= 0.6) if need_state else True
        move_ok = True if path_m is None else (plen >= path_m)
        if near and clear and state_ok and move_ok:
            if win[-1]['t'] - last_hit >= cooldown:
                hits.append((win[-1]['t'], win[-1]['x'], win[-1]['y'], drift, plen,
                             front_med, state_share))
                last_hit = win[-1]['t']
                i = j
                continue
        i += 1
    return hits


def confirm_episodes(hits, radius=2.0, window=60.0):
    """把命中聚成"确认过不去"事件：同一 2 m 区域内 60 s 内命中 >=2 次。返回 [(t_first,t_last,x,y,n)]。"""
    eps = []
    used = [False] * len(hits)
    for i, h in enumerate(hits):
        if used[i]:
            continue
        group = [h]
        used[i] = True
        for j in range(i + 1, len(hits)):
            if used[j]:
                continue
            g = hits[j]
            if math.hypot(g[1] - h[1], g[2] - h[2]) <= radius and g[0] - h[0] <= window:
                group.append(g)
                used[j] = True
        if len(group) >= 2:
            eps.append((group[0][0], group[-1][0], group[0][1], group[0][2], len(group)))
    return eps


def false_positive(rows, hits, horizon=10.0, moved_gate=0.5):
    """命中后 horizon 秒内净位移 > moved_gate ⇒ 其实还能走（判太急，算误报）。"""
    fp = 0
    for (th, _x, _y, *_rest) in hits:
        t0 = th
        seg = [r for r in rows if t0 <= r['t'] <= t0 + horizon]
        if len(seg) < 3:
            continue
        d = math.hypot(seg[-1]['x'] - seg[0]['x'], seg[-1]['y'] - seg[0]['y'])
        if d > moved_gate:
            fp += 1
    return fp


def run(glob_pat, params, quiet=False):
    """按轮统计：长事件(>=15 s)覆盖、L1 命中、误报。"""
    dirs = sorted({os.path.dirname(p) for p in glob.glob(os.path.join(W, glob_pat, 'timeline.jsonl'))})
    tot_hit = tot_fp = tot_eps = tot_eps_wedge = 0
    n_long = n_long_cov = n_wedge = n_wedge_cov = 0
    n_rounds = 0
    lines = []
    for d in dirs:
        rows = load_timeline(d)
        if len(rows) < 30:
            continue
        n_rounds += 1
        evs = digest_stuck(d)
        stuck = [e for e in evs if '卡住' in e[3]]
        longs = [e for e in evs if e[2] >= 15]
        hits = detect(rows, **params)
        eps = confirm_episodes(hits)
        fp = false_positive(rows, hits)
        cov_long = sum(1 for (t0, t1, dur, kind, x, y) in longs
                       if any(t0 - 5 <= h[0] <= t1 + 5 for h in hits))
        wedges = [e for e in longs if '卡住' in e[3] or e[2] >= 30]
        cov_wedge = sum(1 for (t0, t1, dur, kind, x, y) in wedges
                        if any(t0 - 5 <= h[0] <= t1 + 5 for h in hits))
        leads = []
        for (t0, t1, dur, kind, x, y) in wedges:
            near = [h for h in hits if t0 - 60 <= h[0] <= t1 + 5]
            if near:
                leads.append(round(near[0][0] - t0, 0))
        tag = os.path.basename(d).split('_', 2)[-1]
        cov_eps = sum(1 for (t0, t1, dur, kind, x, y) in wedges
                      if any(e[0] - 5 <= t1 and e[1] >= t0 - 5 for e in eps))
        lines.append((tag, len(longs), len(wedges), len(hits), len(eps), fp, cov_long,
                      cov_wedge, cov_eps, leads))
        tot_eps += len(eps)
        tot_eps_wedge += cov_eps
        tot_hit += len(hits)
        tot_fp += fp
        n_long += len(longs)
        n_long_cov += cov_long
        n_wedge += len(wedges)
        n_wedge_cov += cov_wedge
    if not quiet:
        print(f'{"轮次":20s} {"长事件":>5s} {"顶住":>4s} {"命中":>4s} {"确认":>4s} {"误报":>4s} '
              f'{"覆盖长":>6s} {"覆盖顶住":>6s} {"确认覆盖":>6s} {"领先(s)":>18s}')
        for (tag, nl, nw, nh, ne, fp, cl, cw, ce, leads) in lines:
            print(f'{tag:20s} {nl:5d} {nw:4d} {nh:4d} {ne:4d} {fp:4d} '
                  f'{cl:6d} {cw:6d} {ce:6d} {str(leads[:3]):>18s}')
    print(f'\n[{n_rounds} 轮] 长事件 {n_long}（覆盖 {n_long_cov}）｜顶住类 {n_wedge}（覆盖 {n_wedge_cov}，'
          f'确认覆盖 {tot_eps_wedge}）｜L1 命中 {tot_hit} 次 / 确认 {tot_eps} 次（{tot_eps / max(1, n_rounds):.2f}/轮），'
          f'误报 {tot_fp} 次')
    return dict(rounds=n_rounds, hits=tot_hit, eps=tot_eps, fp=tot_fp, long=n_long,
                long_cov=n_long_cov, wedge=n_wedge, wedge_cov=n_wedge_cov,
                wedge_eps=tot_eps_wedge)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--glob', default='logs/round_*')
    ap.add_argument('--window', type=float, default=15.0)
    ap.add_argument('--drift', type=float, default=0.20)
    ap.add_argument('--path', type=float, default=-1.0,
                    help='里程下限；<0 表示不启用（A 类事件里程本来≈0）')
    ap.add_argument('--intent', type=float, default=0.30)
    ap.add_argument('--front', type=float, default=0.60)
    ap.add_argument('--sweep', action='store_true', help='扫几组参数比较')
    a = ap.parse_args()
    base = dict(window=a.window, drift_m=a.drift,
                path_m=(None if a.path < 0 else a.path),
                intent_frac=a.intent, front_m=a.front)
    if a.sweep:
        grid = [
            dict(base),
            dict(base, window=15.0),
            dict(base, window=30.0),
            dict(base, drift_m=0.35),
            dict(base, path_m=0.4),
            dict(base, front_m=0.60),
            dict(base, intent_frac=0.20),
        ]
        print(f'{"window":>7s} {"drift":>6s} {"front":>6s} | {"命中":>5s} {"确认":>5s} {"误报":>4s} '
              f'{"长覆盖":>8s} {"顶住覆盖":>9s} {"确认覆盖":>7s}')
        for g in grid:
            r = run(a.glob, g, quiet=True)
            print(f'{g["window"]:7.0f} {g["drift_m"]:6.2f} {g["front_m"]:6.2f} | '
                  f'{r["hits"]:5d} {r["eps"]:5d} {r["fp"]:4d} '
                  f'{r["long_cov"]:>3d}/{r["long"]:<3d} {r["wedge_cov"]:>3d}/{r["wedge"]:<3d} '
                  f'{r["wedge_eps"]:>3d}')
        return 0
    run(a.glob, base)
    return 0


if __name__ == '__main__':
    sys.exit(main())
