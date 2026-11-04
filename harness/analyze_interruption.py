#!/usr/bin/env python3
"""analyze_interruption.py —— 「前往候选途中被【无故】打断并更换候选」的次数（可复核口径）。

用户 2026-11-04 新增指标。**不用日志关键词归因**（上一版用关键词会被
collision_probe/planner 的常规日志误伤成 false positive），改成
**计数器增量**，判据全部来自现成产物：

  candidate_probe.jsonl  每次派发一条：dispatch_idx / t / goal / robot / dist_to_robot
  timeline.jsonl (1 Hz)  位姿 (x,y) + nav 状态 + 一串累计计数器：
                         veto / reject_cost / reject_noexit / reject_corridor /
                         rejected_narrow / corridor_veto / escape_goal / fallback_obs /
                         contact_fail_cancel / published / busy / n_cands
  probe.jsonl   (1 Hz)   contacts / contact_episodes / stalls

逐"派发区间"判定（第 i 次派发 → 第 i+1 次派发）：
  reached   区间内到该目标的最小距离 ≤ --reach-m（默认 0.50 m）⇒ 正常到达，不算打断
  mid       未到达且区间末已换目标 ⇒ **中途更换**
  无故条件  同时满足：
              (a) 中途更换
              (b) 区间内所有"正当理由"计数器**零增量**：
                  veto / reject_cost / reject_noexit / reject_corridor /
                  rejected_narrow / corridor_veto / escape_goal / fallback_obs /
                  contact_fail_cancel
              (c) 区间内没有新增物理接触（probe 的 contact_episodes / contacts）
              (d) 区间内没有新增卡死事件（probe 的 stalls）
              (e) 区间内 nav 状态出现过 executing（说明不是"根本没在跑"）
  ⇒ 计 1 次「无故打断」

输出：每轮的总派发/到达/中途更换/无故打断，以及无故事件的明细
（时刻、目标、剩余距离、走了多远、区间时长）。

用法：
  python3 harness/analyze_interruption.py --glob 'logs/round_*_m5r*' [--verbose]
"""
import argparse
import glob
import json
import math
import os
import statistics as st
import sys

COUNTERS = ('veto', 'reject_cost', 'reject_noexit', 'reject_corridor',
            'rejected_narrow', 'rejected_oneway', 'corridor_veto',
            'escape_goal', 'fallback_obs', 'contact_fail_cancel')


def load_jsonl(p):
    if not os.path.exists(p):
        return []
    out = []
    for line in open(p, encoding='utf-8'):
        line = line.strip()
        if line:
            try:
                out.append(json.loads(line))
            except Exception:                                  # noqa: BLE001
                pass
    return out


def in_window(rows, t0, t1):
    return [r for r in rows if t0 <= (r.get('t') or 0) <= t1]


def counter_delta(rows, key):
    """区间内某计数器是否有增长（累计量）。"""
    vals = [r.get(key) for r in rows if isinstance(r.get(key), (int, float))]
    if len(vals) < 2:
        return 0
    return max(0, vals[-1] - vals[0])


def analyze_round(d, reach_m, verbose=False, same_goal_m=0.30):
    cp = load_jsonl(os.path.join(d, 'candidate_probe.jsonl'))
    tl = load_jsonl(os.path.join(d, 'timeline.jsonl'))
    pr = load_jsonl(os.path.join(d, 'probe.jsonl'))
    disps = [r for r in cp
             if r.get('dispatch_idx') is not None
             and isinstance(r.get('goal'), (list, tuple)) and len(r['goal']) >= 2
             and r.get('t') is not None]
    disps.sort(key=lambda r: r['t'])
    # ★ 合并"同一目标的重复发布"：candidate_probe 每发一次 dispatch_idx 就 +1，
    #   连续同目标会变成很多 1 秒级区间，从而被误判成"每次都中途更换"。
    merged = []
    for r in disps:
        g = (float(r['goal'][0]), float(r['goal'][1]))
        if merged and math.hypot(g[0] - merged[-1]['goal'][0], g[1] - merged[-1]['goal'][1]) <= same_goal_m:
            merged[-1]['rep'] += 1
            continue
        merged.append(dict(t=r['t'], goal=g, rep=1))
    disps = merged
    if len(disps) < 2:
        return None
    t_end = tl[-1]['t'] if tl else disps[-1]['t'] + 60

    events = []
    for i, cur in enumerate(disps):
        t0 = cur['t']
        t1 = disps[i + 1]['t'] if i + 1 < len(disps) else t_end
        goal = tuple(cur['goal'])
        seg = in_window(tl, t0, t1)
        dmin = None
        for r in seg:
            if r.get('x') is None:
                continue
            dd = math.hypot(r['x'] - goal[0], r['y'] - goal[1])
            dmin = dd if dmin is None else min(dmin, dd)
        reached = (dmin is not None and dmin <= reach_m)
        last_d = None
        if seg:
            last = [r for r in seg if r.get('x') is not None]
            if last:
                last_d = math.hypot(last[-1]['x'] - goal[0], last[-1]['y'] - goal[1])
        flags = {k: counter_delta(seg, k) for k in COUNTERS}
        # 接触/卡死增量
        seg_pr = in_window(pr, t0, t1)

        def pdelta(key):
            vals = [r.get(key) for r in seg_pr if isinstance(r.get(key), (int, float))]
            return max(0, vals[-1] - vals[0]) if len(vals) >= 2 else 0

        d_contacts = pdelta('contact_episodes') + pdelta('contacts')
        d_stalls = pdelta('stalls')
        navs = {r.get('nav') for r in seg if r.get('nav')}
        justified = (any(v > 0 for v in flags.values()) or d_contacts > 0
                     or d_stalls > 0 or ('executing' not in navs))
        ev = dict(i=i, t0=round(t0, 1), dur=round(t1 - t0, 1), goal=goal,
                  rep=cur.get('rep', 1),
                  d_min=(round(dmin, 2) if dmin is not None else None),
                  d_end=(round(last_d, 2) if last_d is not None else None),
                  reached=reached, navs=sorted(n for n in navs if n),
                  flags={k: v for k, v in flags.items() if v},
                  d_contacts=d_contacts, d_stalls=d_stalls, justified=justified)
        if not reached:
            ev['unexplained'] = not justified
        events.append(ev)
        if verbose:
            tag = ('到达' if reached else ('无故打断' if ev['unexplained'] else '有因更换'))
            print(f"    #{i:02d} {tag} 目标={goal} 重复发布={ev['rep']} 最近={ev['d_min']} "
                  f"末距={ev['d_end']} 时长={ev['dur']}s 理由={ev['flags'] or '-'} "
                  f"接触+{d_contacts} 卡死+{d_stalls}")

    mid = [e for e in events if not e['reached']]
    unc = [e for e in mid if e.get('unexplained')]
    return dict(round=os.path.basename(d), dispatches=len(disps),
                reached=sum(1 for e in events if e['reached']),
                mid_way=len(mid), unexplained=len(unc),
                unexplained_events=unc,
                mid_mean_dur=(round(st.mean([e['dur'] for e in mid]), 1) if mid else None),
                unexplained_mean_dur=(round(st.mean([e['dur'] for e in unc]), 1) if unc else None))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--glob', default='logs/round_*')
    ap.add_argument('--reach-m', type=float, default=0.50)
    ap.add_argument('--same-goal-m', type=float, default=0.30,
                    help='目标相距小于该值视为"同一次前往"（合并重复发布），默认 0.30 m')
    ap.add_argument('--out', default='logs/interruption.json')
    ap.add_argument('--verbose', action='store_true')
    a = ap.parse_args()
    rows = []
    for d in sorted(glob.glob(a.glob)):
        if not os.path.isdir(d):
            continue
        if a.verbose:
            print(f'--- {os.path.basename(d)}')
        r = analyze_round(d, a.reach_m, a.verbose, a.same_goal_m)
        if not r:
            continue
        rows.append(r)
        print(f"{r['round'][-26:]:28s} 派发={r['dispatches']:3d} 到达={r['reached']:3d} "
              f"中途更换={r['mid_way']:3d} **无故打断={r['unexplained']:3d}** "
              f"（中途段均时长={r['mid_mean_dur']}s）", flush=True)
    if not rows:
        print('没有可分析的轮次')
        return 1
    json.dump(rows, open(a.out, 'w', encoding='utf-8'), ensure_ascii=False, indent=1)
    U = [r['unexplained'] for r in rows]
    M = [r['mid_way'] for r in rows]
    N = [r['dispatches'] for r in rows]
    print('\n===== 汇总（%d 轮）=====' % len(rows))
    print(f"  每轮派发        中位 {st.median(N):.0f}  合计 {sum(N)}")
    print(f"  每轮中途更换    中位 {st.median(M):.0f}  合计 {sum(M)}")
    print(f"  每轮无故打断    中位 {st.median(U):.0f}  合计 {sum(U)}   ← 用户新增指标")
    if sum(M):
        print(f"  中途更换中「无故」占比 {sum(U) / sum(M) * 100:.1f}%")
    print(f'  输出 {a.out}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
