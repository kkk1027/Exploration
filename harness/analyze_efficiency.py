#!/usr/bin/env python3
"""analyze_efficiency.py —— 探索效率指标（论文 E1/E3 的主指标之一）。

为什么需要它
------------
`analyze_dispatch_risk.py` 只看**派发质量**（距离/转角/取消率），
`p13_ab_report.py` 只看**覆盖率**。两者都不回答：
  * 车到底跑了多远才覆盖这么多？
  * 有多少时间是空转（等着/卡着/脱困）？
  * 探索阶段什么时候结束的？

这三个量是论文里「判决传递是否让探索更有效率」的直接证据。

口径定义（全部只用 `timeline.jsonl`，跨版本可比）
--------------------------------------------------
探索窗口   = 从 sim 起点到**首次进入 returning_home / finalizing_map** 之前
             （之后是返航，不属于探索）
路径长度   = 窗口内相邻采样点的欧氏距离累加（采样 1 Hz，含原地旋转的抖动）
有效前进   = 位移模长 > 0.02 m 的采样点数 / 窗口采样点数
空转占比   = 1 - 有效前进（等待候选、卡住、脱困中的原地转都算空转）
完成时刻   = 首次进入 returning_home 的 sim 时间（越大说明探索越慢）
覆盖率     = 从 map.yaml 读回的已知格数（需要 map.pgm；缺失则跳过）

用法：
  python3 harness/analyze_efficiency.py 'logs/round_*_p13f_a*'
  python3 harness/analyze_efficiency.py logs/round_xxx -v
  python3 harness/analyze_efficiency.py 'logs/round_*' --json /tmp/eff.json
"""
import argparse
import glob
import json
import math
import os
import statistics as st
import sys

EXPLORE_MODES = ('navigating', 'mapping', 'waiting_for_rrt_frontier',
                 'idle', 'safety_stop', 'arrived')
DONE_MODES = ('returning_home', 'finalizing_map', 'return_home_failed_waiting_manual')
MOVE_EPS = 0.02          # m，1 Hz 采样下「真的动了」的下限


def _jl(path):
    out = []
    if not os.path.exists(path):
        return out
    for ln in open(path, encoding='utf-8', errors='ignore'):
        ln = ln.strip()
        if not ln:
            continue
        try:
            out.append(json.loads(ln))
        except Exception:                          # noqa: BLE001
            pass
    return out


def _round_name(d):
    return os.path.basename(d.rstrip('/'))


def _known_cells(round_dir):
    """从 map.pgm + map.yaml 数已知格（非 205 灰度 = 非 unknown）。"""
    try:
        import numpy as np
    except Exception:                              # noqa: BLE001
        return None
    pgm = os.path.join(round_dir, 'map.pgm')
    if not os.path.exists(pgm):
        return None
    try:
        with open(pgm, 'rb') as f:
            data = f.read()
        # P5 二进制头：P5\n<w> <h>\n<max>\n
        parts = []
        idx = 0
        while len(parts) < 4 and idx < len(data):
            # 跳过注释行
            while data[idx:idx + 1] == b'#':
                idx = data.index(b'\n', idx) + 1
            j = idx
            while j < len(data) and data[j:j + 1].split():
                j += 1
            parts.append(data[idx:j])
            idx = j + 1
        w = int(parts[1])
        h = int(parts[2])
        pix = np.frombuffer(data[idx:idx + w * h], dtype=np.uint8)
        if pix.size < w * h:
            return None
        known = int((pix != 205).sum())
        return known
    except Exception:                              # noqa: BLE001
        return None


def efficiency_one(round_dir):
    rows = _jl(os.path.join(round_dir, 'timeline.jsonl'))
    if len(rows) < 5:
        return None
    t0 = None
    t_done = None
    prev = None
    path = 0.0
    n_move = 0
    n_win = 0
    stuck_by_reason = {}
    for r in rows:
        mode = r.get('mode')
        if t0 is None and r.get('sim') is not None:
            t0 = float(r['sim'])
        if mode in DONE_MODES:
            if t_done is None and r.get('sim') is not None:
                t_done = float(r['sim'])
            break
        if mode is not None and mode not in EXPLORE_MODES:
            continue
        n_win += 1
        x, y = r.get('x'), r.get('y')
        if prev is not None and x is not None and y is not None:
            d = math.hypot(x - prev[0], y - prev[1])
            path += d
            if d > MOVE_EPS:
                n_move += 1
            else:
                k = str(r.get('nav') or mode or '?')
                stuck_by_reason[k] = stuck_by_reason.get(k, 0) + 1
        if x is not None and y is not None:
            prev = (x, y)
    sim_end = float(rows[-1].get('sim') or 0.0)
    win = (t_done if t_done is not None else sim_end) - (t0 or 0.0)
    known = _known_cells(round_dir)
    top_stuck = sorted(stuck_by_reason.items(), key=lambda kv: -kv[1])[:3]
    return {
        'round': _round_name(round_dir),
        'win_s': round(win, 1),
        'done_s': round(t_done, 1) if t_done is not None else None,
        'finished': t_done is not None,
        'path_m': round(path, 2),
        'move_pct': round(100.0 * n_move / n_win, 1) if n_win else None,
        'idle_pct': round(100.0 * (1.0 - n_move / n_win), 1) if n_win else None,
        'known_cells': known,
        'stuck_top': top_stuck,
    }


def _stat(vals):
    v = [x for x in vals if x is not None]
    if not v:
        return None, None
    s = sorted(v)
    return round(st.median(s), 2), round(s[-1], 2)


def main():
    ap = argparse.ArgumentParser(description='探索效率指标（窗口/路径/空转）')
    ap.add_argument('patterns', nargs='+', help='round 目录或 glob')
    ap.add_argument('--json', default='')
    ap.add_argument('-v', '--verbose', action='store_true')
    a = ap.parse_args()

    dirs = []
    for p in a.patterns:
        got = sorted(glob.glob(p)) or ([p] if os.path.isdir(p) else [])
        for d in got:
            rp = os.path.realpath(d)
            if rp not in [os.path.realpath(x) for x in dirs]:
                dirs.append(d)
    rows = [r for r in (efficiency_one(d) for d in dirs) if r]
    if not rows:
        print('没有可用 timeline.jsonl', file=sys.stderr)
        return 2

    print('=' * 96)
    print('探索效率 · %d 轮' % len(rows))
    print('=' * 96)
    print('%-42s %7s %7s %8s %8s %8s'
          % ('round', '窗口s', '完成s', '路径m', '前进%', '已知格'))
    print('-' * 96)
    for r in rows:
        print('%-42s %7.1f %7s %8.2f %8s %8s'
              % (_round_name(r['round'])[-42:], r['win_s'],
                 ('%.1f' % r['done_s']) if r['done_s'] is not None else '未完',
                 r['path_m'],
                 ('%.1f' % r['move_pct']) if r['move_pct'] is not None else '-',
                 r['known_cells'] if r['known_cells'] is not None else '-'))

    print('\n汇总')
    wm, wx = _stat([r['win_s'] for r in rows])
    pm, px = _stat([r['path_m'] for r in rows])
    mm, mx = _stat([r['move_pct'] for r in rows])
    dm, _dx = _stat([r['done_s'] for r in rows])
    km, _kx = _stat([r['known_cells'] for r in rows])
    nfin = sum(1 for r in rows if r['finished'])
    print('  探索窗口      中位 %.1f s / 最长 %.1f s' % (wm, wx))
    print('  完成时刻      中位 %s（%d/%d 轮在时长内进入返航）'
          % (('%.1f s' % dm) if dm is not None else '未完成', nfin, len(rows)))
    print('  路径长度      中位 %.2f m / 最长 %.2f m' % (pm, px))
    print('  有效前进占比  中位 %.1f%% / 最高 %.1f%%（其余为等待/卡住/脱困）'
          % (mm, mx))
    if km is not None:
        print('  已知格数      中位 %d' % km)

    # 空转原因（跨轮聚合）
    agg = {}
    for r in rows:
        for k, n in r['stuck_top']:
            agg[k] = agg.get(k, 0) + n
    if agg:
        print('\n空转原因 Top6（按采样点数，跨轮累计）')
        for k, n in sorted(agg.items(), key=lambda kv: -kv[1])[:6]:
            print('  %-42s %d' % (k, n))

    if a.verbose:
        print('\n逐轮空转原因')
        for r in rows:
            print('  %-40s %s' % (_round_name(r['round'])[-40:],
                                  ', '.join('%s=%d' % kv for kv in r['stuck_top'])))

    if a.json:
        os.makedirs(os.path.dirname(a.json) or '.', exist_ok=True)
        with open(a.json, 'w', encoding='utf-8') as f:
            json.dump({'rounds': rows,
                       'summary': {'win_med': wm, 'path_med': pm,
                                   'move_med': mm, 'done_med': dm,
                                   'known_med': km, 'finished': nfin,
                                   'n': len(rows)}}, f,
                      ensure_ascii=False, indent=1)
        print('\nJSON → %s' % a.json)
    return 0


if __name__ == '__main__':
    sys.exit(main())
