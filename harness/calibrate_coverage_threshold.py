#!/usr/bin/env python3
"""θ 标定：离线回放 NCR 曲线，扫多个阈值，看"若用这个 θ 会何时收敛"。

为什么必须回放而不是直接看最终值
--------------------------------
最终 NCR 只是一个数（`p14vU` 十轮中位 0.949）。真正要回答的是
**"用 θ 会在哪一刻停下，停下时是不是已经把该探的探完"**。
这只能在时间序列上回放才能看出来。

判据（本脚本的判定逻辑）
------------------------
对每个候选 θ：

  t_stop(θ) = NCR 连续 H 次 >= θ 的首次时刻
  waste(θ)  = (最终 known − t_stop 时的 known) / 最终 known
              = "收敛之后本来还能探到多少，被丢掉了"
  delay(θ)  = t_stop − t_baseline（baseline = 定时器结束时刻，即 explore_sec）

三个理想性质：
  1. waste 小（≤5%）—— 停下时不漏探
  2. t_stop 不早于"地图基本定型"的时刻
  3. delay >= 0 或 waste 已足够小 —— 要么更晚结束，要么早结束但没漏

只给出指标，**不自动选 θ**：最终取哪个由人看表决定，避免"程序挑了个
数字就当成结论"。

用法
----
  python3 harness/calibrate_coverage_threshold.py
  python3 harness/calibrate_coverage_threshold.py --pattern p14vX --explore-sec 240
"""
import argparse
import glob
import json
import os
import statistics as st
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
THETAS = (0.90, 0.92, 0.94, 0.95, 0.96, 0.97, 0.98)


def load_curve(d):
    """返回 [(sim, elapsed, ncr_int, known_total, converged)]，按时间排序。"""
    p = os.path.join(d, 'timeline.jsonl')
    if not os.path.exists(p):
        return []
    rows = []
    with open(p) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                r = json.loads(line)
            except Exception:
                continue
            if r.get('ncr_interior') is None:
                continue
            rows.append((
                r.get('sim'),
                r.get('ncr_elapsed'),
                float(r['ncr_interior']),
                r.get('ncr_nav_known'),
                r.get('ncr_reach_interior'),
                r.get('ncr_reach_exterior'),
                bool(r.get('ncr_converged')),
            ))
    return rows


def replay(curve, theta, confirm_n, min_elapsed=0.0, min_known=0):
    """回放：复现 manager 的完整判据。

    只比 `ncr >= theta` 是错的 —— manager 还有两道正交门槛：
      * elapsed >= coverage_min_explore_sec
      * known_total >= coverage_min_known_cells
    不回放它们会得出"θ=0.98 也在 sim=16 就收敛、漏探 61%"这种荒谬结论
    （2026-11-15 实际踩到，当时以为是 manager 判错，其实是脚本漏了门槛）。
    """
    streak = 0
    for i, row in enumerate(curve):
        ncr = row[2]
        elapsed = row[1] or 0.0
        known = row[3] or 0
        gate_ok = (elapsed >= min_elapsed and known >= min_known)
        if gate_ok and ncr >= theta:
            streak += 1
            if streak >= confirm_n:
                return i
        else:
            streak = 0
    return None


def main(argv):
    ap = argparse.ArgumentParser()
    ap.add_argument('--pattern', default='p14vX')
    ap.add_argument('--dirs', nargs='*')
    ap.add_argument('--explore-sec', type=float, default=240.0,
                    help='本轮定时器时长（作为 delay 的基线）')
    ap.add_argument('--confirm-n', type=int, default=3)
    ap.add_argument('--min-elapsed', type=float, default=60.0,
                    help='manager 的 coverage_min_explore_sec')
    ap.add_argument('--min-known', type=int, default=0,
                    help='manager 的 coverage_min_known_cells')
    args = ap.parse_args(argv[1:])

    dirs = args.dirs or sorted(glob.glob(os.path.join(
        ROOT, 'logs', 'round_*_%s*' % args.pattern)))
    if not dirs:
        print('没有轮次')
        return 2

    curves = []
    for d in dirs:
        c = load_curve(d)
        if len(c) < 10:
            print('跳过（数据太少）：%s' % os.path.basename(d))
            continue
        curves.append((os.path.basename(d), c))
    if not curves:
        print('没有可用曲线')
        return 1

    print('## 各轮 NCR 轨迹概要\n')
    print('%-30s %-6s %-9s %-9s %-9s %-10s' % (
        '轮次', '拍数', 'NCR末', 'known末', 'NCR峰值', '峰值时刻'))
    print('-' * 78)
    for name, c in curves:
        ncr_last = c[-1][2]
        known_last = c[-1][3] or 0
        peak_i = max(range(len(c)), key=lambda i: c[i][2])
        print('%-30s %-6d %-9.4f %-9d %-9.4f %-10s' % (
            name[:29], len(c), ncr_last, known_last, c[peak_i][2],
            c[peak_i][0]))

    print('\n## 阈值回放（confirm_n=%d, min_elapsed=%.0fs, min_known=%d）\n'
          % (args.confirm_n, args.min_elapsed, args.min_known))
    hdr = ('%-7s' % 'θ'
           + ''.join('%-9s' % k for k in ('命中轮数', '中位t_stop', '中位known@stop',
                                          '中位waste', '中位delay')) )
    print(hdr)
    print('-' * len(hdr))
    summary = {}
    for theta in THETAS:
        stops, wastes, delays, known_at = [], [], [], []
        hit = 0
        for name, c in curves:
            idx = replay(c, theta, args.confirm_n,
                         args.min_elapsed, args.min_known)
            known_last = c[-1][3] or 0
            if idx is None:
                continue
            hit += 1
            t_stop = c[idx][0] or 0.0
            k_stop = c[idx][3] or 0
            known_at.append(k_stop)
            stops.append(t_stop)
            if known_last > 0:
                wastes.append((known_last - k_stop) / float(known_last))
            delays.append(t_stop - args.explore_sec)
        summary[theta] = (hit, stops, wastes, delays, known_at)
        if not stops:
            print('%-7s %-9d %-9s %-9s %-9s %-9s' % (
                '%.2f' % theta, hit, '-', '-', '-', '-'))
            continue
        print('%-7s %-9s %-9s %-9s %-9s %-9s' % (
            '%.2f' % theta,
            '%d/%d' % (hit, len(curves)),
            '%.0f s' % st.median(stops),
            '%d' % st.median(known_at),
            '%.1f%%' % (100 * st.median(wastes)),
            '%+.0f s' % st.median(delays)))

    print('\n## 判读\n')
    print('- **waste** = 收敛时相比最终地图漏掉的已知面积比例。越小越安全。')
    print('- **delay** = 相比定时器（%.0f s）提前/延后多少秒。负数=更早结束。'
          % args.explore_sec)
    print('- 理想：waste <= 5%% 且 命中轮数 = 全部轮次。')
    print('- 若某 θ 命中数 < 全部，说明该轮永远达不到 —— 若无兜底会一直跑。')
    print()
    print('⚠️ **不要只看 waste 小就选高 θ**：θ 高到没人命中，等于判据失效。')
    print('   必须在"命中全部轮次"的前提下取 waste 最小的那个。')

    ok = [(t, s) for t, s in summary.items() if s[0] == len(curves) and s[2]]
    if ok:
        best = min(ok, key=lambda kv: st.median(kv[1][2]))
        print('\n在"命中全部轮次"前提下 waste 最小：θ = **%.2f**（waste %.1f%%）'
              % (best[0], 100 * st.median(best[1][2])))
    else:
        print('\n**没有 θ 能命中全部轮次** —— 需要配合"已知面积门槛"或兜底。')
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
