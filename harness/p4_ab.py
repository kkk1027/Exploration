#!/usr/bin/env python3
"""P4/P5 A/B 归因：先远后近两阶段到底改变了什么。

用法：
  python3 harness/p4_ab.py <A组glob> <B组glob>
  python3 harness/p4_ab.py p4base p4far

A 组应为"总开关关闭"（frame_far_first=false，等价改造前），
B 组为开启。只统计有效轮（有 cand_pool.jsonl 的轮）。

输出分三块：
  1. 交付指标（用户关心的结果）：派发距离、已知面积、覆盖效率、停滞、接触
  2. 机制指标（改造是否真的按设计运转）：阶段切换次数、框架阶段占比、
     far_admit 否决率、中继命中率、候选池规模
  3. 排序达成度：框架阶段拍内"rank1 是否真的是最远"的达成率 —— 这是
     "只排序不改生成"这一前提的核心验收项
"""
import glob
import json
import os
import re
import statistics as st
import sys


ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TS_RE = re.compile(r'^\[[^]]*\]\s*\[?[A-Z]*\]?\s*\[([0-9.]+)\]')
DISP_RE = re.compile(r'navigation goal dispatched:.*?target=\(([-\d.]+), ([-\d.]+)\).*?dist=([\d.]+)m')
CANC_RE = re.compile(r'navigation goal canceled; goal=\(([-\d.]+), ([-\d.]+)')
REACH_RE = re.compile(r'navigation goal reached; goal=\(([-\d.]+), ([-\d.]+)')


def rounds(pat):
    return sorted(d for d in glob.glob(os.path.join(ROOT, 'logs', 'round_*' + pat + '*'))
                  if os.path.isdir(d))


def load_pool(d):
    p = os.path.join(d, 'cand_pool.jsonl')
    if not os.path.exists(p):
        return []
    out = []
    for l in open(p, encoding='utf-8', errors='ignore'):
        try:
            out.append(json.loads(l))
        except Exception:                       # noqa: BLE001
            continue
    return out


def load_metrics(d):
    p = os.path.join(d, 'cand_metrics.json')
    if not os.path.exists(p):
        return {}
    try:
        return json.load(open(p, encoding='utf-8'))
    except Exception:                           # noqa: BLE001
        return {}


def load_dispatch(d):
    """派发 → 终态（用派发日志自带的 dist= 字段，最可靠）。"""
    lp = os.path.join(d, 'launch.log')
    if not os.path.exists(lp):
        return []
    rows = []
    cur = None
    for line in open(lp, encoding='utf-8', errors='ignore'):
        if not TS_RE.match(line):
            continue
        m = DISP_RE.search(line)
        if m:
            cur = (float(m.group(1)), float(m.group(2)), float(m.group(3)))
            continue
        if cur is None:
            continue
        for rx, kind in ((CANC_RE, 'c'), (REACH_RE, 'r')):
            mm = rx.search(line)
            if mm and abs(float(mm.group(1)) - cur[0]) < 0.06 \
                    and abs(float(mm.group(2)) - cur[1]) < 0.06:
                rows.append((cur[2], kind))
                cur = None
                break
    return rows


def stat_block(name, a, b, fmt='%.2f'):
    """打印 A/B 对比 + 相对变化。"""
    if not a or not b:
        print('  %-20s %-12s %-12s' % (name, 'n/a', 'n/a'))
        return
    ma, mb = st.median(a), st.median(b)
    delta = mb - ma
    pct = (100.0 * delta / ma) if ma else 0.0
    flag = ''
    if ma and abs(pct) >= 15:
        flag = '  ←'
    print(('  %-20s ' + fmt + '  ' + fmt + '  %+9.2f (%+6.1f%%)' + flag)
          % (name, ma, mb, delta, pct))


def main(argv):
    if len(argv) < 3:
        print(__doc__)
        return 1
    pa, pb = argv[1], argv[2]
    ga, gb = rounds(pa), rounds(pb)
    ga = [d for d in ga if os.path.exists(os.path.join(d, 'cand_pool.jsonl'))]
    gb = [d for d in gb if os.path.exists(os.path.join(d, 'cand_pool.jsonl'))]
    print('A 组 %s：%d 轮    B 组 %s：%d 轮' % (pa, len(ga), pb, len(gb)))
    if not ga or not gb:
        print('⚠ 某组没有 cand_pool.jsonl（探针未开？）')
        return 1

    def collect(g, keys):
        acc = {k: [] for k in keys}
        for d in g:
            m = load_metrics(d)
            for k in keys:
                v = m.get(k)
                if isinstance(v, (int, float)):
                    acc[k].append(v)
        return acc

    keys = ['G1_known_m2', 'G3_traverse_m', 'G4_gain_per_m', 'C3_repeat_ratio',
            'C4_dispatch_dist_med', 'C4_dispatch_dist_max', 'S3_cands_published',
            'S6_cands_per_dispatch', 'rejected_path', 'rejected_narrow']
    A = collect(ga, keys)
    B = collect(gb, keys)

    print('\n=== 1. 交付指标（中位数）===')
    print('  %-20s %-12s %-12s %-20s' % ('指标', pa, pb, 'Δ'))
    stat_block('已知面积 m²', A['G1_known_m2'], B['G1_known_m2'])
    stat_block('行驶里程 m', A['G3_traverse_m'], B['G3_traverse_m'])
    stat_block('每米增益', A['G4_gain_per_m'], B['G4_gain_per_m'])
    stat_block('重复率 C3', A['C3_repeat_ratio'], B['C3_repeat_ratio'], '%.3f')
    stat_block('派发距离中位 m', A['C4_dispatch_dist_med'], B['C4_dispatch_dist_med'])
    stat_block('派发距离最大 m', A['C4_dispatch_dist_max'], B['C4_dispatch_dist_max'])

    # 停滞/等候选 从 digest 读
    def digest_val(g, pat):
        out = []
        for d in g:
            p = os.path.join(d, 'digest.md')
            if not os.path.exists(p):
                continue
            t = open(p, encoding='utf-8', errors='ignore').read()
            m = re.search(pat, t)
            if m:
                out.append(float(m.group(1)))
        return out
    stall_pat = r'停滞总秒数[:：]\s*\*\*([\d.]+)'
    wait_pat = r'等候选[^:：]*[:：]\s*\*\*([\d.]+)'
    stat_block('停滞总秒数', digest_val(ga, stall_pat), digest_val(gb, stall_pat), '%.1f')
    stat_block('等候选秒数', digest_val(ga, wait_pat), digest_val(gb, wait_pat), '%.1f')

    # 派发距离分布（全部派发汇总）
    def dist_all(g):
        return [d for x in g for (d, _k) in load_dispatch(x)]
    da, db = dist_all(ga), dist_all(gb)
    print('\n=== 2. 派发距离分布（全部派发汇总）===')
    for lo, hi, nm in ((0, 3, '<3 m'), (3, 4.5, '3-4.5'), (4.5, 6, '4.5-6'),
                       (6, 99, '>6 m')):
        ca = sum(1 for x in da if lo <= x < hi)
        cb = sum(1 for x in db if lo <= x < hi)
        ra = 100.0 * ca / len(da) if da else 0
        rb = 100.0 * cb / len(db) if db else 0
        print('  %-8s A %4d (%5.1f%%)   B %4d (%5.1f%%)   Δ %+5.1f pt'
              % (nm, ca, ra, cb, rb, rb - ra))

    # 距离分桶取消率
    def canc_by_bucket(g):
        acc = {}
        for x in g:
            for (dist, kind) in load_dispatch(x):
                k = ('<3' if dist < 3 else '3-4.5' if dist < 4.5
                     else '4.5-6' if dist < 6 else '>6')
                s = acc.setdefault(k, [0, 0])
                s[0] += 1
                if kind == 'c':
                    s[1] += 1
        return acc
    ca_b, cb_b = canc_by_bucket(ga), canc_by_bucket(gb)
    print('\n=== 3. 取消率（按派发距离）===')
    print('  %-8s %-18s %-18s' % ('距离段', pa, pb))
    for k in ('<3', '3-4.5', '4.5-6', '>6'):
        sa, sb = ca_b.get(k, [0, 0]), cb_b.get(k, [0, 0])
        fa = '%5.1f%% (%d/%d)' % (100.0 * sa[1] / sa[0], sa[1], sa[0]) if sa[0] else '     n/a'
        fb = '%5.1f%% (%d/%d)' % (100.0 * sb[1] / sb[0], sb[1], sb[0]) if sb[0] else '     n/a'
        print('  %-8s %-18s %-18s' % (k, fa, fb))

    print('\n=== 4. 机制指标：改造是否真的按设计运转 ===')
    for nm, g in (('A ' + pa, ga), ('B ' + pb, gb)):
        n_ticks = 0
        n_pool = []
        fr_tot = fr_win = 0         # 框架阶段：rank1 是否最远
        fi_tot = fi_win = 0         # 补齐阶段：rank1 是否最高分
        phases = {}
        depth_vals = []
        for d in g:
            for row in load_pool(d):
                rows = row.get('rows') or []
                n_ticks += 1
                n_pool.append(row.get('n_pool') or 0)
                ph = row.get('phase') or 'fill'
                phases[ph] = phases.get(ph, 0) + 1
                for x in rows:
                    if x.get('depth') is not None:
                        depth_vals.append(x['depth'])
                ds = [x['dist'] for x in rows]
                ks = [x['key'] for x in rows]
                if len(ds) >= 2:
                    if ph == 'frame':
                        fr_tot += 1
                        if ds[0] >= max(ds) - 1e-9:
                            fr_win += 1
                    else:
                        fi_tot += 1
                        if ks[0] >= max(ks) - 1e-9:
                            fi_win += 1
        # 统计 P4/P5 计数
        cnt = {}
        for d in g:
            lp = os.path.join(d, 'launch.log')
            if not os.path.exists(lp):
                continue
            last = None
            for line in open(lp, encoding='utf-8', errors='ignore'):
                m = re.search(r'统计 (\{[^}]*\})', line)
                if m:
                    last = m.group(1)
            if last:
                import ast
                try:
                    s = ast.literal_eval(last)
                except Exception:                        # noqa: BLE001
                    s = {}
                for k in ('phase_to_fill', 'phase_to_frame', 'far_admit_reject',
                          'far_relay_ok', 'far_relay_miss', 'frame_depth_reject'):
                    cnt[k] = cnt.get(k, 0) + (s.get(k) or 0)
        print('  --- %s ---' % nm)
        print('    候选池快照拍数 = %d，n_pool 中位 = %s'
              % (n_ticks, st.median(n_pool) if n_pool else 'n/a'))
        print('    阶段分布 = %r' % (phases,))
        if fr_tot:
            print('    框架阶段 rank1 确实最远 = %d/%d = %.0f%%'
                  % (fr_win, fr_tot, 100.0 * fr_win / fr_tot))
        if fi_tot:
            print('    补齐阶段 rank1 确实最高分 = %d/%d = %.0f%%'
                  % (fi_win, fi_tot, 100.0 * fi_win / fi_tot))
        if depth_vals:
            print('    未知厚度 depth：中位 %.2f m  P90 %.2f m'
                  % (st.median(depth_vals),
                     sorted(depth_vals)[int(0.9 * (len(depth_vals) - 1))]))
        if cnt:
            print('    阶段切换 框架→补齐 %d / 补齐→框架 %d（每轮中位 %.1f 次）'
                  % (cnt.get('phase_to_fill', 0), cnt.get('phase_to_frame', 0),
                     (cnt.get('phase_to_fill', 0)
                      + cnt.get('phase_to_frame', 0)) / max(1, len(g))))
            print('    far_admit 否决 %d / 中继 命中 %d 未中 %d / 厚度否决 %d'
                  % (cnt.get('far_admit_reject', 0), cnt.get('far_relay_ok', 0),
                     cnt.get('far_relay_miss', 0),
                     cnt.get('frame_depth_reject', 0)))

    print('\n提示：≥10 轮/组再用配对 bootstrap 判定显著性；'
          '本脚本只给方向与机制归因。')
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
