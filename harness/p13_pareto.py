#!/usr/bin/env python3
"""p13_pareto.py —— 多臂扫描的 Pareto 前沿表（论文 E2/E3 的主图数据）。

为什么需要它
------------
`p13_stats.py` 只能做**两组**配对比较。E2 有 4 个臂（inf/5/8/15），
需要的是「每个臂在『供给量 vs 质量』这张平面上的位置」，
以及哪些臂是 Pareto 最优（不被任何其他臂在两个指标上同时压制）。

核心指标对（论文 Fig.3 就是这张图）
----------------------------------
  x 轴：供给量     = merge_validated + merge_raw   （manager 每轮能看到的候选数）
  y 轴：派发质量   = 100 - cancel_rate，或 -dist_med（越大越好）
  附加：known_cells（覆盖面）、path_m（代价）

Pareto 定义（本脚本口径，两个指标都"越大越好"）：
  臂 A 支配臂 B  ⟺  A.x >= B.x  且  A.y >= B.y  且至少一项严格大于
  前沿 = 不被任何其他臂支配的臂集合

用法
----
  python3 harness/p13_pareto.py 'logs/round_*_p13e2_*'
  python3 harness/p13_pareto.py 'logs/round_*_p13e2_*' --y '100-cancel_rate'
  python3 harness/p13_pareto.py 'logs/round_*_p13e2_*' --json /tmp/pareto.json

臂标签从目录名末段解析（`..._p13e2_f51` → `f5`，`..._p13e2_inf1` → `inf`）：
去掉末尾的轮号数字即可。
"""
import argparse
import glob
import json
import os
import re
import statistics as st
import sys

import analyze_dispatch_risk as adr
import analyze_efficiency as aef


def _split_arm_round(tail):
    """把 `..._p13e2_<臂><轮号>` 的末段拆成 (臂, 轮号)。

    ★ 不能靠"去掉末尾一位数字"猜（首版就是这么错）：
      'f51' 应拆为 ('f5','1') 还是 ('f','51')？ 单看字符串无法判断。
      更糟的是 'f151' → 去掉一位得 'f15'、再去得 'f1'、再去得 'f'，
      于是 f5/f8/f15 全被归并成同一个臂 'f'（实测踩到）。

      正确做法：**轮号是 1..N 的连续小整数**，而臂名是用户起的标签。
      所以先枚举"末段末尾 k 位数字"作为轮号候选（k=1,2），
      再用传入的已知臂集合去匹配最长的合法臂前缀。
      当无法判定时，保守地把末尾全部数字当轮号（宁可多分臂不可错并臂）。
    """
    m = re.match(r'^(.*?)(\d+)$', tail)
    if not m:
        return tail, '0'
    return m.group(1), m.group(2)


def _resolve_arm(tail, known_arms):
    """在已收集到的臂名集合里，找 tail 的最长合法前缀。"""
    best = None
    for arm in known_arms:
        if tail == arm:
            return arm, '0'
        if tail.startswith(arm) and tail[len(arm):].isdigit():
            if best is None or len(arm) > len(best):
                best = arm
    if best is None:
        return _split_arm_round(tail)
    return best, tail[len(best):]


def _arm_of(path):
    base = os.path.basename(path.rstrip('/'))
    if '_' not in base:
        return base
    tail = base.rsplit('_', 1)[-1]
    return _split_arm_round(tail)[0]


def _best_dirs(pattern):
    """收集目录并解析臂标签。

    分两步：先按 _split_arm_round 得到**可能的**臂（可能过粗，如 f5/f8/f15 都被
    拆成 'f'），再用"共同前缀最长匹配"在候选集合内收敛。

    单步实现无法同时兼顾以下两类命名：
      inf1 / inf2        → 臂 'inf'（字母臂，轮号 1..N）
      f51 / f52 / f151   → 臂 'f5'/'f15'（数字后缀臂）
    因此这里用启发式：候选臂 = 去掉末尾 1 位数字后的字符串。
    若多条目录去掉末位后得到同一个候选臂且轮号相同，就会冲突——
    所以额外要求同一 (臂,轮号) 只能有一个目录（补跑取最长 timeline）。
    """
    raw = []
    for d in sorted(glob.glob(pattern)):
        base = os.path.basename(d.rstrip('/'))
        tail = base.rsplit('_', 1)[-1]
        t = os.path.join(d, 'timeline.jsonl')
        n = 0
        if os.path.exists(t):
            try:
                with open(t, encoding='utf-8', errors='ignore') as f:
                    n = sum(1 for _ in f)
            except Exception:                       # noqa: BLE001
                n = 0
        raw.append((tail, n, d))

    # 候选臂：末段去掉末尾 1 位数字（轮号假定为 1 位）。
    # 对 f151 会得到 'f15'（对）；对 f51 会得到 'f5'（对）；
    # 对 inf1 会得到 'inf'（对）。当 N>=10 时轮号为 2 位会误判，
    # 故 p13_sweep 的 N 请保持 <=9。
    best = {}
    for tail, n, d in raw:
        m = re.match(r'^(.*?)(\d)$', tail)
        arm, rnd = (m.group(1), m.group(2)) if m else (tail, '0')
        key = (arm, rnd)
        if key not in best or n > best[key][0]:
            best[key] = (n, d)
    out = []
    skipped = []
    for k, (n, d) in sorted(best.items()):
        if n < 100:
            skipped.append((k, 'timeline=%d' % n))
            continue
        # ★ 剔除"零派发/零供给"的无效轮：这类轮的取消率是 0/0=0%，
        #   在"越低越好"的口径下会被误读成最佳，实测把 f8 的
        #   中位取消率从 ~80% 拉到 37.5%，结论完全反了。
        ok, why = adr.round_is_valid(d)
        if not ok:
            skipped.append((k, why))
            continue
        out.append((k[0], d))
    if skipped:
        print('⚠ 剔除 %d 个无效轮（零派发/零供给/时长不足）：' % len(skipped))
        for k, why in skipped:
            print('    %-6s 轮%s  %s' % (k[0], k[1], why))
    return out


def metrics_of(d):
    """单轮的关键指标。"""
    m = {}
    try:
        disp, _e, cnt = adr.parse_ledger(d)
        if disp:
            adr.attach_features(d, disp)
            known = cnt['reached'] + cnt['canceled'] + cnt['failed']
            m['cancel_rate'] = round(adr._pct(cnt['canceled'], known), 1)
            m['disp'] = cnt['disp']
            dm, _p, _x = adr._stat([x['dist'] for x in disp])
            m['dist_med'] = dm
    except Exception:                               # noqa: BLE001
        pass
    try:
        mg = adr._merge_ratio(d)
        m['validated'] = mg.get('validated')
        m['raw'] = mg.get('raw')
        if mg.get('validated') is not None and mg.get('raw') is not None:
            m['supply'] = mg['validated'] + mg['raw']
    except Exception:                               # noqa: BLE001
        pass
    try:
        vd = adr._verdict_report(d)
        m['vbr'] = vd.get('vbr')
        m['v_block'] = vd.get('v_block')
        m['v_force'] = vd.get('v_force')
    except Exception:                               # noqa: BLE001
        pass
    try:
        e = aef.efficiency_one(d)
        if e:
            m['known_cells'] = e['known_cells']
            m['path_m'] = e['path_m']
            m['move_pct'] = e['move_pct']
    except Exception:                               # noqa: BLE001
        pass
    return m


Y_BUILTINS = {
    '100-cancel_rate': lambda m: (100.0 - m['cancel_rate']
                                  if m.get('cancel_rate') is not None else None),
    'cancel_rate': lambda m: m.get('cancel_rate'),
    'dist_med': lambda m: m.get('dist_med'),
    'known_cells': lambda m: m.get('known_cells'),
    'move_pct': lambda m: m.get('move_pct'),
}


def main():
    ap = argparse.ArgumentParser(description='多臂扫描 Pareto 前沿')
    ap.add_argument('pattern')
    ap.add_argument('--x', default='supply', help='x 轴指标（默认 supply=validated+raw）')
    ap.add_argument('--y', default='known_cells',
                    help='y 轴指标：%s' % list(Y_BUILTINS))
    ap.add_argument('--json', default='')
    a = ap.parse_args()

    rows = _best_dirs(a.pattern)
    if not rows:
        print('没有可用轮次', file=sys.stderr)
        return 2

    yfun = Y_BUILTINS.get(a.y)
    if yfun is None:
        print('未知 --y：%s（可选 %s）' % (a.y, list(Y_BUILTINS)), file=sys.stderr)
        return 2

    arms = {}
    for arm, d in rows:
        m = metrics_of(d)
        arms.setdefault(arm, []).append(m)

    print('=' * 100)
    print('多臂扫描 · %d 臂 · %d 轮   x=%s  y=%s' % (len(arms), len(rows), a.x, a.y))
    print('=' * 100)
    hdr = ('%-8s %5s %10s %10s %10s %10s %10s %10s'
           % ('臂', '轮', 'supply', 'validated', 'raw', 'VBR', 'force', a.y))
    print(hdr)
    print('-' * 100)

    summary = {}
    for arm in sorted(arms):
        ms = arms[arm]
        def med(k):
            v = [x.get(k) for x in ms if x.get(k) is not None]
            return round(st.median(v), 2) if v else None
        row = {'n': len(ms), 'supply': med('supply'), 'validated': med('validated'),
               'raw': med('raw'), 'vbr': med('vbr'), 'force': med('v_force'),
               'y': med(a.y if a.y in ms[0] else '_')}
        # y 用函数逐轮算再取中位（支持 100-cancel_rate 这类派生指标）
        yv = [yfun(x) for x in ms]
        yv = [v for v in yv if v is not None]
        row['y'] = round(st.median(yv), 2) if yv else None
        row['cancel_rate'] = med('cancel_rate')
        row['dist_med'] = med('dist_med')
        row['known_cells'] = med('known_cells')
        row['path_m'] = med('path_m')
        row['disp'] = med('disp')
        summary[arm] = row
        print('%-8s %5d %10s %10s %10s %10s %10s %10s'
              % (arm, row['n'], row['supply'], row['validated'], row['raw'],
                 row['vbr'], row['force'], row['y']))

    # ---------------- Pareto 前沿 ----------------
    pts = [(k, v['supply'], v['y']) for k, v in summary.items()
           if v['supply'] is not None and v['y'] is not None]
    front = []
    for name, x, y in pts:
        dominated = False
        for n2, x2, y2 in pts:
            if n2 == name:
                continue
            if x2 >= x and y2 >= y and (x2 > x or y2 > y):
                dominated = True
                break
        if not dominated:
            front.append((name, x, y))

    print('\nPareto 前沿（%s 与 %s 都越大越好）' % (a.x, a.y))
    if not front:
        print('  （无）')
    else:
        for name, x, y in sorted(front, key=lambda t: -t[1]):
            print('  ★ %-8s %s=%-8s %s=%s' % (name, a.x, x, a.y, y))
    excluded = [k for k in summary if k not in [f[0] for f in front]]
    if excluded:
        print('  被支配：%s' % ', '.join(sorted(excluded)))

    print('\n解读提示')
    print('  * 若所有臂的 supply 相近，说明 force_after 不是供给的主控旋钮；')
    print('  * 若 VBR 全部接近 100%，说明判决仍在"全拦"，供给只是被 force 节流；')
    print('  * force 中位为 0 的臂，其 supply 完全来自"无判决"的 raw 转发。')

    if a.json:
        os.makedirs(os.path.dirname(a.json) or '.', exist_ok=True)
        with open(a.json, 'w', encoding='utf-8') as f:
            json.dump({'x': a.x, 'y': a.y, 'arms': summary,
                       'pareto': [{'arm': n, 'x': x, 'y': y}
                                  for n, x, y in front]},
                      f, ensure_ascii=False, indent=1)
        print('\nJSON → %s' % a.json)
    return 0


if __name__ == '__main__':
    sys.exit(main())
