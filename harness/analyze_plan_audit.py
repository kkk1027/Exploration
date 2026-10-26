#!/usr/bin/env python3
"""analyze_plan_audit.py —— 从 launch.log 的"计划窄处体检"行统计规划安全性。

背景（docs/P9 §19）：用户报"地图上有明显的红障碍，规划路径却直接横穿"。要判定这件事，
必须把两类"最小净宽很小"的计划分开：
  * **起点就在障碍格里**（`min@#0`）—— 车贴着障碍时 clearance_at 取的是车所在格的
    距离场，天然很小，这是测量假象，不是规划错误；
  * **中途真的穿过去**（`min@#k, k>0`）—— 规划器在知道障碍的前提下仍穿过去，才是 bug。

日志格式（`0383bf6` 起）：
  计划窄处体检：/plan 42 点，最小净宽 0.00 m@#3(2.84,2.79)，起点净宽 0.12 m，
  最长窄段 0.15 m（pass_width=1.05 / narrow_len_max=1.00）→ 穿窄处
旧格式（无 `@#`）只能统计条数与最小净宽，脚本兼容。

用法：
  python3 harness/analyze_plan_audit.py v8a            # 按前缀匹配 logs/round_*_<前缀>*
  python3 harness/analyze_plan_audit.py v8a v7ctl      # 多批对比
阈值：CROSS_M 默认 0.35 m（< 车宽 0.47 的一半余量，必然穿障）；
      START_SKIP 默认跳过前 2 个采样点（起点附近 0.1 m）再判"中途"。
"""
import glob
import os
import re
import sys

ROOT = '/home/kbf/stemm_exploration_sim_ws'
RE_NEW = re.compile(
    r'计划窄处体检：/plan (\d+) 点，最小净宽 ([\d.]+) m@#(\d+)\(([-\d.]+),([-\d.]+)\)，'
    r'起点净宽 ([-\d.]+) m，最长窄段 ([\d.]+) m')
RE_OLD = re.compile(r'计划窄处体检：/plan (\d+) 点，最小净宽 ([\d.]+) m，最长窄段 ([\d.]+) m')


def scan(path, cross_m=0.35):
    rows = []
    try:
        fh = open(path, encoding='utf-8', errors='replace')
    except FileNotFoundError:
        return rows
    with fh:
        for line in fh:
            m = RE_NEW.search(line)
            if m:
                n, mw, idx, x, y, fw, run = m.groups()
                rows.append({'n': int(n), 'min_w': float(mw), 'idx': int(idx),
                             'xy': (float(x), float(y)), 'first_w': float(fw),
                             'run': float(run), 'fmt': 'new'})
                continue
            m = RE_OLD.search(line)
            if m:
                n, mw, run = m.groups()
                rows.append({'n': int(n), 'min_w': float(mw), 'idx': None,
                             'xy': None, 'first_w': None, 'run': float(run),
                             'fmt': 'old'})
    return rows


def summarize(pre, cross_m=0.35, start_skip=2):
    dirs = sorted(glob.glob(os.path.join(ROOT, 'logs', 'round_*_' + pre + '*')))
    tot = cross = startcase = mid = 0
    worst = []
    per = []
    for d in dirs:
        rows = scan(os.path.join(d, 'launch.log'), cross_m)
        if not rows:
            continue
        c = sum(1 for r in rows if r['min_w'] < cross_m)
        s = sum(1 for r in rows if r['idx'] is not None and r['idx'] <= start_skip
                and r['min_w'] < cross_m)
        md = sum(1 for r in rows
                 if r['min_w'] < cross_m and (r['idx'] is None or r['idx'] > start_skip))
        tot += len(rows)
        cross += c
        startcase += s
        mid += md
        per.append((os.path.basename(d), len(rows), c, s, md))
        for r in rows:
            if r['min_w'] < cross_m:
                worst.append((r['min_w'], os.path.basename(d), r['idx'], r['xy']))
    print('== 批次 %s：%d 轮有体检数据，共 %d 条计划' % (pre, len(per), tot))
    for name, n, c, s, md in per:
        print('   %-38s 计划 %3d 条，<%.2fm %3d 条（起点 %d / 中途 %d）'
              % (name[:38], n, cross_m, c, s, md))
    if tot:
        print('   合计：<%.2f m 占 %d/%d（起点型 %d、中途型 %d）'
              % (cross_m, cross, tot, startcase, mid))
    worst.sort()
    if worst:
        print('   最小净宽最差的 5 条：' + '; '.join(
            '%0.2f@%s#%s' % (w[0], w[1][-14:], w[2]) for w in worst[:5]))
    return {'plans': tot, 'cross': cross, 'start': startcase, 'mid': mid}


def main():
    prefixes = sys.argv[1:] or ['v8a']
    out = {}
    for p in prefixes:
        out[p] = summarize(p)
    if len(prefixes) >= 2:
        print('\n== 组间对照（中途型穿障计划） ==')
        for p, v in out.items():
            print('   %-8s 计划 %d，中途型 %d（%.1f%%），起点型 %d'
                  % (p, v['plans'], v['mid'], 100.0 * v['mid'] / max(1, v['plans']), v['start']))
    return 0


if __name__ == '__main__':
    sys.exit(main())
