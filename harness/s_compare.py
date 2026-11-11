#!/usr/bin/env python3
"""对比"旧代码"与"新代码"（S1~S3）的探索行为差异。

用法：
  python3 harness/s_compare.py <旧轮 glob 前缀> <新轮 glob 前缀> [...]

默认对比：
  python3 harness/s_compare.py p13e1_a p13e1_b      # 基线两组
  python3 harness/s_compare.py p13e1_a s             # 基线 A 组 vs 本次 s* 组

每个组是一组 logs/round_*_<组名>* 目录。只统计有效轮（有派发）。
指标与判定：
  wait_clear_pct   前方净空>1.2m 却静止的探索时间占比（越低越好）
  waits>=5s        这类滞留片段数与最长时长
  narrow_hard      走廊硬门否决次数/轮（应大幅下降）
  published        仲裁器发出目标数/轮（应上升）
  dispatch         实际派发数/轮（应上升）
  goal_life_med    目标"派发→取消"存活时长中位（应上升）
  cancel_mig       取消中紧跟"后置迁移"的占比（应下降，需 S4 生效）
  known_cells      末态已知栅格数（覆盖面）
"""
import glob
import json
import math
import os
import re
import statistics as st
import sys


ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

TS_RE = re.compile(r'^\[[^]]*\]\s*\[?[A-Z]*\]?\s*\[([0-9.]+)\]')
DISP_RE = re.compile(r'navigation goal dispatched')
CANC_RE = re.compile(
    r'navigation goal canceled; goal=\(([-\d.]+), ([-\d.]+).*current=\(([-\d.]+), ([-\d.]+)')
MIG_RE = re.compile(r'V2-③ 后置迁移')
REJ_RE = re.compile(r'本轮候选全部被拒：(\{.*?\})')


def _dirs(prefix):
    """按真实路径去重（logs/ 下可能有同名软链）。"""
    out, seen = [], set()
    for d in sorted(glob.glob(os.path.join(ROOT, 'logs', 'round_*_%s*' % prefix))):
        rp = os.path.realpath(d)
        if rp in seen:
            continue
        seen.add(rp)
        out.append(d)
    return out


def _round(d):
    """采集单轮指标；无效轮返回 None。"""
    lp = os.path.join(d, 'launch.log')
    if not os.path.exists(lp):
        return None
    txt = open(lp, encoding='utf-8', errors='ignore').read()

    # ---- 有效轮判定：派发 >= 3 次（1~2 次的多为启动失败/提前收尾，指标不可比）----
    dispatch = len(DISP_RE.findall(txt))
    if dispatch < 3:
        return None

    # ---- 取消与存活时长 ----
    events = []
    for line in txt.splitlines():
        m = TS_RE.match(line)
        if not m:
            continue
        t = float(m.group(1))
        if DISP_RE.search(line):
            events.append((t, 'disp'))
        elif MIG_RE.search(line):
            events.append((t, 'mig'))
        elif CANC_RE.search(line):
            events.append((t, 'canc'))
    lives, n_canc, n_mig_in_canc = [], 0, 0
    last_disp, mig_win = None, []
    for t, kind in events:
        mig_win = [x for x in mig_win if t - x < 1.5]
        if kind == 'disp':
            last_disp = t
        elif kind == 'mig':
            mig_win.append(t)
        else:
            n_canc += 1
            if last_disp is not None:
                lives.append(t - last_disp)
            if mig_win:
                n_mig_in_canc += 1

    # ---- 否决计数（取最后一次累计快照）----
    narrow_hard = published = None
    m_all = REJ_RE.findall(txt)
    if m_all:
        kv = {k: int(v) for k, v in
              re.findall(r"'([a-z_]+)':\s*(\d+)", m_all[-1])}
        narrow_hard = kv.get('reject_narrow_hard')
        published = kv.get('published')

    # ---- 逐秒时间线：滞留与覆盖 ----
    wait_sec = tot_sec = 0
    wait_runs, run = [], 0
    known_cells = None
    inplace_n = migrate_n = last_inplace = last_migrate = 0
    tl = os.path.join(d, 'timeline.jsonl')
    if os.path.exists(tl):
        for line in open(tl, encoding='utf-8', errors='ignore'):
            line = line.strip()
            if not line:
                continue
            try:
                r = json.loads(line)
            except Exception:                       # noqa: BLE001
                continue
            if r.get('mode') in ('returning_home', 'finalizing_map'):
                break
            cmd = r.get('cmd') or [0, 0]
            v = math.hypot(cmd[0], cmd[1]) if len(cmd) >= 2 else 0.0
            front = r.get('front')
            tot_sec += 1
            if front is not None and front > 1.2 and v < 0.02:
                wait_sec += 1
                run += 1
            else:
                if run >= 5:
                    wait_runs.append(run)
                run = 0
            if r.get('known_cells') is not None:
                known_cells = r['known_cells']
            # S4：timeline 里是累计值，取最后一次读到的即可
            if r.get('veto_inplace') is not None:
                last_inplace = r['veto_inplace']
            if r.get('veto_migrate') is not None:
                last_migrate = r['veto_migrate']
        if run >= 5:
            wait_runs.append(run)
    inplace_n, migrate_n = last_inplace, last_migrate

    # ---- 安全指标（从 report.md 提取；缺失记 None）----
    def _rep(pat):
        rp = os.path.join(d, 'report.md')
        if not os.path.exists(rp):
            return None
        m = re.search(pat, open(rp, encoding='utf-8', errors='ignore').read())
        return int(m.group(1)) if m else None

    contact = _rep(r'物理接触（底盘/轮） \| ([0-9]+)')
    hard_rev = _rep(r'非受控硬倒车（必须 0） \| \*{0,2}([0-9]+)')
    giveup = _rep(r'giveup_40s \| ([0-9]+)')
    soft_pass = None
    if m_all:
        kv2 = {k: int(v) for k, v in
               re.findall(r"'([a-z_]+)':\s*(\d+)", m_all[-1])}
        soft_pass = kv2.get('pass_narrow_soft')

    # ---- 覆盖与效率（cand_metrics.json，由 analyze_dispatch_risk 生成）----
    cm = os.path.join(d, 'cand_metrics.json')
    known_m2 = free_m2 = traverse_m = None
    gain_per_m = None
    if os.path.exists(cm):
        try:
            _cm = json.load(open(cm, encoding='utf-8'))
            known_m2 = _cm.get('G1_known_m2')
            free_m2 = _cm.get('G2_free_m2')
            traverse_m = _cm.get('G3_traverse_m')
            gain_per_m = _cm.get('G4_gain_per_m')
        except Exception:                       # noqa: BLE001
            pass

    return dict(
        name=os.path.basename(d),
        dispatch=dispatch,
        cancel=n_canc,
        cancel_mig_pct=100.0 * n_mig_in_canc / max(1, n_canc),
        life_med=st.median(lives) if lives else None,
        narrow_hard=narrow_hard,
        soft_pass=soft_pass,
        published=published,
        wait_pct=100.0 * wait_sec / max(1, tot_sec),
        wait_n=len(wait_runs),
        wait_max=max(wait_runs) if wait_runs else 0,
        known_cells=known_m2,
        free_m2=free_m2,
        traverse_m=traverse_m,
        gain_per_m=gain_per_m,
        contact=contact,
        hard_rev=hard_rev,
        giveup=giveup,
        inplace=inplace_n,
        migrate=migrate_n,
        explore_sec=tot_sec,
    )


def _agg(rows, key):
    vals = [r[key] for r in rows if r[key] is not None]
    return (st.median(vals) if vals else None), len(vals)


def main():
    if len(sys.argv) < 3:
        print(__doc__)
        return 2
    groups = sys.argv[1:]
    data = {}
    for g in groups:
        rows = []
        for d in _dirs(g):
            r = _round(d)
            if r:
                rows.append(r)
        data[g] = rows

    keys = [
        ('dispatch', '派发数/轮', 1),
        ('published', '仲裁器发出/轮', 1),
        ('narrow_hard', '走廊硬门否决/轮', 1),
        ('soft_pass', '窄但放行/轮', 1),
        ('wait_pct', '等待占比 %', 1),
        ('wait_n', '等待片段/轮', 1),
        ('wait_max', '最长等待 s', 1),
        ('life_med', '目标存活中位 s', 1),
        ('cancel', '取消次数/轮', 1),
        ('cancel_mig_pct', '取消由迁移引起 %', 1),
        ('inplace', '原地改向/轮(S4)', 1),
        ('migrate', '取消重派/轮', 1),
        ('known_cells', '已知面积 m2', 1),
        ('free_m2', '自由面积 m2', 1),
        ('traverse_m', '行驶里程 m', 1),
        ('gain_per_m', '覆盖/里程', 2),
        ('contact', '物理接触/轮', 2),
        ('hard_rev', '非受控硬倒车/轮', 2),
        ('giveup', 'giveup_40s/轮', 1),
        ('explore_sec', '探索时长 s', 0),
    ]

    w = max(len(v) for _, v, _ in keys)
    hdr = '  '.join('%-14s' % g for g in groups)
    print('%s  %-16s  %s   %s' % ('指标'.ljust(w), '', hdr, '有效轮'))
    print('-' * (w + 24 + 16 * len(groups)))
    for k, label, nd in keys:
        cells = []
        for g in groups:
            med, n = _agg(data[g], k)
            cells.append('%-14s' % ('—' if med is None else
                                    ('%.*f' % (nd, med))))
        ns = ', '.join(str(len(data[g])) for g in groups)
        print('%-*s  %-22s  %s' % (w, label, '  '.join(cells), ns))

    print()
    for g in groups:
        rows = data[g]
        print('[%s] 有效轮 %d（无效轮已剔除）' % (g, len(rows)))
        if not rows:
            continue
        # 逐轮明细，便于发现异常轮
        for r in rows[:24]:
            print('  %-34s 派发%3d 等待%5.1f%% 最长%3ds 存活%s 硬门%s' % (
                r['name'][-34:], r['dispatch'], r['wait_pct'], r['wait_max'],
                '—' if r['life_med'] is None else '%.1fs' % r['life_med'],
                r['narrow_hard']))
        if len(rows) > 24:
            print('  ...（仅显示前 24 轮）')
    return 0


if __name__ == '__main__':
    sys.exit(main())
