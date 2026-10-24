#!/usr/bin/env python3
"""compare_rounds.py —— 多组轮次的同口径对比（V0/V1/V2 这种 A/B 用）。

用法:
  python3 harness/compare_rounds.py cand g1r v2r            # 按 tag 前缀分组
  python3 harness/compare_rounds.py --last 40 cand v2r      # 只看最近 N 轮目录

每组输出（口径与 metrics.py / evaluate.py 保持一致）：
  有效轮/总数、PASS 率、返航失败、物理接触、静默静止、起栈失败、
  派发来源分布（arbiter / rrt / manager）、静默卡死次数、
  仲裁闸门与保底动作计数（gate_reject / fallback_obs / narrow_fallback）

关键点：`source=exploration_manager` 是"manager 绕过仲裁自己采样"的那条路径，
V2 的目标就是压低它的占比 —— 所以这个分布是主指标之一。
"""
import argparse
import glob
import json
import os
import re
import statistics

WS = '/home/kbf/stemm_exploration_sim_ws'
SRC = re.compile(r'dispatched: source=([a-z_]+)')
# 与 metrics.py 完全同口径：看门狗的 '静止超时'
STALL = re.compile(r'静止超时')
STATS = re.compile(r'统计 (\{[^}]*\})')
# 更直接的"卡死"指标：规划失败次数、候选全灭次数（比"归因率"这种比值敏感得多）
PLAN_FAIL = re.compile(r'failed to create plan')
NO_CAND = re.compile(r'本轮候选全部被拒')


def load_probe(d):
    """物理接触以 evaluate.json 的 collision.physical_contacts 为准（与验收同口径）。

    probe.jsonl 的 summary 是"雷达告警级别"的计数（contact/graze/alert 三档），
    contact>0 并不等于车体真碰上（graze 是擦边告警），曾因此把 10 轮全判成接触。
    """
    p = os.path.join(d, 'evaluate.json')
    if os.path.exists(p):
        try:
            o = json.load(open(p, encoding='utf-8'))
        except Exception:                # noqa: BLE001
            o = {}
        c = o.get('collision') or {}
        if 'physical_contacts' in c:
            return {'contacts': c['physical_contacts'],
                    'chassis': c.get('chassis_contacts', 0),
                    'wheel': c.get('wheel_contacts', 0)}
    return {}


def load_verdict(d):
    """以 evaluate.json 为准（evaluate.py 的十项判据），缺文件才回落到 report.md。"""
    p = os.path.join(d, 'evaluate.json')
    if os.path.exists(p):
        try:
            o = json.load(open(p, encoding='utf-8'))
        except Exception:                # noqa: BLE001
            o = {}
        v = o.get('verdict') or 'NA'
        fails = [k for k, ok in (o.get('checks') or {}).items() if ok is False]
        return v, fails
    p = os.path.join(d, 'report.md')
    if not os.path.exists(p):
        return 'NA', []
    txt = open(p, encoding='utf-8', errors='ignore').read()
    m = re.search(r'verdict[=:]\s*(\w+)', txt)
    return (m.group(1) if m else 'NA'), []


def load_launch(d):
    p = os.path.join(d, 'launch.log')
    if not os.path.exists(p):
        return {'src': {}, 'stalls': 0, 'stats': {},
                'ready': False, 'dispatched': False}
    src = {}
    stalls = 0
    stats = {}
    plan_fail = 0
    no_cand = 0
    for line in open(p, encoding='utf-8', errors='ignore'):
        for m in SRC.finditer(line):
            src[m.group(1)] = src.get(m.group(1), 0) + 1
        if STALL.search(line):
            stalls += 1
        if PLAN_FAIL.search(line):
            plan_fail += 1
        if NO_CAND.search(line):
            no_cand += 1
        for m in STATS.finditer(line):
            try:
                o = json.loads(m.group(1).replace("'", '"'))
            except Exception:            # noqa: BLE001
                continue
            for k, v in o.items():       # 统计是累计值，取最大即可
                if isinstance(v, (int, float)):
                    stats[k] = max(stats.get(k, 0), int(v))
    txt = open(p, encoding='utf-8', errors='ignore').read()
    ready = 'ready_guard_timing' in txt
    dispatched = 'navigation goal dispatched' in txt
    return {'src': src, 'stalls': stalls, 'stats': stats,
            'ready': ready, 'dispatched': dispatched,
            'plan_fail': plan_fail, 'no_cand': no_cand}


def rounds_for(prefix):
    """不合并重试：每一次尝试都是一个真实轮次（重试本身也是"上一轮没过"的证据）。

    同 tag 出现多个目录时按目录名（含启动时刻）自然排序，两个都算。
    """
    out = []
    for d in sorted(glob.glob(os.path.join(WS, 'logs', 'round_*'))):
        base = os.path.basename(d)
        tag = base.split('_', 2)[-1]
        if tag.startswith(prefix) and os.path.isdir(d):
            out.append((tag, d))
    return out


def summarize(prefix, rows):
    n = len(rows)
    # 有效轮 = 起栈成功 且 派发过目标（与 metrics.py 同口径）
    eff = [r for r in rows if r['ready'] and r['dispatched']]
    ok = [r for r in eff if r['verdict'] == 'PASS']
    ret = [r for r in eff if 'return' in ' '.join(r['fails']).lower()]
    contact = [r for r in eff if r['contact']]
    quiet = [r for r in eff if r['stalls'] > 0 or r['quiet']]
    src = {}
    for r in rows:
        for k, v in r['src'].items():
            src[k] = src.get(k, 0) + v
    tot_src = sum(src.values()) or 1
    st = [r['stalls'] for r in rows if r['stalls'] is not None]
    pf = [r.get('plan_fail', 0) for r in rows]
    nc = [r.get('no_cand', 0) for r in rows]
    gates = [r['stats'].get('gate_reject', 0) for r in rows]
    fb = [r['stats'].get('fallback_obs', 0) for r in rows]
    print(f'\n===== {prefix}（{n} 轮，有效 {len(eff)}） =====')
    print(f'  全绿率      : {len(ok)}/{len(eff)} = '
          f'{100*len(ok)/max(1,len(eff)):.0f}%')
    print(f'  返航失败    : {len(ret)}/{len(eff)}')
    print(f'  物理接触    : {len(contact)}/{len(eff)}')
    print(f'  静默静止    : {len(quiet)}/{len(eff)}')
    print(f'  起栈失败    : {sum(1 for r in rows if not (r["ready"] and r["dispatched"]))}/{n}')
    print(f'  派发来源    : ' + '  '.join(
        f'{k}={v}({100*v/tot_src:.0f}%)' for k, v in
        sorted(src.items(), key=lambda x: -x[1])))
    if st:
        print(f'  卡死次数    : 合计 {sum(st)}，中位 {statistics.median(st):.1f}/轮')
    print(f'  规划失败    : 合计 {sum(pf)}（中位 {statistics.median(pf) if pf else 0:.0f}/轮）')
    print(f'  候选全灭    : 合计 {sum(nc)}（中位 {statistics.median(nc) if nc else 0:.0f}/轮）')
    print(f'  闸门拒绝    : 合计 {sum(gates)}（中位 {statistics.median(gates) if gates else 0:.0f}/轮）')
    print(f'  保底观察点  : 合计 {sum(fb)}')
    return {'n': n, 'eff': len(eff), 'ok': len(ok), 'ret': len(ret),
            'plan_fail': sum(pf), 'no_cand': sum(nc),
            'contact': len(contact), 'quiet': len(quiet), 'src': src,
            'stalls': sum(st) if st else 0, 'gate': sum(gates), 'fb': sum(fb)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('prefixes', nargs='+')
    ap.add_argument('--last', type=int, default=0)
    a = ap.parse_args()
    allrows = {}
    for p in a.prefixes:
        rows = []
        for tag, d in rounds_for(p):
            pr = load_probe(d)
            v, fails = load_verdict(d)
            lg = load_launch(d)
            rows.append({'tag': tag, 'dir': d, 'verdict': v, 'fails': fails,
                         'plan_fail': lg.get('plan_fail', 0), 'no_cand': lg.get('no_cand', 0),
                         'contact': (pr.get('contacts', 0) or 0) > 0,
                         'quiet': False, 'stalls': lg['stalls'],
                         'ready': lg['ready'], 'dispatched': lg['dispatched'],
                         'src': lg['src'], 'stats': lg['stats']})
        allrows[p] = rows
    saved = {}
    for p in a.prefixes:
        saved[p] = summarize(p, allrows[p])
    # 关键对比
    print('\n===== 关键差异（以第一组为基准） =====')
    base = a.prefixes[0]
    b = saved[base]
    for p in a.prefixes[1:]:
        c = saved[p]
        def pct(grp):
            return 100.0 * grp['ok'] / max(1, grp['eff'])
        mgr = lambda g: 100.0 * g['src'].get('exploration_manager', 0) / max(1, sum(g['src'].values()))
        print(f'  {p} vs {base}: 全绿 {pct(b):.0f}%→{pct(c):.0f}%'
              f' | manager占比 {mgr(b):.0f}%→{mgr(c):.0f}%'
              f' | 卡死 {b["stalls"]}→{c["stalls"]}'
              f' | 返航失败 {b["ret"]}→{c["ret"]}'
              f' | 接触 {b["contact"]}→{c["contact"]}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
