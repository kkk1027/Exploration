#!/usr/bin/env python3
"""report.py —— 单轮"体检报告"（一条命令出结论，替代手工 grep）。

用法：report.py <run_dir> <tag> [--context 20]
产物：<run_dir>/report.md（同时把摘要打到 stdout）

内容：
  1) 结论：八项验收（复用 evaluate.py 的判据）
  2) 时间分布：启动 / 探索 / 其中 导航中·选点等待·恢复态 / 返航（严格按事件配对，不重复计数）
  3) 撞障与后退取证：物理接触(底盘/轮)、雷达近距点、受控倒车、非受控硬倒车
  4) 脱困监督事件与仲裁统计
  5) 异常上下文：每次接触/硬倒车前后各 N 行 launch.log（按墙钟对齐）
"""
import json
import os
import re
import sys

TS = re.compile(r'\[(\d{10}\.\d+)\]')


def load_jsonl(path):
    out = []
    if not os.path.exists(path):
        return out
    for line in open(path, encoding='utf-8', errors='ignore'):
        line = line.strip()
        if not line:
            continue
        try:
            out.append(json.loads(line))
        except Exception:
            pass
    return out


MGR = 'stemm_cartographer_nav2_manager'


def launch_events(path):
    """从 launch.log 抽 (墙钟, 事件名, 原文)。

    只用 manager 自己打的"目标生命周期"行，避免 bt_navigator / controller_server 的
    同义日志把同一个终态重复计数（第一版就踩了这个坑，导航中一度算到 112.7%）。
    """
    ev = []
    if not os.path.exists(path):
        return ev
    for line in open(path, encoding='utf-8', errors='ignore'):
        m = TS.search(line)
        if not m:
            continue
        t = float(m.group(1))
        low = line.lower()
        is_mgr = MGR in line
        if 'ready_guard_timing' in line:
            ev.append((t, 'ready', line.strip()))
        elif is_mgr and 'navigation goal dispatched' in line:
            ev.append((t, 'dispatch', line.strip()))
        elif is_mgr and 'navigation goal reached' in line:
            ev.append((t, 'succ', line.strip()))
        elif is_mgr and 'navigation failed with status' in line:
            ev.append((t, 'fail', line.strip()))
        elif is_mgr and 'navigation goal canceled' in line:
            ev.append((t, 'cancel', line.strip()))
        elif is_mgr and 'mapping done acknowledged' in line:
            ev.append((t, 'done', line.strip()))
        elif is_mgr and 'return-home completed' in line:
            ev.append((t, 'home', line.strip()))
        elif is_mgr and 'front blocked' in low:
            ev.append((t, 'frontblocked', line.strip()))
        elif is_mgr and 'no net progress' in low:
            ev.append((t, 'noprogress', line.strip()))
    ev.sort()
    return ev


def first_log_ts(path):
    if not os.path.exists(path):
        return None
    for line in open(path, encoding='utf-8', errors='ignore'):
        m = TS.search(line)
        if m:
            return float(m.group(1))
    return None


def time_breakdown(ev, log_first=None):
    """严格配对 + 区间并集：每个 dispatch 只与它之后的第一个终态配对，重叠区间合并，
    保证"导航中 + 选点等待 <= 探索总时长"（第一版按总和计，出现过 112.7% 的荒唐值）。
    """
    disp = [i for i, (_t, k, _l) in enumerate(ev) if k == 'dispatch']
    used = set()
    intervals = []
    last_term_t = None
    first_disp_t = None
    for i in disp:
        t0 = ev[i][0]
        if first_disp_t is None:
            first_disp_t = t0
        end = None
        for j in range(i + 1, len(ev)):
            if ev[j][1] in ('succ', 'fail', 'cancel') and j not in used:
                end = ev[j]
                used.add(j)
                break
        if end is not None:
            intervals.append((t0, end[0]))
            last_term_t = end[0]
    # 区间并集
    nav = 0.0
    for a, b in sorted(intervals):
        if b <= a:
            continue
        nav += (b - a) - max(0.0, min(b, (nav and 0) or b) and 0)  # placeholder
    nav = 0.0
    cur_a = cur_b = None
    for a, b in sorted(intervals):
        if cur_a is None:
            cur_a, cur_b = a, b
        elif a <= cur_b:
            cur_b = max(cur_b, b)
        else:
            nav += cur_b - cur_a
            cur_a, cur_b = a, b
    if cur_a is not None:
        nav += cur_b - cur_a
    done = next((t for t, k, _l in ev if k == 'done'), None)
    home = next((t for t, k, _l in ev if k == 'home'), None)
    ready = next((t for t, k, _l in ev if k == 'ready'), None)
    launched = log_first
    span = (done - first_disp_t) if (done and first_disp_t) else None
    wait = max(0.0, span - nav) if (span is not None) else None
    return {
        'startup_s': (ready - launched) if (ready and launched) else None,
        'explore_s': span,
        'nav_s': nav,
        'select_wait_s': wait,
        'return_s': (home - done) if (done and home) else None,
        'dispatch_n': len(disp),
        'paired_n': len(intervals),
        'succ_n': sum(1 for _t, k, _l in ev if k == 'succ'),
        'fail_n': sum(1 for _t, k, _l in ev if k == 'fail'),
        'cancel_n': sum(1 for _t, k, _l in ev if k == 'cancel'),
        'front_blocked_n': sum(1 for _t, k, _l in ev if k == 'frontblocked'),
        'no_progress_n': sum(1 for _t, k, _l in ev if k == 'noprogress'),
    }


def probe_summary(probe):
    last = {}
    events = []
    for r in probe:
        if r.get('type') == 'summary':
            last = r
        elif r.get('type') in ('contact', 'graze', 'alert', 'reverse', 'stall',
                               'giveup', 'no_direction'):
            events.append(r)
    return last, events


def supervisor_summary(sup):
    stats = {'unit_start': 0, 'unit_end': 0, 'unit_abort': 0, 'no_direction': 0,
             'giveup_40s': 0, 'reverse_assist_start': 0, 'reverse_assist_end': 0,
             'reverse_assist_refused': 0}
    for r in sup:
        k = r.get('type')
        if k in stats:
            stats[k] += 1
    return stats


def arbiter_stats(launch_log):
    last = None
    for line in open(launch_log, encoding='utf-8', errors='ignore'):
        if 'frontier_arbiter' in line and '统计' in line:
            m = re.search(r"统计 (\{.*\})\s*$", line.strip())
            if m:
                try:
                    last = eval(m.group(1), {'__builtins__': {}}, {})
                except Exception:
                    pass
    return last


def fmt(v, unit='s', nd=1):
    return '—' if v is None else f'{v:.{nd}f} {unit}'


def pct(a, b):
    if not a or not b:
        return '—'
    return f'{a / b * 100:.1f}%'


def context_lines(launch_log, wall_ts, n=20):
    """取某墙钟时刻前后 n 行（launch.log 是时间有序的，按最近时间戳定位）。"""
    if not os.path.exists(launch_log):
        return []
    lines = open(launch_log, encoding='utf-8', errors='ignore').read().splitlines()
    best = 0
    for i, line in enumerate(lines):
        m = TS.search(line)
        if m and abs(float(m.group(1)) - wall_ts) < 1.0:
            best = i
            break
    lo = max(0, best - n)
    return lines[lo:best + n]


def main():
    run_dir = sys.argv[1] if len(sys.argv) > 1 else '.'
    tag = sys.argv[2] if len(sys.argv) > 2 else os.path.basename(run_dir.rstrip('/'))
    n_ctx = 20
    if '--context' in sys.argv:
        n_ctx = int(sys.argv[sys.argv.index('--context') + 1])

    launch_log = os.path.join(run_dir, 'launch.log')
    probe = load_jsonl(os.path.join(run_dir, 'probe.jsonl'))
    sup = load_jsonl(os.path.join(run_dir, 'supervisor.jsonl'))

    ev = launch_events(launch_log)
    tb = time_breakdown(ev, first_log_ts(launch_log))
    p_last, p_events = probe_summary(probe)
    s_stats = supervisor_summary(sup)
    arb = arbiter_stats(launch_log)

    physical = (p_last.get('chassis_contacts', 0) + p_last.get('wheel_contacts', 0))
    hard_rev = [e for e in p_events
                if e.get('type') == 'reverse' and e.get('phase') == 'end'
                and not e.get('assist') and (e.get('dist') or 0) >= 0.05]
    assist_rev = [e for e in p_events
                  if e.get('type') == 'reverse' and e.get('phase') == 'end'
                  and e.get('assist')]

    verdict = 'PASS'
    fails = []
    if tb['dispatch_n'] == 0:
        verdict, _ = 'FAIL', fails.append('无目标派发')
    if tb['return_s'] is None:
        verdict = 'FAIL'
        fails.append('返航未完成')
    if physical:
        verdict = 'FAIL'
        fails.append(f'物理接触 {physical}')
    if hard_rev:
        verdict = 'FAIL'
        fails.append(f'非受控硬倒车 {len(hard_rev)}')

    out = []
    out.append(f'# 单轮体检报告 · {tag}\n')
    out.append(f'**结论：{verdict}**' + (f"（{'；'.join(fails)}）" if fails else '（全项通过）') + '\n')

    out.append('## 1. 时间分布\n')
    out.append('| 项 | 值 |')
    out.append('| --- | --- |')
    out.append(f"| 启动（launch→Nav2 就绪） | {fmt(tb['startup_s'])} |")
    out.append(f"| 探索（首目标→结束建图） | {fmt(tb['explore_s'])} |")
    out.append(f"| 其中 导航中（派发→终态） | {fmt(tb['nav_s'])}（{pct(tb['nav_s'], tb['explore_s'])}）|")
    out.append(f"| 其中 选点等待（终态→下次派发） | {fmt(tb['select_wait_s'])}（{pct(tb['select_wait_s'], tb['explore_s'])}）|")
    out.append(f"| 返航（结束建图→返航完成） | {fmt(tb['return_s'])} |")
    out.append(f"| 目标派发/成功/失败/取消 | {tb['dispatch_n']} / {tb['succ_n']} / {tb['fail_n']} / {tb['cancel_n']} |")
    out.append(f"| front blocked / 无净进展 事件 | {tb['front_blocked_n']} / {tb['no_progress_n']} |")

    out.append('\n## 2. 撞障与后退取证\n')
    out.append('| 项 | 值 |')
    out.append('| --- | --- |')
    out.append(f"| 物理接触（底盘/轮） | {p_last.get('chassis_contacts', 0)} / {p_last.get('wheel_contacts', 0)} |")
    out.append(f"| 受控倒车（仅返航，允许） | {len(assist_rev)} 次 / "
               f"{sum((e.get('dist') or 0) for e in assist_rev):.3f} m |")
    out.append(f"| **非受控硬倒车（必须 0）** | **{len(hard_rev)} 次 / "
               f"{sum((e.get('dist') or 0) for e in hard_rev):.3f} m** |")
    out.append(f"| 雷达近距点 contact/graze/alert | {p_last.get('contacts', 0)} / "
               f"{p_last.get('grazes', 0)} / {p_last.get('alerts', 0)} |")
    out.append(f"| 全程最小车体间隙 | {fmt(p_last.get('min_d_ever'), 'm', 3)} |")

    out.append('\n## 3. 脱困监督\n')
    out.append('| 项 | 次数 |')
    out.append('| --- | --- |')
    for k in ('unit_start', 'unit_end', 'unit_abort', 'no_direction', 'giveup_40s',
              'reverse_assist_start', 'reverse_assist_refused'):
        out.append(f'| {k} | {s_stats.get(k, 0)} |')

    if arb:
        out.append('\n## 4. 仲裁统计\n')
        out.append('```\n' + json.dumps(arb, ensure_ascii=False, indent=1) + '\n```')

    out.append('\n## 5. 异常上下文\n')
    interesting = [e for e in p_events if e.get('type') in ('contact', 'stall', 'giveup')]
    interesting += hard_rev[:5] + assist_rev[:5]
    if not interesting:
        out.append('（本轮无接触/停滞/倒车事件）')
    for e in interesting[:6]:
        wall = e.get('wall')
        out.append(f"\n### {e.get('type')} @ {e.get('pos')} (wall={wall})\n")
        if wall:
            for line in context_lines(launch_log, wall, n_ctx):
                out.append('    ' + line[:200])

    text = '\n'.join(out)
    with open(os.path.join(run_dir, 'report.md'), 'w', encoding='utf-8') as fh:
        fh.write(text + '\n')
    print(text)
    return 0 if verdict == 'PASS' else 1


if __name__ == '__main__':
    sys.exit(main())
