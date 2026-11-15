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
        elif (is_mgr and 'mapping done acknowledged' in line) \
                or '探索限时到（' in line \
                or '结束探索（' in line \
                or ('SetBool_Response' in line and 'success=True' in line):
            # 结束建图的"起点"。四个来源都要认：
            #   1) manager 自己的确认行
            #   2) 定时器终止：会话驱动打印"探索限时到（"
            #   3) 覆盖率终止：会话驱动打印"结束探索（…
            #      ★ 2026-11-15 修：P2 把 coverage_terminate 默认打开后，
            #      "探索限时到（"不再出现，这里没跟上 ⇒ done 缺失 ⇒
            #      return_s=None ⇒ 误报"返航未完成"。实测当天 19 轮全部
            #      被误判，是历史单日最高。
            #   4) set_mapping_done 的返回值（节点名是 session_driver，
            #      不是 manager，所以上面的 is_mgr 拦不住）
            ev.append((t, 'done', line.strip()))
        elif is_mgr and ('return-home completed' in line
                         or 'return-home goal reached' in line):
            ev.append((t, 'home', line.strip()))
        elif re.search(r'返航(?:到位|完成)', line):
            # ★ 2026-11-15：会话驱动的返航终点有两种措辞，都必须认：
            #   「=== 返航完成（manager 状态：…」—— manager 走到最终态
            #   「=== 返航到位：距 map 原点 X m，返航用时 Y s」—— 到点判定
            # 正则与 evaluate.py 的 POS 保持一致（r'返航(?:到位|完成)'），
            # 否则两个工具的 return_home 会再次分歧。
            # 早于 manager 自报 goal reached 的轮次（2026-10 期）只有这两句，
            # 之前漏认导致 return_s=None、报告里返航耗时显示"—"。
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
    """严格配对 + 区间并集 + 区间裁剪。

    * 每个 dispatch 只与它之后的第一个终态配对（第一版按总和计，出现过 112.7% 的荒唐值）；
    * 重叠区间合并；
    * 区间裁到 [首派发, 结束建图/返航完成]，保证 导航中 + 选点等待 == 探索总时长。
    """
    done = next((t for t, k, _l in ev if k == 'done'), None)
    home_t = next((t for t, k, _l in ev if k == 'home'), None)
    ready = next((t for t, k, _l in ev if k == 'ready'), None)

    disp = [i for i, (_t, k, _l) in enumerate(ev) if k == 'dispatch']
    used = set()
    intervals = []
    first_disp_t = None
    for i in disp:
        t0 = ev[i][0]
        if first_disp_t is None:
            first_disp_t = t0
        for j in range(i + 1, len(ev)):
            if ev[j][1] in ('succ', 'fail', 'cancel') and j not in used:
                used.add(j)
                intervals.append((t0, ev[j][0]))
                break

    end_cap = done if done is not None else home_t
    if end_cap is not None:
        intervals = [(a, min(b, end_cap)) for a, b in intervals if a < end_cap and b > a]

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

    span = (done - first_disp_t) if (done and first_disp_t) else None
    wait = max(0.0, span - nav) if (span is not None) else None
    return {
        'startup_s': (ready - log_first) if (ready and log_first) else None,
        'explore_s': span,
        'nav_s': nav,
        'select_wait_s': wait,
        'return_s': (home_t - done) if (done and home_t) else None,
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


def load_evaluate(run_dir):
    """读 evaluate.json —— 结论的**唯一权威源**。

    ★ 2026-11-15：本文件原来自行实现一套判据（看 `assist` 真假、看 return_s
    是否为 None），与 evaluate.py 的两处差异造成**166 / 955 轮（17.4%）结论相反**：

      1) 硬倒车：旧代码只看 `assist == False and dist >= 0.05`，
         而 evaluate.py 有三级授权包络（explore_ok / return_ok / mppi_ok）。
         后果：0.065 m 的受控探索期小步倒车被当成\"非受控\" ⇒ 126 轮误判。
      2) 返航：旧代码看 `return_s`（home 时间戳 - done 时间戳），
         而 done 事件在覆盖率终止后识别不到 ⇒ 50 轮误报\"返航未完成\"。

    现在：evaluate.json 存在就以它为准，本文件不再重算结论；
    只有在 evaluate.json 缺失时（例如单独跑单个目录）才回退到内置逻辑，
    并在报告里明确标注\"回退口径\"，避免又一次无声分歧。
    """
    p = os.path.join(run_dir, 'evaluate.json')
    if not os.path.exists(p):
        return None
    try:
        with open(p, encoding='utf-8') as f:
            return json.load(f)
    except Exception:                      # noqa: BLE001 - 坏文件按缺失处理
        return None


def main():
    run_dir = sys.argv[1] if len(sys.argv) > 1 else '.'
    tag = sys.argv[2] if len(sys.argv) > 2 else os.path.basename(run_dir.rstrip('/'))
    n_ctx = 20
    if '--context' in sys.argv:
        n_ctx = int(sys.argv[sys.argv.index('--context') + 1])

    launch_log = os.path.join(run_dir, 'launch.log')
    probe = load_jsonl(os.path.join(run_dir, 'probe.jsonl'))
    sup = load_jsonl(os.path.join(run_dir, 'supervisor.jsonl'))
    evj = load_evaluate(run_dir)

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

    if evj is not None:
        # ——— 权威口径：完全复刻 evaluate.py 的结论，不重算 ———
        verdict = evj.get('verdict') or 'UNKNOWN'
        failed = [k for k, v in (evj.get('checks') or {}).items() if not v]
        coll = evj.get('collision') or {}
        rev_authorized = bool(coll.get('reverse_authorized'))
        rev_hard_n = int(coll.get('reverse_hard_windows') or 0)
        src = 'evaluate.json'
        fails = [f'{k}' for k in failed]
        if rev_hard_n and not rev_authorized:
            fails.append(f'非受控硬倒车 {rev_hard_n}')
    else:
        # ——— 回退口径（evaluate.json 缺失）：保留旧逻辑，但标注清楚 ———
        src = '回退口径（无 evaluate.json）'
        verdict = 'PASS'
        fails = []
        if tb['dispatch_n'] == 0:
            verdict = 'FAIL'
            fails.append('无目标派发')
        if tb['return_s'] is None:
            verdict = 'FAIL'
            fails.append('返航未完成')
        if physical:
            verdict = 'FAIL'
            fails.append(f'物理接触 {physical}')
        if hard_rev:
            verdict = 'FAIL'
            fails.append(f'非受控硬倒车 {len(hard_rev)}')
        rev_authorized = None
        rev_hard_n = len(hard_rev)

    out = []
    out.append(f'# 单轮体检报告 · {tag}\n')
    out.append(f'**结论：{verdict}**'
               + (f"（{'；'.join(fails)}）" if fails else '（全项通过）')
               + f'\n\n> 判定来源：{src}'
               + ('' if evj is not None
                  else ' —— ⚠ 与 evaluate.py 口径可能不同，请以 evaluate.py 为准')
               + '\n')

    out.append('## 1. 时间分布\n')
    out.append('| 项 | 值 |')
    out.append('| --- | --- |')
    out.append(f"| 启动（launch→Nav2 就绪） | {fmt(tb['startup_s'])} |")
    out.append(f"| 探索（首目标→结束建图） | {fmt(tb['explore_s'])} |")
    out.append(f"| 其中 导航中（派发→终态） | {fmt(tb['nav_s'])}（{pct(tb['nav_s'], tb['explore_s'])}）|")
    out.append(f"| 其中 选点等待（终态→下次派发） | {fmt(tb['select_wait_s'])}（{pct(tb['select_wait_s'], tb['explore_s'])}）|")
    out.append(f"| 返航（结束建图→返航完成） | {fmt(tb['return_s'])} |")
    # 2026-11-15：返航为 None 有两种完全不同的原因，不能都显示成"—"：
    #   a) 真的没返航（evaluate 也判失败）
    #   b) evaluate 用**位置兜底**放行（by_geometry：地图原点被家具围住不可规划，
    #      车停在 1.03~2.0 m 内且已静止）—— 此时日志里本来就没有"到点"事件，
    #      return_s 为空是事实正确，不是漏识别。
    if tb['return_s'] is None and evj is not None:
        _rk = evj.get('return_home_kind') or {}
        if _rk.get('by_geometry'):
            out.append('')
            out.append(f"> 注：本轮返航由 **位置兜底** 放行（终点距原点 "
                       f"{_rk.get('final_dist_to_home')} m、已静止）。"
                       f"日志中没有\"到点\"事件，故返航耗时不可算。")
        elif _rk.get('by_manager_geometry'):
            out.append('')
            out.append('> 注：本轮返航由 **manager 几何判据** 放行，'
                       '日志中无\"到点\"事件，故返航耗时不可算。')
    out.append(f"| 目标派发/成功/失败/取消 | {tb['dispatch_n']} / {tb['succ_n']} / {tb['fail_n']} / {tb['cancel_n']} |")
    out.append(f"| front blocked / 无净进展 事件 | {tb['front_blocked_n']} / {tb['no_progress_n']} |")

    out.append('\n## 2. 撞障与后退取证\n')
    out.append('| 项 | 值 |')
    out.append('| --- | --- |')
    out.append(f"| 物理接触（底盘/轮） | {p_last.get('chassis_contacts', 0)} / {p_last.get('wheel_contacts', 0)} |")
    out.append(f"| 受控倒车（仅返航，允许） | {len(assist_rev)} 次 / "
               f"{sum((e.get('dist') or 0) for e in assist_rev):.3f} m |")
    # 2026-11-15：\"非受控\"不再由本文件自行判定，而是引用 evaluate.py 的授权结论。
    # 两者必须同源，否则又会出现\"同一轮两份报告说法不同\"。
    if rev_authorized is None:
        _hard_txt = (f"{len(hard_rev)} 次 / "
                     f"{sum((e.get('dist') or 0) for e in hard_rev):.3f} m"
                     f"（回退口径，未做包络授权）")
    elif rev_hard_n == 0:
        _hard_txt = "**0 次**（无倒车窗口）"
    elif rev_authorized:
        _hard_txt = (f"**{rev_hard_n} 次，但全部在授权包络内**"
                     f"（explore_ok/return_ok/mppi_ok 之一成立）")
    else:
        _hard_txt = f"**{rev_hard_n} 次，且未被授权** ← 必须修"
    out.append(f"| 硬倒车窗口（判定见右） | {_hard_txt} |")
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
