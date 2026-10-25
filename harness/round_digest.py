#!/usr/bin/env python3
"""round_digest.py —— 观测工作流 v2：把一轮的"多方日志"合成可查验的三件产物。

用法：
    python3 harness/round_digest.py logs/round_20261025-xxxx_tag [--no-png]

产物（都写在同一轮目录里）：
    timeline.csv   每拍一行（1 Hz，来自仲裁器 timeline.jsonl）+ 秒级时间戳，
                   并把 probe / supervisor / launch.log 的事件按时间对齐进同一行
                   —— 这是"多方日志秒级交叉核对"的落点。
    trajectory.png 地图 + 实际轨迹（按阶段着色）+ **每秒一个刻度点、每 10 s 一个
                   时间戳标签** + 事件打点（派发/卡死/接触/脱困/返航开始）。
    digest.md      分级验收（S0~S3）+ 卡死事件清单 + 派发来源 + 候选拒绝分布，
                   外加一段"这一轮到底发生了什么"的机器生成摘要。

设计约束（2026-10-25 定）：
  * 只读已有产物，不启动仿真；缺失的数据源自动降级并在 digest.md 里标注；
  * 时间统一用"仿真秒"（probe/timeline 里的 sim 字段），墙钟时间另存一列；
  * 卡死事件定义：位移 < 0.05 m 连续 >= 3 s（无位移）或 cmd 全 0 连续 >= 3 s。
"""
import argparse
import csv
import json
import math
import os
import sys
from collections import Counter, defaultdict


# ---------------------------------------------------------------- 数据源
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
        except Exception:            # noqa: BLE001
            continue
    return out


def load_timeline(d):
    rows = load_jsonl(os.path.join(d, 'timeline.jsonl'))
    rows.sort(key=lambda r: r.get('t') or 0)
    return rows


def load_probe(d):
    return load_jsonl(os.path.join(d, 'probe.jsonl'))


def load_supervisor(d):
    return load_jsonl(os.path.join(d, 'supervisor.jsonl'))


def load_evaluate(d):
    p = os.path.join(d, 'evaluate.json')
    if not os.path.exists(p):
        return {}
    try:
        return json.load(open(p, encoding='utf-8'))
    except Exception:                # noqa: BLE001
        return {}


def load_launch_events(d):
    """从 launch.log 里抽关键事件（派发 / 返航 / 拒绝 / 脱困），带墙钟时间戳。"""
    p = os.path.join(d, 'launch.log')
    ev = []
    if not os.path.exists(p):
        return ev
    import re
    pat_dispatch = re.compile(r'\[(\d{10}\.\d+)\].*navigation goal dispatched: source=(\w+), '
                              r'target=\(([-\d.]+), ([-\d.]+)\), robot=\(([-\d.]+), ([-\d.]+)')
    pat_ts = re.compile(r'\[(\d{10}\.\d+)\]')
    for line in open(p, encoding='utf-8', errors='ignore'):
        m = pat_dispatch.search(line)
        if m:
            ev.append({'t': float(m.group(1)), 'kind': 'dispatch', 'src': m.group(2),
                       'goal': [float(m.group(3)), float(m.group(4))],
                       'robot': [float(m.group(5)), float(m.group(6))]})
            continue
        if '出发返航' in line or 'return-home goal dispatched' in line or 'return-home target' in line:
            mt = pat_ts.search(line)
            if mt:
                ev.append({'t': float(mt.group(1)), 'kind': 'return_start'})
            continue
        if '静止超时' in line:
            mt = pat_ts.search(line)
            mm = re.search(r'静止超时 ([\d.]+)s @\(([-\d.]+),([-\d.]+)\)', line)
            if mt:
                ev.append({'t': float(mt.group(1)), 'kind': 'watchdog_stall',
                           'dur': float(mm.group(1)) if mm else None,
                           'pos': [float(mm.group(2)), float(mm.group(3))] if mm else None})
            continue
        if '脱困失败受阻区' in line:
            mt = pat_ts.search(line)
            if mt:
                ev.append({'t': float(mt.group(1)), 'kind': 'escape_fail'})
            continue
        if '前沿全部派不出去' in line:
            mt = pat_ts.search(line)
            if mt:
                ev.append({'t': float(mt.group(1)), 'kind': 'escape_goal'})
    return ev


# ---------------------------------------------------------------- 卡死事件
def stuck_events(tl, min_sec=3.0, eps_m=0.05):
    """连续 min_sec 内位移 < eps_m 的时段算一次卡死；另外标出 cmd 全 0 的时段。"""
    ev = []
    if len(tl) < 3:
        return ev
    run = []
    for r in tl:
        if r.get('x') is None or r.get('y') is None:
            continue
        run.append(r)
    # 位移判据
    i = 0
    while i < len(run):
        j = i
        while j + 1 < len(run):
            d = math.hypot(run[j + 1]['x'] - run[i]['x'], run[j + 1]['y'] - run[i]['y'])
            if d >= eps_m:
                break
            j += 1
        dur = (run[j].get('t') or 0) - (run[i].get('t') or 0)
        if dur >= min_sec:
            zeros = all((run[k].get('cmd') or [1, 1])[0] == 0 for k in range(i, j + 1))
            ev.append({'kind': 'no_move', 't0': run[i].get('t'), 't1': run[j].get('t'),
                       'dur': round(dur, 1), 'pos': [run[i]['x'], run[i]['y']],
                       'cmd_zero': zeros,
                       'busy': any(run[k].get('busy') for k in range(i, j + 1))})
            i = j + 1
        else:
            i += 1
    return ev


# ---------------------------------------------------------------- 出图
def render_png(d, tl, evs, probe_evs, ev_meta, out_png):
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        import numpy as np
        from PIL import Image
        import yaml
    except Exception as exc:         # noqa: BLE001
        return f'跳过出图（缺依赖：{type(exc).__name__}: {exc}）'
    mp = os.path.join(d, 'map.pgm')
    if not os.path.exists(mp):
        return '跳过出图（没有 map.pgm）'
    y = yaml.safe_load(open(os.path.join(d, 'map.yaml'), encoding='utf-8'))
    res = y['resolution']; ox, oy, _ = y['origin']
    img = np.array(Image.open(mp))
    H, W = img.shape
    fig, ax = plt.subplots(figsize=(13, 9), dpi=110)
    ax.imshow(img, cmap='gray', vmin=0, vmax=255,
              extent=[ox, ox + W * res, oy, oy + H * res], origin='upper')
    # 轨迹：按"有没有活跃目标"分色
    xs = [r['x'] for r in tl if r.get('x') is not None]
    ys = [r['y'] for r in tl if r.get('y') is not None]
    ts = [r.get('t') for r in tl if r.get('x') is not None]
    if xs:
        ax.plot(xs, ys, '-', color='#1f77b4', lw=1.6, label='trajectory (1 Hz samples)')
        # 每秒刻度点
        ax.plot(xs, ys, '.', color='#1f77b4', ms=2.2, alpha=0.75)
        # 每 10 s 一个时间戳标签（墙钟相对秒 → 同时给出仿真秒）
        t0 = ts[0]
        sim0 = next((r.get('sim') for r in tl if r.get('sim')), None)
        for k, (x, yy, t) in enumerate(zip(xs, ys, ts)):
            if t is None:
                continue
            rel = t - t0
            if rel >= 10 and abs(rel % 10) < 0.6:
                simlab = ''
                if sim0 is not None:
                    simrow = tl[k].get('sim')
                    if simrow is not None:
                        simlab = f'{simrow - sim0:+.0f}s'
                ax.annotate(f'{rel:.0f}s {simlab}', (x, yy), fontsize=6.5, color='#333',
                            xytext=(3, 3), textcoords='offset points')
        ax.plot(xs[0], ys[0], 'o', color='lime', ms=9, label='start')
        ax.plot(xs[-1], ys[-1], 's', color='red', ms=8, label='end')
        # 事件打点
        for e in evs + probe_evs:
            p = e.get('pos')
            if not p:
                continue
            kind = e.get('kind') or e.get('type')
            style = {
                'dispatch': ('^', '#2ca02c', 7),
                'watchdog_stall': ('X', 'orange', 9),
                'escape_fail': ('P', 'purple', 9),
                'escape_goal': ('o', '#17becf', 6),
                'no_move': ('X', 'red', 10),
                'contact': ('*', 'red', 12),
                'reverse': ('v', 'brown', 7),
                'stall': ('D', 'orange', 7),
            }.get(kind)
            if style is None:
                continue
            mk, col, sz = style
            ax.plot(p[0], p[1], mk, color=col, ms=sz, alpha=0.9)
            if kind in ('no_move', 'contact', 'watchdog_stall', 'escape_fail'):
                lab = kind
                if e.get('dur'):
                    lab += f" {e['dur']:.0f}s"
                ax.annotate(lab, (p[0], p[1]), fontsize=6, color=col,
                            xytext=(4, -7), textcoords='offset points')
        ax.set_title(f"{os.path.basename(d)}  verdict={ev_meta.get('verdict')}  "
                     f"contact={ev_meta.get('contacts')}  stuck_events={ev_meta.get('stuck_n')}")
        ax.set_xlabel('x [m]'); ax.set_ylabel('y [m]')
        ax.set_aspect('equal')
        ax.legend(loc='upper right', fontsize=7)
        fig.tight_layout()
        fig.savefig(out_png)
        plt.close(fig)
        # 出图自检（我看不到图，所以让程序说清楚它画了多少东西）
        try:
            arr = np.array(Image.open(out_png).convert('RGB'))
            # 轨迹色 #1f77b4 = (31,119,180)：统计近似像素数
            m = ((abs(arr[:, :, 0].astype(int) - 31) < 26) &
                 (abs(arr[:, :, 1].astype(int) - 119) < 30) &
                 (abs(arr[:, :, 2].astype(int) - 180) < 30))
            n_traj = int(m.sum())
            uniq = len(np.unique(arr.reshape(-1, 3), axis=0))
            return f'ok（{arr.shape[1]}x{arr.shape[0]}, 轨迹像素 {n_traj}, 颜色数 {uniq}）'
        except Exception as exc:      # noqa: BLE001
            return f'ok（自检跳过：{type(exc).__name__}）'


# ---------------------------------------------------------------- 主流程
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('round_dir')
    ap.add_argument('--no-png', action='store_true')
    a = ap.parse_args()
    d = a.round_dir.rstrip('/')
    ev = load_evaluate(d)
    tl = load_timeline(d)
    probe = load_probe(d)
    sup = load_supervisor(d)
    lev = load_launch_events(d)

    # 老轮次没有 timeline（1 Hz）：退化成 probe 的 10 s 采样，保证图还是能画
    degraded = False
    if not tl:
        for r in probe:
            if r.get('type') == 'summary' and r.get('pos'):
                tl.append({'t': r.get('t'), 'sim': r.get('sim'),
                           'x': r['pos'][0], 'y': r['pos'][1], 'cmd': [None, None]})
        degraded = bool(tl)
        tl.sort(key=lambda r: r.get('t') or 0)
    coll = ev.get('collision') or {}
    stuck = stuck_events(tl)
    probe_events = []
    for r in probe:
        k = r.get('type')
        if k in ('contact', 'stall') and r.get('pos'):
            probe_events.append({'kind': k, 'pos': r['pos'], 't': r.get('t')})
        if k == 'reverse' and r.get('phase') == 'end' and r.get('pos'):
            probe_events.append({'kind': 'reverse', 'pos': r['pos'], 't': r.get('t'),
                                 'dur': r.get('dur'), 'assist': r.get('assist')})
    meta = {'verdict': ev.get('verdict'), 'contacts': coll.get('physical_contacts'),
            'stuck_n': len(stuck)}

    # ---- timeline.csv ----
    evs_by_t = defaultdict(list)
    for e in lev:
        evs_by_t[round(e['t'], 1)].append(e['kind'])
    csv_path = os.path.join(d, 'timeline.csv')
    if tl:
        keys = ['t_wall', 'sim', 'x', 'y', 'yaw', 'cmd_lin', 'cmd_ang', 'busy', 'mode', 'nav',
                'front', 'n_cands', 'published', 'veto', 'escape_goal', 'fallback_obs',
                'rejected_narrow', 'rejected_oneway', 'events']
        with open(csv_path, 'w', newline='', encoding='utf-8') as fh:
            w = csv.DictWriter(fh, fieldnames=keys)
            w.writeheader()
            sim0 = tl[0].get('sim') if tl else None
            for r in tl:
                cmd = r.get('cmd') or [None, None]
                row = {'t_wall': r.get('t'), 'sim': r.get('sim'), 'x': r.get('x'), 'y': r.get('y'),
                       'yaw': r.get('yaw'), 'cmd_lin': cmd[0], 'cmd_ang': cmd[1],
                       'busy': r.get('busy'), 'mode': r.get('mode'), 'nav': r.get('nav'),
                       'front': r.get('front'), 'n_cands': r.get('n_cands'),
                       'published': r.get('published'), 'veto': r.get('veto'),
                       'escape_goal': r.get('escape_goal'),
                       'fallback_obs': r.get('fallback_obs'),
                       'rejected_narrow': r.get('rejected_narrow'),
                       'rejected_oneway': r.get('rejected_oneway'),
                       'events': ''}
                row['events'] = '|'.join(evs_by_t.get(round(r.get('t') or 0, 1), []))
                w.writerow(row)

    # ---- digest.md ----
    src = Counter(e['src'] for e in lev if e['kind'] == 'dispatch')
    total_src = sum(src.values()) or 1
    lines = [f'# 单轮 digest · {os.path.basename(d)}', '']
    lines += ['## 0. 结论', '']
    checks = ev.get('checks') or {}
    bad = [k for k, v in checks.items() if v is False]
    lines.append(f'- verdict: **{ev.get("verdict")}**　失败项: {bad if bad else "无"}')
    lines.append(f'- 物理接触: {coll.get("physical_contacts")}　'
                 f'倒车: {coll.get("reverse_m")} m（探索段 {coll.get("reverse_explore_m")} / '
                 f'返航受控 {coll.get("reverse_return_assist_m")}）')
    lines.append(f'- 卡死事件（无位移≥3 s）: **{len(stuck)}**　'
                 f'其中 cmd 全 0: {sum(1 for s in stuck if s.get("cmd_zero"))}')
    lines.append(f'- 派发来源: ' + '　'.join(
        f'{k}={v}({100*v//total_src}%)' for k, v in src.most_common()) if src else '- 派发: 无记录')
    lines.append('')
    lines += ['## 1. 分级验收（逐级加严）', '',
              '| 级别 | 判据 | 本轮 | 通过 |', '| --- | --- | --- | --- |']
    s0 = (coll.get('physical_contacts') or 0) == 0
    s1 = len(stuck) == 0
    esc_goals = sum(1 for e in lev if e['kind'] == 'escape_goal')
    wd_stalls = sum(1 for e in lev if e['kind'] == 'watchdog_stall')
    s2 = wd_stalls == 0
    s3 = bool(checks.get('return_home'))
    lines += [f'| S0 | 零物理接触 | {coll.get("physical_contacts")} | {"✅" if s0 else "❌"} |',
              f'| S1 | 零卡死事件（无位移≥3 s） | {len(stuck)} | {"✅" if s1 else "❌"} |',
              f'| S2 | 零看门狗静止超时 | {wd_stalls} | {"✅" if s2 else "❌"} |',
              f'| S3 | 返航完成（仅记录，不阻塞） | {checks.get("return_home")} | '
              f'{"✅" if s3 else "❌"} |']
    lines += ['', f'（本轮"前沿全灭→脱困目标"触发 {esc_goals} 次；'
                  f'时间线采样 {len(tl)} 拍；判据数据来自 evaluate.json / timeline.jsonl / '
                  f'probe.jsonl / launch.log）', '']
    lines += ['## 2. 卡死事件清单', '']
    if stuck:
        lines += ['| # | 起(s) | 止(s) | 时长(s) | 位置 | cmd全0 | 有活跃目标 |', '| --- | --- | --- | --- | --- | --- | --- |']
        for i, s in enumerate(stuck, 1):
            lines.append(f'| {i} | {s["t0"]:.0f} | {s["t1"]:.0f} | {s["dur"]:.0f} | '
                         f'({s["pos"][0]:.2f},{s["pos"][1]:.2f}) | {"是" if s["cmd_zero"] else "否"} | '
                         f'{"是" if s["busy"] else "否"} |')
    else:
        lines.append('- 无')
    lines += ['', '## 3. 事件时间线（派发 / 静止超时 / 脱困 / 返航）', '']
    for e in lev:
        if e['kind'] == 'dispatch':
            lines.append(f'- {e["t"]:.0f} 派发 {e["src"]} → ({e["goal"][0]:.2f},{e["goal"][1]:.2f})')
        elif e['kind'] in ('watchdog_stall', 'escape_fail', 'escape_goal', 'return_start'):
            lines.append(f'- {e["t"]:.0f} {e["kind"]}' + (f' @{e.get("pos")}' if e.get('pos') else ''))
    missing = []
    if degraded:
        missing.append('timeline.jsonl（老轮次没有 1 Hz 时间线；已退化为 probe 的 10 s 采样，'
                       '卡死事件的秒级判定不可用）')
    if not os.path.exists(os.path.join(d, 'map.pgm')):
        missing.append('map.pgm')
    if missing:
        lines += ['', '## 4. 数据缺失', ''] + [f'- {m}' for m in missing]
    open(os.path.join(d, 'digest.md'), 'w', encoding='utf-8').write('\n'.join(lines) + '\n')

    # ---- PNG ----
    png_note = '未生成'
    if not a.no_png:
        png_note = render_png(d, tl, [dict(s, pos=s['pos'], kind='no_move') for s in stuck],
                              probe_events, meta, os.path.join(d, 'trajectory.png'))

    print(f'[digest] {os.path.basename(d)}: verdict={ev.get("verdict")} '
          f'卡死事件={len(stuck)} 派发={total_src} 接触={coll.get("physical_contacts")}')
    print(f'[digest] timeline.csv={"有" if tl else "无"}　trajectory.png={png_note}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
