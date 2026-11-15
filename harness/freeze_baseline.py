#!/usr/bin/env python3
"""把某一批次的逐轮指标冻结成可入库的 jsonl。

为什么需要它
------------
`logs/` 与 `runs/` 都在 `.gitignore` 里 —— 所有批次数据只存在于本机，
换机器、清理磁盘或 180 天后失效的日志都会让"基线"凭空消失。
P0 的结论（`frame_far_first` 收益不显著）是后续所有对照的基础，
必须变成**随仓库走**的正式产物，而不是"某次本地 logs 里还有"。

用法
----
  python3 harness/freeze_baseline.py p4base p4far -o harness/baselines/p4_pre_p0.jsonl
  python3 harness/freeze_baseline.py p14vU -o harness/baselines/p14vU_p0_paired.jsonl

  # 列出可用批次
  python3 harness/freeze_baseline.py --list

输出字段（每轮一行）
--------------------
  round / tag / group / pose
  verdict / contacts / return_home / failed
  known0 / known1 / known_gain
  frame / fill
  tick_avg_med / sim_end
  wait_s / stall_s / stuck / stuck_mov
  reverse_explore_m / reverse_return_assist_m

约定
----
* `group` 从 tag 里的 `_far` / `_base` 推断，推不出记为 `-`。
* 缺失项写 null，**不写 0** —— 0 与"没测到"必须在分析时能区分。
* 出生点从该轮 `harness.log` 的 `本轮随机起点` 行抽取，抽不到写 null。
"""
import argparse
import glob
import json
import os
import re
import statistics as st
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LOGS = os.path.join(ROOT, 'logs')

RE_WAIT = re.compile(r'等候选\(waiting_for_rrt_goal\): \*\*([\d.]+) s\*\*')
RE_STALL = re.compile(r'停滞总秒数: \*\*([\d.]+) s\*\*')
RE_STUCK = re.compile(r'卡死事件（探索区间内无位移≥3 s）: \*\*(\d+)\*\*')
RE_STUCK_MOV = re.compile(r'想动却动不了\(cmd非0\): \*\*(\d+)\*\*')
RE_POSE = re.compile(r'本轮随机起点：x=([-\d.]+) y=([-\d.]+) yaw=([-\d.]+)')
RE_POSE_ALT = re.compile(r"check\":\"pose\",\"x\":([-\d.]+),\"y\":([-\d.]+),"
                         r"\"yaw\":([-\d.]+)")


def list_tags():
    """列出 logs/ 下出现过的批次 tag。"""
    tags = {}
    for d in glob.glob(os.path.join(LOGS, 'round_*')):
        n = os.path.basename(d)
        parts = n.split('_', 2)
        if len(parts) < 3:
            continue
        tag = parts[2]
        tag = re.sub(r'\d+$', '', tag) or tag
        tags[tag] = tags.get(tag, 0) + 1
    for k in sorted(tags):
        print('%-24s %d 轮' % (k, tags[k]))


def group_of(tag):
    """从 tag 推断 A/B 组。

    兼容两种命名：`p14vU_far01`（带下划线）与 `p4far`（无下划线，历史批次）。
    先看 `_far`/`_base`，再退回词尾匹配，避免把 `p4far` 判成 '-'。
    """
    if '_far' in tag:
        return 'far'
    if '_base' in tag:
        return 'base'
    # 去掉尾部的轮次序号，再看词尾
    stem = re.sub(r'\d+$', '', tag)
    if stem.endswith('far'):
        return 'far'
    if stem.endswith('base'):
        return 'base'
    return '-'


def load_round(d, tag):
    out = {'round': os.path.basename(d), 'tag': tag, 'group': group_of(tag)}
    out.update({k: None for k in (
        'verdict', 'contacts', 'return_home', 'failed',
        'known0', 'known1', 'known_gain', 'frame', 'fill',
        'tick_avg_med', 'sim_end', 'wait_s', 'stall_s',
        'stuck', 'stuck_mov', 'reverse_explore_m', 'reverse_return_assist_m',
        'pose')})

    ev = os.path.join(d, 'evaluate.json')
    if os.path.exists(ev):
        try:
            with open(ev) as f:
                e = json.load(f)
        except Exception:
            e = {}
        c = e.get('collision') or {}
        ch = e.get('checks') or {}
        out['verdict'] = e.get('verdict')
        out['contacts'] = c.get('physical_contacts')
        out['return_home'] = ch.get('return_home')
        out['failed'] = [k for k, v in ch.items() if not v]
        out['reverse_explore_m'] = c.get('reverse_explore_m')
        out['reverse_return_assist_m'] = c.get('reverse_return_assist_m')

    pool = os.path.join(d, 'cand_pool.jsonl')
    if os.path.exists(pool):
        known, frame, fill = [], 0, 0
        with open(pool) as f:
            for line in f:
                line = line.strip()
                if not line:
                    continue
                try:
                    r = json.loads(line)
                except Exception:
                    continue
                if r.get('known') is not None:
                    known.append(r['known'])
                ph = r.get('phase')
                if ph == 'frame':
                    frame += 1
                elif ph == 'fill':
                    fill += 1
        if known:
            out['known0'] = known[0]
            out['known1'] = known[-1]
            if len(known) >= 2:
                out['known_gain'] = known[-1] - known[0]
        out['frame'] = frame
        out['fill'] = fill

    tl_path = os.path.join(d, 'timeline.jsonl')
    if os.path.exists(tl_path):
        tavg, simend = [], None
        with open(tl_path) as f:
            for line in f:
                line = line.strip()
                if not line:
                    continue
                try:
                    r = json.loads(line)
                except Exception:
                    continue
                v = r.get('tick_ms_avg')
                if isinstance(v, (int, float)):
                    tavg.append(v)
                if r.get('sim') is not None:
                    simend = r['sim']
        if tavg:
            out['tick_avg_med'] = round(st.median(tavg), 2)
        out['sim_end'] = simend

    dg = os.path.join(d, 'digest.md')
    if os.path.exists(dg):
        txt = open(dg, encoding='utf-8', errors='replace').read()
        for key, rx, fn in (('wait_s', RE_WAIT, float),
                            ('stall_s', RE_STALL, float),
                            ('stuck', RE_STUCK, int),
                            ('stuck_mov', RE_STUCK_MOV, int)):
            m = rx.search(txt)
            if m:
                out[key] = fn(m.group(1))

    hl = os.path.join(d, 'harness.log')
    if os.path.exists(hl):
        txt = open(hl, encoding='utf-8', errors='replace').read()
        m = RE_POSE.search(txt) or RE_POSE_ALT.search(txt)
        if m:
            out['pose'] = [float(m.group(1)), float(m.group(2)),
                           float(m.group(3))]
    return out


def main(argv):
    ap = argparse.ArgumentParser()
    ap.add_argument('tags', nargs='*', help='批次 tag 前缀，如 p4base p4far')
    ap.add_argument('-o', '--out', help='输出 jsonl 路径（相对仓库根）')
    ap.add_argument('--list', action='store_true', help='列出可用 tag')
    args = ap.parse_args(argv[1:])

    if args.list or not args.tags:
        list_tags()
        return 0
    if not args.out:
        print('必须给 -o/--out')
        return 2

    dirs = []
    for tag in args.tags:
        dirs.extend(sorted(glob.glob(os.path.join(LOGS, 'round_*_%s*' % tag))))
    if not dirs:
        print('没有匹配的轮次目录')
        return 1

    rows = []
    for d in dirs:
        n = os.path.basename(d)
        tag = n.split('_', 2)[-1] if n.count('_') >= 2 else n
        rows.append(load_round(d, tag))

    outp = args.out if os.path.isabs(args.out) else os.path.join(ROOT, args.out)
    os.makedirs(os.path.dirname(outp), exist_ok=True)
    with open(outp, 'w', encoding='utf-8') as f:
        for r in rows:
            f.write(json.dumps(r, ensure_ascii=False, sort_keys=True) + '\n')
    print('已写入 %s（%d 轮）' % (outp, len(rows)))

    # 即时摘要，便于肉眼核对冻结结果是否合理
    for g in ('far', 'base', '-'):
        sub = [r for r in rows if r['group'] == g]
        if not sub:
            continue
        kn = [r['known1'] for r in sub if r['known1'] is not None]
        ct = [r['contacts'] for r in sub if r['contacts'] is not None]
        rt = [r['return_home'] for r in sub if r['return_home'] is not None]
        print('  %-5s n=%d  接触合计=%s  返航成功=%s/%s  known末中位=%s' % (
            g, len(sub),
            sum(ct) if ct else '-',
            sum(1 for x in rt if x), len(rt) if rt else '-',
            int(st.median(kn)) if kn else '-'))
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
