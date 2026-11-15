#!/usr/bin/env python3
"""NCR 离线原型：验证「可通行面积已覆盖比例」这个指标**本身是否可用**。

为什么先离线
------------
P2 换终止判据最大的风险不是实现，而是**阈值可能永远达不到**
（分母里的"可达未知"若被地图外的开阔区撑爆，NCR 会长期趴在 0.5 左右，
判据就废了）。这个风险必须在动手写节点之前用历史地图量出来。

本脚本与将来 ROS 节点的计算内核共用同一套逻辑（`ncr_components`），
所以离线结论直接对线上成立。

指标定义
--------
    NCR = navigable_known / (navigable_known + reach_unknown)

  navigable_known
      = 已知空闲 且 净空 >= 机器人内切半径 的格数。
      净空用 EDT，且**把未知格也当障碍**——未知处可能藏障碍，
      机器人中心不能确定地站在那里，保守。

  reach_unknown
      = 从机器人当前位置出发，允许穿过「空闲或未知」能到达的未知格数。
      只算可达的那部分，地图外被墙隔开的未知区不计入。

偏差方向（重要）
----------------
分子保守（排除前沿附近），分母过宽（可达未知里有一部分其实是墙/不可通行），
⇒ NCR **低估**真实覆盖率 ⇒ 终止偏晚而不是偏早。
对「不许提前停止建图」这个要求来说，这是安全的一侧。

用法
----
  python3 harness/probe_ncr.py                      # 扫 logs/ 下全部轮次
  python3 harness/probe_ncr.py --dirs logs/round_x  # 指定轮次
  python3 harness/probe_ncr.py --pairs /tmp/p14vT_pairs.txt
"""
import argparse
import glob
import json
import os
import statistics as st
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# /map 编码：PGM 成品里 0=占据、205=未知、254=空闲。
# 注意 PGM 行是**自下而上**存的，row = h-1-my。
UNKNOWN = 205


def load_pgm(path):
    """返回 (pix: uint8 数组, w, h)；失败返回 (None, 0, 0)。"""
    import numpy as np
    with open(path, 'rb') as f:
        data = f.read()
    parts, idx = [], 0
    while len(parts) < 4 and idx < len(data):
        while data[idx:idx + 1] == b'#':
            idx = data.index(b'\n', idx) + 1
        j = idx
        while j < len(data) and data[j:j + 1].split():
            j += 1
        parts.append(data[idx:j])
        idx = j + 1
    w = int(parts[1])
    h = int(parts[2])
    pix = np.frombuffer(data[idx:idx + w * h], dtype=np.uint8)
    if pix.size < w * h:
        return None, 0, 0
    return pix[:w * h].reshape(h, w), w, h


def load_map_yaml(path):
    """极简 yaml 解析：只取 resolution / origin / width / height。"""
    out = {}
    if not os.path.exists(path):
        return out
    for line in open(path, encoding='utf-8', errors='replace'):
        s = line.strip()
        if s.startswith('resolution:'):
            out['res'] = float(s.split(':', 1)[1].split('#')[0])
        elif s.startswith('origin:'):
            body = s.split(':', 1)[1].strip().strip('[]')
            vals = [float(v) for v in body.split(',')]
            out['ox'], out['oy'] = vals[0], vals[1]
        elif s.startswith('width:'):
            out['w'] = int(float(s.split(':', 1)[1]))
        elif s.startswith('height:'):
            out['h'] = int(float(s.split(':', 1)[1]))
    return out


def ncr_components(pix, res, ox, oy, rx, ry, inscribed_m,
                   unknown=UNKNOWN):
    """核心计算。纯函数，无 ROS 依赖，便于单测与移植到车端。

    参数
    ----
    pix        : (h, w) uint8，PGM 原始值（行自下而上）
    res        : 分辨率 m/格
    ox, oy     : origin.position.x / y
    rx, ry     : 机器人世界坐标
    inscribed_m: 机器人内切半径（m）

    返回
    ----
    dict: ncr / ncr_interior / navigable_known / reach_unknown /
          reach_unknown_exterior / free_all / unknown_all / reach_ok / ms

    ★ 为什么必须再分一层「外部未知」（2026-11-15 实测）
    ---------------------------------------------------
    直接用 reach_unknown 当分母时，101 个历史轮次的 NCR **最高只有 0.644**
    （含覆盖最佳的 s4manual，known=68,502）。原因是「可达未知」里混进了
    房子**外面**的区域：地图窗口内、与图边连通的未知格，机器人永远探不到，
    分母消不掉 ⇒ NCR 存在永远达不到的硬地板，阈值判据直接失效。

    把「与地图边框连通的未知」剔除后（只算建筑内部的未知），同样 9 轮实测：
    NCR 从 0.52~0.64 升到 **0.915~0.960**（中位 0.948），指标落进可用区间。

    两者都输出：`ncr` 供对照，`ncr_interior` 是终止判据用的那个。
    """
    import numpy as np
    from scipy import ndimage

    t0 = time.time()
    h, w = pix.shape
    # ★ PGM 行序（2026-11-15 用整条运动轨迹判定，不再靠假设）
    #
    #   row = h - 1 - my      （my = (y - origin.y) / res）
    #
    # 判定方法：拿 probe.jsonl 里该轮全部位姿，分别按「直行」与「翻转」
    # 两种索引去查 map.pgm，统计落在 free(254) 的比例。p14vS 八轮实测：
    #   翻转 100.0% × 5 轮，直行 82.2~100%（3 轮并列、无一轮严格更好）
    # 与项目既有结论一致（行自下而上）。此前写反过一次，故留下判据。
    ci = int(round((rx - ox) / res))
    my = int(round((ry - oy) / res))
    cj = h - 1 - my
    start_ok = (0 <= ci < w) and (0 <= cj < h)
    ci = min(max(ci, 0), w - 1)
    cj = min(max(cj, 0), h - 1)

    unknown_mask = (pix == unknown)
    free_mask = (pix != unknown) & (pix > 0)      # 254 为主；>0 且非未知 = 空闲
    blocked = ~free_mask                           # 占据 或 未知

    # 1) 已知可通行面积：空闲 且 净空 >= 内切半径（未知按障碍算，保守）
    dist_cells = ndimage.distance_transform_edt(~blocked)
    nav_known = free_mask & (dist_cells * res >= inscribed_m)
    navigable_known = int(nav_known.sum())

    # 2) 外部未知：从地图四边出发、只走未知格能到达的未知格
    lab_u, _n = ndimage.label(unknown_mask)
    edge_ids = set()
    for edge in (lab_u[0, :], lab_u[-1, :], lab_u[:, 0], lab_u[:, -1]):
        edge_ids.update(int(v) for v in edge if v)
    if edge_ids:
        exterior = np.isin(lab_u, list(edge_ids)) & unknown_mask
    else:
        exterior = np.zeros_like(unknown_mask)

    # 3) 可达未知：允许穿「空闲或未知」，4 连通
    labels, _n = ndimage.label(free_mask | unknown_mask)
    lab0 = labels[cj, ci]
    reach_ok = bool(lab0 != 0)
    if reach_ok:
        reach_mask = (labels == lab0) & unknown_mask
        reach_unknown = int(reach_mask.sum())
        reach_ext = int((reach_mask & exterior).sum())
    else:
        reach_unknown = 0
        reach_ext = 0
    reach_interior = reach_unknown - reach_ext

    denom = navigable_known + reach_unknown
    ncr = (navigable_known / float(denom)) if denom > 0 else None
    denom_i = navigable_known + reach_interior
    ncr_int = (navigable_known / float(denom_i)) if denom_i > 0 else None

    return {
        'ncr': ncr,
        'ncr_interior': ncr_int,
        'navigable_known': navigable_known,
        'reach_unknown': reach_unknown,
        'reach_unknown_exterior': reach_ext,
        'reach_unknown_interior': reach_interior,
        'free_all': int(free_mask.sum()),
        'unknown_all': int(unknown_mask.sum()),
        'exterior_all': int(exterior.sum()),
        'reach_ok': reach_ok,
        'start_cell': (ci, cj),
        'start_ok': start_ok,
        'ms': (time.time() - t0) * 1000.0,
    }


def last_pose(round_dir):
    """从 probe.jsonl 取最后一个有效位姿（返航结束后仍在原位）。"""
    p = os.path.join(round_dir, 'probe.jsonl')
    if not os.path.exists(p):
        return None
    pos = None
    with open(p) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                r = json.loads(line)
            except Exception:
                continue
            q = r.get('pos')
            if isinstance(q, list) and len(q) >= 2:
                pos = (float(q[0]), float(q[1]))
    return pos


def run_one(round_dir, inscribed_m):
    pix, w, h = load_pgm(os.path.join(round_dir, 'map.pgm'))
    if pix is None:
        return None
    meta = load_map_yaml(os.path.join(round_dir, 'map.yaml'))
    res = meta.get('res', 0.05)
    ox, oy = meta.get('ox', 0.0), meta.get('oy', 0.0)
    pose = last_pose(round_dir)
    if pose is None:
        return None
    out = ncr_components(pix, res, ox, oy, pose[0], pose[1], inscribed_m)
    out['w'], out['h'] = w, h
    out['pose'] = pose
    return out


def main(argv):
    ap = argparse.ArgumentParser()
    ap.add_argument('--dirs', nargs='*')
    ap.add_argument('--pairs', default=None,
                    help='配对表；给了就按 far/base 成对统计')
    ap.add_argument('--pattern', default='p14vT')
    ap.add_argument('--inscribed', type=float, default=0.235,
                    help='机器人内切半径（m），默认 wheeltec_diff')
    ap.add_argument('--probe', action='store_true',
                    help='只对单轮做探针，打印全部中间量')
    args = ap.parse_args(argv[1:])

    dirs = args.dirs or sorted(glob.glob(os.path.join(ROOT, 'logs', 'round_*')))
    if not dirs:
        print('没有轮次目录')
        return 2

    print('## NCR 离线评估（内切半径 %.3f m）\n' % args.inscribed)
    hdr = '%-22s %-8s %-8s %-9s %-8s %-8s %-6s' % (
        '轮次', 'NCR原', 'NCR内部', '可通行已知', '可达未知', '其中外部', 'ms')
    print(hdr)
    print('-' * len(hdr))

    vals = []
    for d in dirs:
        r = run_one(d, args.inscribed)
        if r is None:
            continue
        name = os.path.basename(d).split('_', 2)[-1][:21]
        print('%-22s %-8s %-8s %-9d %-8d %-8d %-6.1f' % (
            name,
            ('%.4f' % r['ncr']) if r['ncr'] is not None else '-',
            ('%.4f' % r['ncr_interior']) if r['ncr_interior'] is not None else '-',
            r['navigable_known'], r['reach_unknown'],
            r['reach_unknown_exterior'], r['ms']))
        if r['ncr_interior'] is not None:
            vals.append((name, r))

    if not vals:
        print('\n没有成功计算的轮次')
        return 1

    raw = [r['ncr'] for _n, r in vals if r['ncr'] is not None]
    itr = [r['ncr_interior'] for _n, r in vals]
    ms_all = [r['ms'] for _n, r in vals]
    print('\n## 汇总（%d 轮，均为该轮**最终**地图）\n' % len(vals))
    if raw:
        print('NCR 原（分母含外部未知）：中位 %.4f  最小 %.4f  最大 %.4f'
              % (st.median(raw), min(raw), max(raw)))
    print('NCR 内部（分母剔除外部未知）：中位 %.4f  最小 %.4f  最大 %.4f'
          % (st.median(itr), min(itr), max(itr)))
    print('单次耗时 ：中位 %.1f ms  最大 %.1f ms' % (
        st.median(ms_all), max(ms_all)))
    for th in (0.85, 0.90, 0.93, 0.95, 0.97):
        print('  NCR内部 >= %.2f 的轮次：%d / %d' % (
            th, sum(1 for v in itr if v >= th), len(vals)))

    if args.probe:
        name, r = vals[0]
        print('\n## 探针明细：%s\n' % name)
        for k, v in sorted(r.items()):
            print('  %-18s %s' % (k, v))
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
