#!/usr/bin/env python3
"""s4_ghost_audit.py —— 判定「大到达误差」是 inplace 的正常记账，还是真幻觉。

为什么需要它
------------
S4 启用 inplace 后，manager 的 `reached; ... error=(xy=X m)` 会天然偏大：
  * 仲裁器用 /goal_update 把目标从 G 挪到附近的合格点 G*（例如 2.95 m 外）；
  * controller 到 G* 即判 succeeded（它只知道 G*）；
  * **manager 仍按原目标 G 记账** → 误差 = |car - G| 而不是 |car - G*|。
所以「误差大」本身不能证明有问题，必须区分两种情况：

  A. 正常（inplace 记账）  存活时长 >> 30 ms，且车到 **G*** 的距离很小
  B. 幻觉（P13-5 老问题）  存活时长 ~30 ms，车几乎没动却报到达

判据（任一成立即 A）：
  * 该目标在到达前出现过 inplace 纠偏记录，且 |car - G*| < 0.5 m
  * 或 存活时长 > 5 s（幻觉的特征是瞬时）

用法：
  python3 harness/s4_ghost_audit.py 'logs/round_*_p13s4_g*'
"""
import glob
import os
import re
import sys


TS = re.compile(r'^\[[^]]*\]\s*\[?[A-Z]*\]?\s*\[([0-9.]+)\]')
DISP = re.compile(r'navigation goal dispatched: source=\S+, target=\(([-\d.]+), ([-\d.]+)')
INPL = re.compile(r'后置纠偏.*?\(([-\d.]+),([-\d.]+)\) → \(([-\d.]+),([-\d.]+)\)')
REACH = re.compile(
    r'navigation goal reached; goal=\(([-\d.]+), ([-\d.]+).*'
    r'current=\(([-\d.]+), ([-\d.]+).*error=\(xy=([-\d.]+)m')


def audit(path):
    """返回 (正常条数, 疑似幻觉列表)。"""
    disp_t = None
    disp_xy = None
    inpl = {}            # (gx,gy) -> (t, gx*, gy*)
    ok, ghosts = 0, []
    for line in open(path, encoding='utf-8', errors='ignore'):
        m = TS.match(line)
        if not m:
            continue
        t = float(m.group(1))
        md = DISP.search(line)
        if md:
            disp_t = t
            disp_xy = (float(md.group(1)), float(md.group(2)))
            continue
        mi = INPL.search(line)
        if mi:
            key = (round(float(mi.group(1)), 2), round(float(mi.group(2)), 2))
            inpl[key] = (t, float(mi.group(3)), float(mi.group(4)))
            continue
        mr = REACH.search(line)
        if not mr:
            continue
        gx, gy = float(mr.group(1)), float(mr.group(2))
        cx, cy = float(mr.group(3)), float(mr.group(4))
        err = float(mr.group(5))
        life = (t - disp_t) if disp_t is not None else None
        # 找这个目标是否被 inplace 纠偏过
        key = (round(gx, 2), round(gy, 2))
        hit = inpl.get(key)
        if hit is not None:
            to_star = ((cx - hit[1]) ** 2 + (cy - hit[2]) ** 2) ** 0.5
            if to_star < 0.5:
                ok += 1
                continue
        if life is not None and life > 5.0:
            ok += 1
            continue
        ghosts.append(dict(goal=(gx, gy), cur=(cx, cy), err=err, life=life))
    return ok, ghosts


def main():
    pat = sys.argv[1] if len(sys.argv) > 1 else 'logs/round_*_p13s4_g*'
    tot_ok = tot_bad = 0
    print('%-10s %8s %8s  %s' % ('轮次', '正常到达', '疑似幻觉', '明细'))
    print('-' * 70)
    for d in sorted(glob.glob(os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))), pat))):
        lp = os.path.join(d, 'launch.log')
        if not os.path.exists(lp):
            continue
        ok, ghosts = audit(lp)
        tag = os.path.basename(d).split('_')[-1]
        tot_ok += ok
        tot_bad += len(ghosts)
        detail = '; '.join('err=%.2fm life=%s' % (
            g['err'], '—' if g['life'] is None else '%.2fs' % g['life'])
            for g in ghosts[:3])
        print('%-10s %8d %8d  %s' % (tag, ok, len(ghosts), detail))
    print('-' * 70)
    print('合计：正常 %d 条，疑似幻觉 %d 条' % (tot_ok, tot_bad))
    if tot_bad:
        print('⚠ 疑似幻觉需要人工核查（P13-5 的症状是 life≈0.03s 且车几乎没动）')
        return 1
    print('✓ 无幻觉：所有大误差都能归因于 inplace 记账口径或正常收敛')
    return 0


if __name__ == '__main__':
    sys.exit(main())
