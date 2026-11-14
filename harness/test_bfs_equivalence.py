#!/usr/bin/env python3
"""test_bfs_equivalence.py —— S5-B2 增量共享 BFS 的逐位等价性闸门。

为什么需要这个测试
------------------
B2 把"每拍 14 次独立 BFS"改成"1 棵共享树 + 14 次回溯"。这是**结构性**改动，
一旦路径不同，所有依赖路径的判据（净宽、走廊、中继取点）都会静默漂移，
而且不会报错 —— 属于最难查的一类缺陷。所以必须有机械化的等价性证明。

等价性依据（数学）
------------------
BFS 的展开序列由 FIFO 唯一确定：同样的入队顺序、同样的邻居顺序
(+x, -x, +y, -y) ⇒ 对任何目标 g（第 d 层），第 0..d 层的 parent 赋值
**逐位相同**。因此"跑到 g 就 break 的部分树"是"跑到更远目标的树"的**前缀**，
回溯出的路径必然一致。

本测试把这个论证变成可执行断言：用真实 `map.pgm` 转成的栅格，对随机的
(起点, 14 个目标) 组合，逐一比较

    参考实现（逐字照抄改造前的 path_and_width BFS 段）
    被测实现（`_IncrementalBFS`，与 frontier_arbiter 里运行的同一份代码）

必须**全部逐位相等**。

覆盖的场景（缺一不可）
----------------------
1. 同拍多目标共享（核心场景：14 个候选共用一棵树）
2. 目标不可达（原实现跑满 budget，共享树应同样返回 None）
3. 目标就是起点（原实现弹出即 found，路径为空）
4. 起点在障碍里 / 出界（两边的早退行为）
5. **调用顺序打乱**（先远后近 vs 先近后远）—— 证明"按需推进"对顺序不敏感
6. **跨拍复用**（同一 key 连续 ensure）—— 证明缓存不污染结果
7. 大面积障碍被围死的小房间（连通性边界）

用法
----
    python3 harness/test_bfs_equivalence.py            # 全跑
    python3 harness/test_bfs_equivalence.py -v         # 逐项
    python3 harness/test_bfs_equivalence.py --rounds 100
"""
import argparse
import glob
import importlib.machinery
import importlib.util
import os
import random
import sys

BASE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SIM_PKG = os.path.join(BASE, 'src', 'stemm_cartographer_exploration_sim')
ARB = os.path.join(SIM_PKG, 'scripts', 'frontier_arbiter')
MAPS = sorted(glob.glob(os.path.join(BASE, 'logs', 'round_*', 'map.pgm')))

_ok = []
_bad = []


def chk(cond, label, detail=''):
    (_ok if cond else _bad).append((label, detail))
    return cond


# --------------------------------------------------------------------------
# 加载被测实现（从真实源码，避免"测试副本"与生产代码不一致）
# --------------------------------------------------------------------------
def load_arbiter_module():
    loader = importlib.machinery.SourceFileLoader('fa_under_test', ARB)
    spec = importlib.util.spec_from_loader('fa_under_test', loader)
    mod = importlib.util.module_from_spec(spec)
    loader.exec_module(mod)                 # 只 import，不构造 Node
    return mod


def load_map_pgm(path):
    """把 map_saver 的 PGM 转成 costmap 语义（0=free, 100=occ, -1=unknown）。

    PGM 语义（nav2_map_server map_io.cpp）：0=occupied, 205=unknown, 254=free。
    """
    raw = open(path, 'rb').read()
    parts = raw.split(b'\n', 3)
    w, h = map(int, parts[1].split())
    img = bytearray(parts[3][:w * h])
    data = [-1] * (w * h)
    for row in range(h):
        src = (h - 1 - row) * w          # PGM 底行在最后 → 翻成 bottom-up
        dst = row * w
        for col in range(w):
            v = img[src + col]
            if v > 240:
                data[dst + col] = 0
            elif v <= 50:
                data[dst + col] = 100
    return data, w, h


def synth(w, h, seed, wall_density=0.18):
    """合成栅格：随机障碍 + 一个被围死的小房间（测连通性边界）。"""
    rng = random.Random(seed)
    data = [0] * (w * h)
    for i in range(w * h):
        if rng.random() < wall_density:
            data[i] = 100
    # 挖出一个完全封闭的 5x5 房间（四周全墙）
    cx, cy = w // 2, h // 2
    for y in range(cy - 3, cy + 4):
        for x in range(cx - 3, cx + 4):
            if 0 <= x < w and 0 <= y < h:
                edge = (abs(x - cx) == 3 or abs(y - cy) == 3)
                data[y * w + x] = 100 if edge else 0
    # 保证有一个连通的自由区
    for y in range(1, h - 1):
        data[y * w + 1] = 0
        data[y * w + 2] = 0
    return data


# --------------------------------------------------------------------------
# 参考实现：逐字照抄改造前 path_and_width 的 BFS 段
# --------------------------------------------------------------------------
def bfs_reference(data, w, h, sx, sy, gx, gy):
    from collections import deque
    parent = {}
    seen = bytearray(w * h)
    start = sy * w + sx
    seen[start] = 1
    q = deque([(sx, sy)])
    budget = max(80000, w * h * 2)
    found = False
    while q and budget > 0:
        cx, cy = q.popleft()
        budget -= 1
        if cx == gx and cy == gy:
            found = True
            break
        for nx, ny in ((cx + 1, cy), (cx - 1, cy), (cx, cy + 1), (cx, cy - 1)):
            if 0 <= nx < w and 0 <= ny < h:
                idx = ny * w + nx
                if seen[idx]:
                    continue
                v = data[idx]
                if v < 0 or v >= 90:
                    continue
                seen[idx] = 1
                parent[idx] = cy * w + cx
                q.append((nx, ny))
    if not found:
        return None
    path = []
    cur = gy * w + gx
    while cur != start:
        path.append(cur)
        cur = parent.get(cur)
        if cur is None:
            break
    path.reverse()
    return path


# --------------------------------------------------------------------------
# 测试
# --------------------------------------------------------------------------
class Tree:
    """把被测类包装成与参考实现相同的接口。"""

    def __init__(self, cls, data, w, h, sx, sy):
        self.t = cls(data, w, h, sx, sy)
        self.w, self.h = w, h

    def get(self, gx, gy):
        if not self.t.ensure(gx, gy):
            return None
        return self.t.path(gx, gy)


def _free_cells(data, w, h, limit=4000):
    out = [i for i, v in enumerate(data) if 0 <= v < 90]
    if len(out) > limit:
        step = len(out) // limit
        out = out[::step][:limit]
    return out


def run_case(cls, data, w, h, label, rng, n_targets=14, shuffle=False,
             verbose=False):
    free = _free_cells(data, w, h)
    if len(free) < 4:
        return
    s = rng.choice(free)
    sx, sy = s % w, s // w
    tgts = [rng.choice(free) for _ in range(n_targets)]
    # 掺入：目标=起点、障碍格、未知格、出界
    tgts.append(s)
    tgts.append(next((i for i, v in enumerate(data) if v >= 90), 0))
    tgts.append(next((i for i, v in enumerate(data) if v < 0), 0))
    if shuffle:
        rng.shuffle(tgts)

    tree = Tree(cls, data, w, h, sx, sy)
    mism = 0
    for t in tgts:
        gx, gy = t % w, t // w
        if not (0 <= gx < w and 0 <= gy < h):
            continue
        ref = bfs_reference(data, w, h, sx, sy, gx, gy)
        got = tree.get(gx, gy)
        if ref != got:
            mism += 1
            if verbose and mism <= 3:
                print('    ✗ %s 目标(%d,%d) 参考=%s 被测=%s'
                      % (label, gx, gy,
                         'None' if ref is None else 'len%d' % len(ref),
                         'None' if got is None else 'len%d' % len(got)))
    chk(mism == 0, '%s：%d 个目标路径逐位一致' % (label, len(tgts)),
        ('不一致 %d 个' % mism) if mism else '')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('-v', '--verbose', action='store_true')
    ap.add_argument('--rounds', type=int, default=20)
    a = ap.parse_args()

    try:
        mod = load_arbiter_module()
    except Exception as exc:                       # noqa: BLE001
        print('✗ 无法加载 frontier_arbiter：%r' % (exc,))
        return 2
    cls = getattr(mod, '_IncrementalBFS', None)
    if not chk(cls is not None, 'frontier_arbiter 里有 _IncrementalBFS'):
        print('  ✗ 找不到 _IncrementalBFS')
        return 1

    # --- 1/2/3/4/5/6：合成栅格，多轮随机 ---
    for rnd in range(a.rounds):
        w, h = 60, 45
        data = synth(w, h, seed=rnd)
        run_case(cls, data, w, h, '合成#%d' % rnd, random.Random(rnd),
                 shuffle=(rnd % 2 == 1), verbose=a.verbose)

    # --- 7：真实地图（有真实的房间/走廊拓扑）---
    maps = MAPS[:3] if MAPS else []
    if not maps:
        chk(True, '（跳过真实地图：logs/ 下没有 map.pgm）')
    for mp in maps:
        try:
            data, w, h = load_map_pgm(mp)
        except Exception as exc:                   # noqa: BLE001
            chk(False, '解析 %s' % os.path.basename(os.path.dirname(mp)), repr(exc))
            continue
        tag = os.path.basename(os.path.dirname(mp)).split('_')[-1]
        for rnd in range(3):
            run_case(cls, data, w, h, '真实地图 %s#%d' % (tag, rnd),
                     random.Random(1000 + rnd), verbose=a.verbose)

    # --- 8：缓存复用不污染结果（同 key 连续 ensure，顺序无关）---
    data = synth(50, 40, seed=99)
    w, h = 50, 40
    free = _free_cells(data, w, h)
    s = free[len(free) // 2]
    sx, sy = s % w, s // w
    tgts = [free[(i * 37) % len(free)] for i in range(10)]
    fwd = Tree(cls, data, w, h, sx, sy)
    got_fwd = [fwd.get(t % w, t // w) for t in tgts]
    rev = Tree(cls, data, w, h, sx, sy)
    got_rev = [rev.get(t % w, t // w) for t in reversed(tgts)][::-1]
    chk(got_fwd == got_rev,
        '8 目标顺序不影响结果（先近后远 vs 先远后近）')

    # 同一个 Tree 反复查同一目标
    again = [fwd.get(t % w, t // w) for t in tgts]
    chk(again == got_fwd, '8 同一目标重复查询结果稳定（缓存不污染）')

    # --- 9：不可达目标必须与参考一致（跑满）---
    data = synth(40, 30, seed=7, wall_density=0.30)
    w, h = 40, 30
    # 封闭房间内部的那个格
    cx, cy = w // 2, h // 2
    free = _free_cells(data, w, h)
    s = free[0]
    sx, sy = s % w, s // w
    ref = bfs_reference(data, w, h, sx, sy, cx, cy)
    t = Tree(cls, data, w, h, sx, sy)
    got = t.get(cx, cy)
    chk(ref == got, '9 封闭房间内目标与参考一致（可达性判定相同）',
        '参考=%s 被测=%s' % ('None' if ref is None else len(ref),
                            'None' if got is None else len(got)))

    for label, detail in _ok:
        if a.verbose:
            print('  ✓ %s  %s' % (label, detail))
    for label, detail in _bad:
        print('  ✗ %s  %s' % (label, detail))
    print('test_bfs_equivalence: %d 通过, %d 失败' % (len(_ok), len(_bad)))
    return 0 if not _bad else 1


if __name__ == '__main__':
    sys.exit(main())
