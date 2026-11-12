#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
classify_costmap.py —— 用「世界真值 2D 足迹」判定视觉层标记的格子是真障碍还是鬼影

纯离线：只读 harness/gt_footprints.py 生成的 logs/gt 产物 + 你给的格子来源，
不启动、不 kill 任何 Gazebo/ROS 进程。

真值位图（gt_grid.npy）的位定义（详见 gt_grid.json）
    bit0  有实体覆盖（任意 z，含地面铺装）
    bit1  低矮带覆盖：z 范围与 [0.05, 0.35] m 相交     → 「低矮家具」判据
    bit2  高带覆盖：z 范围存在 > 0.35 m 的部分         → 诊断位
    bit3  地面以上实体覆盖：z_max > 0.05 m             → 「真障碍」判据（剔除地板/地毯）

三分类（每一格只给一个主标签，三者互斥且合计 = 总格数）
    real_low_band   真障碍·低矮带障碍(0.05~0.35 m)   bit3=1 且 bit1=1
    real_high_only  真障碍·仅高带（有实体但 0.05~0.35 m 带内没有料，
                    例如吊灯/挂画/贴墙画框）           bit3=1 且 bit1=0
    ghost           无真值支撑（鬼影候选）             bit3=0
    （打印时同时给出「真障碍(任意高度)」= real_low_band + real_high_only）

关于容差
========
gt_grid 是 SDF 真值，视觉层是相机检测投影到 map 帧的估计值，两者都有误差：
本仓库实测（explore_latest.pgm 对比 gt_grid）静态地图与真值的整体偏差约 0.1~0.25 m。
因此默认按「±2 格（0.10 m）」容差判定（--tol-cells 2），同时输出严格（0 格容差）
与容差的计数，便于你自己决定口径。--tol-cells 0 即严格逐格判定。

输入来源（可任意组合，自动去重）
================================
  --cells-json <file>     JSON：[[x,y], ...] 或 {"cells": [[x,y], ...]} 或 {"cells":[{"x":..,"y":..}]}
  --jsonl <file>          JSONL：每行 {"cells": [[x,y], ...]}（空行 / # 注释忽略）
  --cells "x,y x,y"       直接在命令行给点（快捷自测）
  --costmap-pgm <pgm> <yaml>   nav2/map_saver 的 PGM + YAML，取其中「占用」格
  坐标语义：map 帧（== world 帧）米制坐标，格心。

输出
====
  --out 指定的 JSON（默认 logs/gt/classification.json）：
    分类计数与占比、按来源的计数、0.5 m 热力图（哪片区域鬼影最多）、逐点明细
  以及 stdout 的中文摘要。

用法示例
========
  python3 harness/classify_costmap.py --cells-json /tmp/cells20.json
  python3 harness/classify_costmap.py --jsonl logs/vision_cells.jsonl
  python3 harness/classify_costmap.py --costmap-pgm maps/explore_latest.pgm maps/explore_latest.yaml
  python3 harness/classify_costmap.py --cells "1.0,2.0 3.1,-4.2" --tol-cells 0 --out /tmp/r.json
"""

from __future__ import annotations

import argparse
import json
import math
import os
import re
import sys
from datetime import datetime

try:
    import numpy as np
except Exception as _exc:  # pragma: no cover - 只在缺依赖时走到
    sys.stderr.write(
        "错误：本工具需要 numpy（读取 gt_grid.npy 并判定格子），但导入失败：\n"
        f"  {_exc}\n"
        "请先安装：pip3 install numpy   或  sudo apt install python3-numpy\n"
    )
    sys.exit(2)

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_GT_DIR = os.path.join(REPO_ROOT, "logs", "gt")
DEFAULT_OUT = os.path.join(REPO_ROOT, "logs", "gt", "classification.json")
DEFAULT_BIN = 0.5      # 热力图网格边长 [m]
DEFAULT_TOL = 2        # 容差格数（2 格 = 0.10 m）

BITS = {
    "bit0": 0x01,
    "bit1": 0x02,
    "bit2": 0x04,
    "bit3": 0x08,
}

LABEL_NAME = {
    "real_low_band": "真障碍·低矮带(0.05~0.35m)",
    "real_high_only": "真障碍·仅高带(0.35m 以上)",
    "ghost": "无真值支撑(鬼影候选)",
}


# ================================================================ 真值位图
class GTGrid:
    """gt_grid.npy + gt_grid.json 的只读封装（含容差膨胀位图）。"""

    def __init__(self, gt_dir: str, tol_cells: int):
        self.dir = gt_dir
        npy = os.path.join(gt_dir, "gt_grid.npy")
        js = os.path.join(gt_dir, "gt_grid.json")
        if not os.path.isfile(npy):
            raise SystemExit(
                f"错误：找不到真值位图 {npy}\n"
                "请先运行：python3 harness/gt_footprints.py（会生成 logs/gt/gt_grid.npy）"
            )
        self.grid = np.load(npy)
        if self.grid.dtype != np.uint8:
            raise SystemExit(f"错误：{npy} 的 dtype 是 {self.grid.dtype}，期望 uint8")
        meta = {}
        if os.path.isfile(js):
            with open(js, encoding="utf-8") as fh:
                meta = json.load(fh)
        else:
            sys.stderr.write(f"警告：缺 {js}，按默认 res=0.05 / origin=(-9.7,-6.0) 处理\n")
        self.meta = meta
        self.res = float(meta.get("res", 0.05))
        origin = meta.get("origin", [-9.7, -6.0])
        self.ox, self.oy = float(origin[0]), float(origin[1])
        self.h, self.w = self.grid.shape
        self.tol = max(0, int(tol_cells))
        self.tol_grid = self._dilate_any(self.grid, self.tol) if self.tol > 0 else self.grid

    @staticmethod
    def _dilate_any(grid: "np.ndarray", radius: int) -> "np.ndarray":
        """
        对 bit0/bit1/bit2/bit3 逐位做方形膨胀（半径 radius 格），返回新位图。
        只用于「容差」判定，原始位图不受影响。
        """
        out = grid.copy()
        for bit in BITS.values():
            acc = (grid & bit) > 0
            if not acc.any():
                continue
            for _ in range(radius):
                nb = acc.copy()
                nb[1:, :] |= acc[:-1, :]
                nb[:-1, :] |= acc[1:, :]
                nb[:, 1:] |= acc[:, :-1]
                nb[:, :-1] |= acc[:, 1:]
                nb[1:, 1:] |= acc[:-1, :-1]
                nb[1:, :-1] |= acc[:-1, 1:]
                nb[:-1, 1:] |= acc[1:, :-1]
                nb[:-1, :-1] |= acc[1:, 1:]
                acc = nb
            out[acc] |= np.uint8(bit)
        return out

    def world_to_cell(self, x: float, y: float):
        """世界(map)帧米 → (row, col)；越界返回 None。"""
        col = int(math.floor((x - self.ox) / self.res))
        row = int(math.floor((y - self.oy) / self.res))
        if row < 0 or row >= self.h or col < 0 or col >= self.w:
            return None
        return row, col

    def cell_center(self, row: int, col: int):
        return self.ox + (col + 0.5) * self.res, self.oy + (row + 0.5) * self.res

    def bits_at(self, x: float, y: float, tolerant: bool = False):
        """返回该世界点所在格的位值；越界返回 None。"""
        rc = self.world_to_cell(x, y)
        if rc is None:
            return None
        grid = self.tol_grid if tolerant else self.grid
        return int(grid[rc[0], rc[1]])


def label_of(bits):
    """
    三分类主标签（输入原始位值）：
      仅 bit3 → ghost 判据；bit3+bit1 → 低矮带；bit3 且无 bit1 → 仅高带
    """
    if bits is None:
        return "out_of_grid"
    has_solid = (bits & BITS["bit3"]) != 0
    has_low = (bits & BITS["bit1"]) != 0
    if not has_solid:
        return "ghost"
    return "real_low_band" if has_low else "real_high_only"


# ================================================================ 输入读取
def load_cells_json(path: str):
    """JSON：[[x,y],...] 或 {"cells":[[x,y],...]} 或 {"cells":[{"x":..,"y":..}]}"""
    with open(path, encoding="utf-8") as fh:
        data = json.load(fh)
    return _extract_cells(data, path)


def _extract_cells(data, where: str):
    if isinstance(data, dict):
        for key in ("cells", "points", "cells_map", "data"):
            if key in data:
                return _extract_cells(data[key], where)
        raise SystemExit(f"错误：{where} 里没有 cells/points 字段")
    if not isinstance(data, list):
        raise SystemExit(f"错误：{where} 的顶层既不是列表也不是含 cells 的对象")
    out = []
    for i, item in enumerate(data):
        if isinstance(item, dict):
            if "x" in item and "y" in item:
                out.append((float(item["x"]), float(item["y"])))
                continue
            if "cells" in item:                     # 允许嵌套一层
                if isinstance(item["cells"], list):
                    out.extend(_extract_cells(item["cells"], f"{where}[{i}]"))
                continue
            raise SystemExit(f"错误：{where}[{i}] 是对象但缺少 x/y 字段：{item!r}")
        if isinstance(item, (list, tuple)) and len(item) >= 2:
            out.append((float(item[0]), float(item[1])))
            continue
        raise SystemExit(f"错误：{where}[{i}] 不是 [x, y]：{item!r}")
    return out


def load_cells_jsonl(path: str):
    """JSONL：每行 {"cells": [[x,y],...]}；空行与 # 注释忽略。"""
    out = []
    n_lines = 0
    with open(path, encoding="utf-8") as fh:
        for ln, line in enumerate(fh, 1):
            s = line.strip()
            if not s or s.startswith("#"):
                continue
            n_lines += 1
            try:
                obj = json.loads(s)
            except json.JSONDecodeError as exc:
                raise SystemExit(f"错误：{path}:{ln} 不是合法 JSON：{exc}")
            if isinstance(obj, dict) and "cells" in obj:
                out.extend(_extract_cells(obj["cells"], f"{path}:{ln}"))
            elif isinstance(obj, list):
                out.extend(_extract_cells(obj, f"{path}:{ln}"))
            else:
                raise SystemExit(f"错误：{path}:{ln} 既不是 {{'cells':[...]}} 也不是列表")
    if not out:
        sys.stderr.write(f"警告：{path} 里没有解析到任何格子（有效行 {n_lines} 行）\n")
    return out


def load_cells_inline(text: str):
    """'x,y x,y' 或 'x,y;x,y' 形式的命令行点列表。"""
    out = []
    # 形如 "1.0,2.0 3.1,-4.2" 或 "1.0 2.0;3.1 -4.2"：只用正则成对抽取，避免歧义
    pairs = re.findall(r"(-?\d+(?:\.\d+)?)\s*[, ]\s*(-?\d+(?:\.\d+)?)", text)
    for a, b in pairs:
        out.append((float(a), float(b)))
    if not out:
        raise SystemExit(f"错误：--cells 无法解析出任何坐标：{text!r}（格式示例：\"1.0,2.0 3.1,-4.2\"）")
    return out


def _read_pgm(path: str):
    """读 P2/P5 PGM。返回 (width, height, maxval, ndarray[h,w] uint8/uint16)。"""
    with open(path, "rb") as fh:
        data = fh.read()
    idx = 0
    toks = []
    while len(toks) < 4:
        while idx < len(data) and data[idx:idx + 1].isspace():
            idx += 1
        if idx >= len(data):
            raise SystemExit(f"错误：{path} 头部不完整")
        if data[idx:idx + 1] == b"#":
            while idx < len(data) and data[idx:idx + 1] not in (b"\n", b"\r"):
                idx += 1
            continue
        start = idx
        while idx < len(data) and not data[idx:idx + 1].isspace():
            idx += 1
        toks.append(data[start:idx])
    magic = toks[0]
    width, height, maxval = int(toks[1]), int(toks[2]), int(toks[3])
    if magic not in (b"P5", b"P2"):
        raise SystemExit(f"错误：{path} 不是 PGM（magic={magic!r}）；请给 map_saver 保存的 .pgm")
    if magic == b"P5":
        idx += 1
        dtype = np.uint8 if maxval < 256 else np.uint16
        need = width * height * (1 if maxval < 256 else 2)
        buf = np.frombuffer(data[idx:idx + need], dtype=dtype)
        if buf.size < width * height:
            raise SystemExit(f"错误：{path} 像素数据不足（{buf.size} < {width * height}）")
        img = buf[: width * height].reshape(height, width)
    else:
        vals = np.array(data[idx:].split(), dtype=int)
        img = vals[: width * height].reshape(height, width)
    return width, height, maxval, img


def _read_map_yaml(path: str):
    """读 nav2/map_saver 的 YAML；优先用 pyyaml，缺库时用正则兜底。"""
    try:
        import yaml  # noqa: F401

        with open(path, encoding="utf-8") as fh:
            doc = yaml.safe_load(fh) or {}
        return {
            "resolution": float(doc.get("resolution", 0.05)),
            "origin": [float(v) for v in doc.get("origin", [0.0, 0.0, 0.0])[:2]],
            "negate": int(doc.get("negate", 0)),
            "occupied_thresh": float(doc.get("occupied_thresh", 0.65)),
            "free_thresh": float(doc.get("free_thresh", 0.196)),
        }
    except Exception:
        txt = open(path, encoding="utf-8").read()
        def grab(key, default):
            m = re.search(rf"^\s*{key}\s*:\s*(.+)$", txt, re.M)
            return m.group(1).strip() if m else default
        origin_txt = grab("origin", "[0.0, 0.0, 0.0]").strip("[]")
        origin = [float(v) for v in re.split(r"[,\s]+", origin_txt.strip()) if v][:2]
        return {
            "resolution": float(grab("resolution", "0.05")),
            "origin": origin if len(origin) == 2 else [0.0, 0.0],
            "negate": int(float(grab("negate", "0"))),
            "occupied_thresh": float(grab("occupied_thresh", "0.65")),
            "free_thresh": float(grab("free_thresh", "0.196")),
        }


def load_cells_costmap(pgm: str, yaml_path: str, row_origin: str = "top"):
    """
    从 nav2/map_saver 的 PGM+YAML 取「占用」格，换算成 map 帧米制格心坐标。

    row_origin="top"    （nav2/map_saver 规范）行 0 在图上方：y = oy + (h-1-row+0.5)*res
    row_origin="bottom" 行 0 在图的 y 最小侧：        y = oy + (row+0.5)*res
    两者互为 y 镜像。本仓库的 maps/explore_latest.pgm 实测是 "bottom"（见主程序的自检输出），
    用 "top" 读会把整张图上下镜像，绝大多数格子会被误判成鬼影。
    """
    meta = _read_map_yaml(yaml_path)
    res = meta["resolution"]
    ox, oy = meta["origin"]
    width, height, maxval, img = _read_pgm(pgm)

    if maxval == 0:
        raise SystemExit(f"错误：{pgm} 的 maxval 为 0")
    occ_p = (img.astype(np.float64) / float(maxval)) if meta["negate"] else \
            (1.0 - img.astype(np.float64) / float(maxval))
    occupied = occ_p >= meta["occupied_thresh"]
    n_occ = int(occupied.sum())
    if n_occ == 0:
        sys.stderr.write(f"警告：{pgm} 里没有占用格（occupied_thresh={meta['occupied_thresh']}）\n")

    rows, cols = np.nonzero(occupied)
    xs = ox + (cols + 0.5) * res
    if row_origin == "bottom":
        ys = oy + (rows + 0.5) * res
    else:
        ys = oy + (height - 1 - rows + 0.5) * res
    cells = list(zip(xs.tolist(), ys.tolist()))
    info = {
        "pgm": pgm, "yaml": yaml_path, "resolution": res, "origin": [ox, oy],
        "size": [width, height], "occupied_cells": n_occ, "row_origin": row_origin,
        "mode": "occupied (PGM 值 → 占用概率 ≥ occupied_thresh)",
    }
    return cells, info


def measure_support(points, gt: GTGrid):
    """
    衡量一批点在真值上「有实体支撑」的比例（含容差）。用于自检两种 PGM 行序哪种对。
    """
    if not points:
        return 0.0
    solid = 0
    for x, y in points:
        bits = gt.bits_at(x, y, tolerant=True)
        if bits is not None and (bits & BITS["bit3"]):
            solid += 1
    return solid / len(points)


# ================================================================ 分类
def classify(points, gt: GTGrid):
    """对每个点做严格 + 容差两次判定，返回逐点结果列表。"""
    results = []
    for x, y in points:
        bits_strict = gt.bits_at(x, y, tolerant=False)
        bits_tol = gt.bits_at(x, y, tolerant=True)
        rc = gt.world_to_cell(x, y)
        results.append({
            "x": round(float(x), 4),
            "y": round(float(y), 4),
            "cell": [int(rc[1]), int(rc[0])] if rc else None,   # [col, row]
            "bits_strict": bits_strict,
            "bits_tolerant": bits_tol,
            "label": label_of(bits_strict),
            "label_tolerant": label_of(bits_tol),
            "flag_low_band": (bits_tol is not None and (bits_tol & BITS["bit1"]) != 0),
            "flag_high_band": (bits_tol is not None and (bits_tol & BITS["bit2"]) != 0),
        })
    return results


def summarize(results, tol_cells):
    """按主标签（容差口径）与严格口径分别计数。"""
    def count(key):
        c = {"real_low_band": 0, "real_high_only": 0, "ghost": 0, "out_of_grid": 0}
        for r in results:
            lab = r.get(key)
            c[lab if lab in c else "out_of_grid"] += 1
        return c

    tol = count("label_tolerant")
    strict = count("label")
    total = len(results)
    for c in (tol, strict):
        c["real_any_height"] = c["real_low_band"] + c["real_high_only"]
    return {
        "total": total,
        "primary": f"容差口径（±{tol_cells} 格）",
        "tolerant": tol,
        "strict": strict,
    }


def heatmap(results, bin_m: float, key: str = "label_tolerant"):
    """按 bin_m 米方格聚合，统计每块的格数 / 鬼影数 / 鬼影比例。"""
    bins = {}
    for r in results:
        if r["cell"] is None:
            continue
        ix = int(math.floor(r["x"] / bin_m))
        iy = int(math.floor(r["y"] / bin_m))
        b = bins.setdefault((ix, iy), {"total": 0, "ghost": 0, "real_low": 0, "real_high": 0})
        b["total"] += 1
        lab = r.get(key) or r["label"]
        if lab == "ghost":
            b["ghost"] += 1
        elif lab == "real_low_band":
            b["real_low"] += 1
        elif lab == "real_high_only":
            b["real_high"] += 1
    out = []
    for (ix, iy), b in bins.items():
        out.append({
            "ix": ix, "iy": iy,
            "x0": round(ix * bin_m, 3), "y0": round(iy * bin_m, 3),
            "x1": round((ix + 1) * bin_m, 3), "y1": round((iy + 1) * bin_m, 3),
            "total": b["total"], "ghost": b["ghost"],
            "real_low": b["real_low"], "real_high": b["real_high"],
            "ghost_ratio": round(b["ghost"] / b["total"], 4) if b["total"] else 0.0,
        })
    out.sort(key=lambda d: (-d["ghost"], -d["ghost_ratio"]))
    return out


# ================================================================ 输出
def print_report(res_by_source, summary, hm, top, gt: GTGrid, sources, out_path, bin_m):
    total = summary["total"]
    tol = summary["tolerant"]
    strict = summary["strict"]

    def pct(a, b=total):
        return (a / b * 100.0) if b else 0.0

    print("=" * 72)
    print("视觉层格子 × 世界真值足迹 分类结果")
    print("=" * 72)
    print(f"真值位图      : {os.path.join(gt.dir, 'gt_grid.npy')}"
          f"（{gt.w}x{gt.h} @ {gt.res} m，origin=({gt.ox}, {gt.oy})）")
    for name, info in sources.items():
        n = len(res_by_source.get(name, []))
        print(f"格子来源      : {name}  → {n} 格")
    print(f"容差          : ±{gt.tol} 格 = ±{gt.tol * gt.res:.2f} m（--tol-cells {gt.tol}，"
          f"0 即严格逐格）")
    print("-" * 72)
    print(f"总格数        : {total}")
    print(f"【容差口径 ±{gt.tol} 格 · 三分类主标签】")
    for key in ("real_low_band", "real_high_only", "ghost", "out_of_grid"):
        if tol.get(key):
            print(f"  {LABEL_NAME.get(key, key):26s}: {tol[key]:6d} 格  {pct(tol[key]):6.2f}%")
    print(f"  {'真障碍(任意高度) = 低矮带+仅高带':26s}: {tol['real_any_height']:6d} 格  "
          f"{pct(tol['real_any_height']):6.2f}%")
    print(f"【严格口径（0 容差）对照】")
    print(f"  真障碍(任意高度): {strict['real_any_height']} 格（{pct(strict['real_any_height']):.2f}%）"
          f"   低矮带: {strict['real_low_band']} 格   仅高带: {strict['real_high_only']} 格"
          f"   鬼影: {strict['ghost']} 格（{pct(strict['ghost']):.2f}%）")

    multi = len([k for k, v in res_by_source.items() if v])
    if multi > 1:
        print("-" * 72)
        print("按来源分别统计（容差口径）：")
        for name, rs in res_by_source.items():
            if not rs:
                continue
            s = summarize(rs, gt.tol)["tolerant"]
            n = len(rs)
            print(f"  {name}: {n} 格 → 真障碍 {s['real_any_height']} "
                  f"（低矮带 {s['real_low_band']} / 仅高带 {s['real_high_only']}），"
                  f"鬼影 {s['ghost']} ({s['ghost'] / n * 100:.1f}%)")

    print("-" * 72)
    print(f"热力图（{bin_m} m 方格）——鬼影最集中的区域 top {top}：")
    if not hm:
        print("  （没有落在真值网格内的点）")
    else:
        print(f"  {'x 区间':>16s} {'y 区间':>16s} {'格数':>6s} {'鬼影':>6s} {'鬼影比例':>8s}")
        for b in hm[:top]:
            print(f"  [{b['x0']:6.2f},{b['x1']:6.2f}] [{b['y0']:6.2f},{b['y1']:6.2f}] "
                  f"{b['total']:6d} {b['ghost']:6d} {b['ghost_ratio'] * 100:7.1f}%")
        full = [b for b in hm if b["ghost_ratio"] >= 1.0]
        print(f"  纯鬼影方块（该块内全是鬼影）: {len(full)} 个 / 共 {len(hm)} 个非空方块")
    print("-" * 72)
    print(f"输出 JSON：{out_path}")
    print("=" * 72)


def main(argv=None):
    ap = argparse.ArgumentParser(
        description="用世界真值 2D 足迹位图判定「视觉层标记的格子是真障碍还是鬼影」（纯离线）",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    ap.add_argument("--cells-json", action="append", default=[],
                    help="JSON 文件：[[x,y],...] 或 {\"cells\":[[x,y],...]}（可多次给）")
    ap.add_argument("--jsonl", action="append", default=[],
                    help="JSONL 文件：每行 {\"cells\": [[x,y],...]}（可多次给）")
    ap.add_argument("--cells", default=None,
                    help="直接在命令行给点，如 \"1.0,2.0 3.1,-4.2\"")
    ap.add_argument("--costmap-pgm", nargs=2, metavar=("PGM", "YAML"), default=None,
                    help="nav2/map_saver 的 PGM+YAML，取占用格作为待判定格子")
    ap.add_argument("--pgm-row-origin", choices=("top", "bottom"), default="top",
                    help="PGM 行 0 对应图的上方(top，nav2 规范)还是 y 最小侧(bottom)。"
                         "本仓库 maps/explore_latest.pgm 实测是 bottom——用 top 读会上下镜像，"
                         "把绝大多数格子误判成鬼影；程序会自动打印两种读法的真值支撑率")
    ap.add_argument("--gt", default=DEFAULT_GT_DIR,
                    help="真值产物目录（含 gt_grid.npy / gt_grid.json）")
    ap.add_argument("--tol-cells", type=int, default=DEFAULT_TOL,
                    help="容差膨胀格数（默认 2 格 = 0.10 m；0 为严格逐格）")
    ap.add_argument("--bin", type=float, default=DEFAULT_BIN, metavar="M",
                    help="热力图方格边长 [m]")
    ap.add_argument("--top", type=int, default=10, help="打印多少个鬼影热点方块")
    ap.add_argument("--out", default=DEFAULT_OUT, help="输出 JSON 路径")
    args = ap.parse_args(argv)

    if not (args.cells_json or args.jsonl or args.cells or args.costmap_pgm):
        ap.error("至少要给一个格子来源：--cells-json / --jsonl / --cells / --costmap-pgm")

    gt = GTGrid(args.gt, args.tol_cells)

    # ---- 收集各来源
    res_by_source = {}
    meta_sources = {}
    all_points = []

    for p in args.cells_json:
        pts = load_cells_json(p)
        key = f"--cells-json {p}"
        res_by_source[key] = pts
        all_points.extend(pts)
        meta_sources[key] = {"type": "cells-json", "path": p, "count": len(pts)}
    for p in args.jsonl:
        pts = load_cells_jsonl(p)
        key = f"--jsonl {p}"
        res_by_source[key] = pts
        all_points.extend(pts)
        meta_sources[key] = {"type": "jsonl", "path": p, "count": len(pts)}
    if args.cells:
        pts = load_cells_inline(args.cells)
        key = "--cells (命令行)"
        res_by_source[key] = pts
        all_points.extend(pts)
        meta_sources[key] = {"type": "inline", "count": len(pts)}
    if args.costmap_pgm:
        pgm, yml = args.costmap_pgm
        if not os.path.isfile(pgm):
            raise SystemExit(f"错误：找不到 PGM：{pgm}")
        if not os.path.isfile(yml):
            raise SystemExit(f"错误：找不到 YAML：{yml}")
        pts, info = load_cells_costmap(pgm, yml, args.pgm_row_origin)
        # 自检：两种行序都量一下「落在地面以上实体上」的比例，谁高谁才是对的
        try:
            alt = "bottom" if args.pgm_row_origin == "top" else "top"
            pts_alt, _ = load_cells_costmap(pgm, yml, alt)
            r_cur = measure_support(pts, gt)
            r_alt = measure_support(pts_alt, gt)
            info["row_origin_selfcheck"] = {
                "current": args.pgm_row_origin, "current_solid_support": round(r_cur, 4),
                "alternative": alt, "alternative_solid_support": round(r_alt, 4),
            }
            print(f"[PGM 行序自检] 当前 --pgm-row-origin={args.pgm_row_origin}："
                  f"真值实体支撑率 {r_cur * 100:.1f}%")
            print(f"[PGM 行序自检] 若改成 {alt}：真值实体支撑率 {r_alt * 100:.1f}%")
            if r_alt > r_cur + 0.20:
                print(f"[PGM 行序自检] 警告：{alt} 明显更贴合世界真值（高出 "
                      f"{(r_alt - r_cur) * 100:.1f} 个百分点）。")
                print(f"               这张 PGM 的行序与 nav2 规范相反，"
                      f"用 --pgm-row-origin {alt} 重跑才能得到正确判定。")
        except SystemExit:
            raise
        except Exception as exc:      # 自检失败不影响主流程
            print(f"[PGM 行序自检] 跳过（{type(exc).__name__}: {exc}）")
        key = f"--costmap-pgm {os.path.basename(pgm)}"
        res_by_source[key] = pts
        all_points.extend(pts)
        info["type"] = "costmap-pgm"
        info["count"] = len(pts)
        meta_sources[key] = info

    if not all_points:
        raise SystemExit("错误：所有来源都没有解析出格子，无内容可判定")

    # ---- 去重（跨来源重复坐标只判一次；来源归属仍按各来源单独统计）
    seen = set()
    uniq = []
    dup = 0
    for x, y in all_points:
        k = (round(x, 6), round(y, 6))
        if k in seen:
            dup += 1
            continue
        seen.add(k)
        uniq.append((x, y))

    results = classify(uniq, gt)
    summary = summarize(results, gt.tol)
    summary["duplicates_removed"] = dup
    summary["unique_points"] = len(uniq)
    hm = heatmap(results, args.bin)

    # 每个来源单独出结果（用于分来源统计）
    res_by_source_full = {}
    for name, pts in res_by_source.items():
        res_by_source_full[name] = classify(pts, gt)

    # ---- 输出 JSON
    out_dir = os.path.dirname(os.path.abspath(args.out))
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
    doc = {
        "generated_at": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
        "tool": "harness/classify_costmap.py",
        "gt": {
            "dir": gt.dir,
            "npy": os.path.join(gt.dir, "gt_grid.npy"),
            "meta": os.path.join(gt.dir, "gt_grid.json"),
            "res": gt.res,
            "origin": [gt.ox, gt.oy],
            "shape": [gt.h, gt.w],
            "tol_cells": gt.tol,
            "tol_m": gt.tol * gt.res,
            "bits": {k: v for k, v in BITS.items()},
            "bit_meaning": {
                "bit0": "有实体覆盖（任意 z，含地面）",
                "bit1": "低矮带覆盖 z∈[0.05,0.35] m",
                "bit2": "高带覆盖 z>0.35 m（诊断）",
                "bit3": "地面以上实体覆盖 z_max>0.05 m（真障碍判据）",
            },
        },
        "sources": meta_sources,
        "summary": summary,
        "heatmap": {"bin_m": args.bin, "bins": hm},
        "points": results,
    }
    with open(args.out, "w", encoding="utf-8") as fh:
        json.dump(doc, fh, ensure_ascii=False, indent=2)

    print_report(res_by_source_full, summary, hm, max(1, args.top), gt,
                 meta_sources, args.out, args.bin)
    return 0


if __name__ == "__main__":
    sys.exit(main())
