#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
gt_footprints.py —— 世界真值 2D 足迹构建器（纯离线：只读 SDF/网格文件 + numpy）

禁止事项：本工具不启动、不 kill、不接触任何 Gazebo / ROS 进程；只读 world 文件和
~/.gazebo/models 下的模型库，绝不写入这两处。

用途
====
把 Gazebo SDF 世界（aws_small_house.world）里每个 <include> 模型的每个 <collision>
几何，解析成 world 帧 XY 平面上的**真实投影轮廓**（不是外接矩形：网格按三角面投影
求并集），再栅格化成一张 uint8 位图。这张位图就是
「视觉层标记的格子到底是真实障碍，还是鬼影」的分类器核心输入。

产出（默认 --out logs/gt）
=========================
  footprints.json   每个 include / 每个 collision 的世界 bbox、类型、来源文件、
                    是否 mesh、mesh 顶点/三角面数、z 范围、低矮带命中、覆盖格数
  gt_grid.npy       uint8 位图（res 默认 0.05 m）
  gt_grid.json      位图元数据（res / origin / width / height / 位定义 / 统计）

gt_grid.npy 位定义
==================
  bit0 = 1  有实体覆盖（任意 z，**含地面铺装**）——严格按需求定义
  bit1 = 1  低矮带覆盖：该 collision 的世界 z 范围与 [0.05, 0.35] m 相交
  bit2 = 1  高带覆盖：该 collision 的世界 z 范围存在 > 0.35 m 的部分（诊断扩展位）
  bit3 = 1  地面以上实体覆盖：该 collision 的 z_max > 0.05 m
            （**分类器实际使用的「真障碍」判据**；等于 bit0 剔除纯地面铺装/地毯）

  为什么还要 bit3：本世界里有 FloorB_01（19.0 m × 11.4 m 地板，z∈[-0.04, 0]）
  和 Carpet_01（地毯，z≈-0.01）。纯按 bit0「任意 z」算，整栋房子内部都是 1，
  位图对分类毫无区分力。bit3 用 z_max>0.05 m 剔除地面铺装——这条阈值规则是从
  数据算出来的，不是按模型名猜的，且被剔除的碰撞体在 footprints.json 里全部列明。

坐标 / 索引约定（务必注意）
===========================
  1) world 帧 == map 帧（机器人从世界原点 (0,0,0,yaw=0) 起图）。
  2) 栅格按 numpy 矩阵约定：行 0 = 世界 y 最小值，列 0 = 世界 x 最小值。
       格 (row, col) 的格心:  x = ox + (col + 0.5) * res
                              y = oy + (row + 0.5) * res
     这与 nav2/map_saver 的 PGM（行 0 在上方）相反；读 PGM 时要做换算，
     classify_costmap.py 里已处理。
  3) 默认栅格 origin=(-9.7,-6.0)、res=0.05，宽高自动算到至少覆盖
     x∈[ox, 10.0]、y∈[oy, 7.0]（--bounds-max 的右/上边界）。

几何处理的几条「真话」（全部实测，不是猜的）
==========================================
  * 每份 collision DAE 的场景 <node> 上带一个 <matrix>，里面往往烘进了
    0.1 / 0.118 之类的**缩放**（FBX COLLADA 导出器干的），asset 里还有
    <unit meter="0.01">（厘米）。两者都必须乘上，否则 AABB 会大 10~100 倍。
    实测：Bed_01 顶点范围 183cm×198cm，×unit(0.01) → 1.83m×1.98m ✔
          ChairA_01 顶点 462×483×917cm ×node(0.1) ×unit(0.01)
                                  → 0.46m×0.48m×0.92m ✔（椅子尺寸）
    本工具按 COLLADA 语义走：顶点 → 场景节点矩阵链（列主序 <matrix>）→ × unit。
  * collision <pose> 存在时（本世界只有 Carpet_01 有 <pose>0 0 -0.01</pose>）先应用。
  * <mesh><scale> 存在时（Carpet_01 = 1 1 0.95、FloorB_01 = 1 1 0.9）逐轴乘顶点。
  * <box> 用 8 角点投影凸包（旋转矩形，精确轮廓）；<cylinder>/<sphere> 用 24 边形
    圆盘近似（footprints.json 的 bbox 记外接方框）；<mesh> 用三角面投影并集。
  * 全部按「格心落在几何内部」判定，不做额外膨胀（--dilate 可显式加膨胀）。
    因此细于半格（<2.5cm）的薄片可能不占格——保守选择，宁可判「无真值支撑」
    也不凭空给真值加覆盖。

命令行
======
  python3 harness/gt_footprints.py [--res 0.05] [--out logs/gt]
                                   [--world <world文件>] [--model-root <模型库>]
                                   [--origin x y] [--bounds-max x y] [--dilate N]
                                   [--collada-matrix-order row|col]
"""

from __future__ import annotations

import argparse
import json
import math
import os
import struct
import sys
import time
import xml.etree.ElementTree as ET
from datetime import datetime

# ---------------------------------------------------------------- numpy 可用性
try:
    import numpy as np
except Exception as _exc:  # pragma: no cover - 只在缺依赖时走到
    sys.stderr.write(
        "错误：本工具需要 numpy（用于栅格化世界真值足迹），但导入失败：\n"
        f"  {_exc}\n"
        "请先安装：pip3 install numpy   或  sudo apt install python3-numpy\n"
    )
    sys.exit(2)

# ---------------------------------------------------------------- 默认路径常量
REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_WORLD = "/home/kbf/stemm_sim_ws/src/wheeltec_two_wheel_model/world/aws_small_house.world"
DEFAULT_MODEL_ROOT = os.path.expanduser("~/.gazebo/models")
DEFAULT_OUT = os.path.join(REPO_ROOT, "logs", "gt")

DEFAULT_RES = 0.05                # 栅格分辨率 [m]
DEFAULT_ORIGIN = (-9.7, -6.0)     # 栅格左下角（世界坐标）
DEFAULT_BOUNDS_MAX = (10.0, 7.0)  # 栅格至少覆盖到的右上角世界坐标

LOW_BAND = (0.05, 0.35)           # 低矮带 [m]：能过但会撞、2D 视觉/激光最关心的区间
SOLID_Z_MIN = LOW_BAND[0]         # bit3「地面以上实体」的 z_max 阈值
HIGH_BAND_MIN = LOW_BAND[1]       # bit2「高带」的起点

CIRCLE_SEGMENTS = 24              # 圆盘 / 球体近似多边形边数
EPS_AREA = 1e-12                  # 三角面退化判定阈值 [m^2]

# COLLADA <matrix> 的主序。规范说列主序，但这批 AWS 模型的 DAE 是 FBX 导出器写的，
# 实测**行主序**才和 world 文件里的位姿自洽。证据（本工具自己的解析，可复算）：
#   判据 = 「网格世界 z 最低点」是否等于 world <pose> 的 z（作者是把物体底面贴到该 z 摆放的）
#     行主序：73/85 命中（85.9%）   列主序：41/85 命中（48.2%）
#   例：Handle_01_001 pose z=0.994 → 行主序 z_min=0.994 ✔ / 列主序 0.899 ✗
#       Refrigerator_01_001       → 行主序 z∈[0, 1.961]（1.96m 高，立着）✔
#                                     列主序 z∈[-1.961, 0]（整台埋进地板）✗
#       DeskPortraitD_02 pose z=0.192 → 行主序 z_min=0.192 ✔ / 列主序 0.060 ✗
# 可用 --collada-matrix-order col 切回规范读法（不推荐）。
MATRIX_ORDER_DEFAULT = "row"


# ================================================================ 基础数学工具
def rot_matrix(roll: float, pitch: float, yaw: float) -> "np.ndarray":
    """SDF 约定：R = Rz(yaw) * Ry(pitch) * Rx(roll)。"""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]], dtype=float)
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]], dtype=float)
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]], dtype=float)
    return rz @ ry @ rx


def pose_to_matrix(xyz, rpy) -> "np.ndarray":
    """(x,y,z) + (roll,pitch,yaw) → 4x4 齐次变换。"""
    m = np.eye(4, dtype=float)
    m[:3, :3] = rot_matrix(float(rpy[0]), float(rpy[1]), float(rpy[2]))
    m[:3, 3] = np.array([float(xyz[0]), float(xyz[1]), float(xyz[2])], dtype=float)
    return m


def apply_matrix(m: "np.ndarray", pts: "np.ndarray") -> "np.ndarray":
    """对 (N,3) 点集应用 4x4 齐次变换，返回 (N,3)。"""
    pts = np.asarray(pts, dtype=float).reshape(-1, 3)
    homo = np.hstack([pts, np.ones((pts.shape[0], 1), dtype=float)])
    return (homo @ m.T)[:, :3]


def parse_pose_text(text, where: str, warnings: list):
    """解析 SDF <pose> 文本 'x y z roll pitch yaw'（也容忍只给 3 个数的写法）。"""
    if text is None:
        return (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)
    try:
        vals = [float(t) for t in text.split()]
    except ValueError:
        warnings.append(f"{where}: <pose> 无法解析为浮点数：{text!r}，按全 0 处理")
        return (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)
    if len(vals) >= 6:
        return tuple(vals[0:3]), tuple(vals[3:6])
    if len(vals) == 3:
        return tuple(vals[0:3]), (0.0, 0.0, 0.0)
    warnings.append(f"{where}: <pose> 字段数异常（{len(vals)}）：{text!r}，缺失部分按 0 处理")
    vals = (vals + [0.0] * 6)[:6]
    return tuple(vals[0:3]), tuple(vals[3:6])


def parse_vec3_text(text, default, where: str, warnings: list):
    """解析 'a b c'；失败时退回 default 并记警告（不静默猜）。"""
    if text is None:
        return tuple(float(v) for v in default)
    try:
        vals = [float(t) for t in text.split()]
    except ValueError:
        warnings.append(f"{where}: 向量无法解析为浮点数：{text!r}，退回默认值 {default}")
        return tuple(float(v) for v in default)
    if len(vals) != 3:
        warnings.append(f"{where}: 向量应为 3 个数，实际 {len(vals)} 个：{text!r}，退回默认值 {default}")
        return tuple(float(v) for v in default)
    return (vals[0], vals[1], vals[2])


def convex_hull_2d(points: "np.ndarray") -> "np.ndarray":
    """Andrew 单调链凸包（返回 CCW）。输入 (N,2) → 输出 (M,2)。"""
    pts = np.unique(np.round(np.asarray(points, dtype=float).reshape(-1, 2), 9), axis=0)
    if pts.shape[0] <= 2:
        return pts
    order = np.lexsort((pts[:, 1], pts[:, 0]))
    pts = pts[order]

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower = []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    upper = []
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    hull = np.array(lower[:-1] + upper[:-1], dtype=float)
    if hull.shape[0] < 3:
        return pts
    area = 0.0
    for i in range(hull.shape[0]):
        x1, y1 = hull[i]
        x2, y2 = hull[(i + 1) % hull.shape[0]]
        area += x1 * y2 - x2 * y1
    return hull if area > 0 else hull[::-1]


def bbox3_of(points: "np.ndarray"):
    """(N,3) → (min3, max3)。"""
    pts = np.asarray(points, dtype=float).reshape(-1, 3)
    return pts.min(axis=0), pts.max(axis=0)


# ================================================================ 网格文件解析
def _strip_namespaces(root: ET.Element):
    """去掉 DAE 的 XML 命名空间，方便用裸标签名查找。"""
    for el in root.iter():
        if isinstance(el.tag, str) and "}" in el.tag:
            el.tag = el.tag.split("}", 1)[1]
    return root


def _dae_node_matrix(node: ET.Element, warnings: list, where: str) -> "np.ndarray":
    """
    合成 COLLADA <node> 内的 transform 元素（4x4，按出现顺序左乘）。
    <matrix> 是列主序 16 个数；<rotate> 是 (x y z 角度)。
    """
    m = np.eye(4, dtype=float)
    for child in list(node):
        try:
            if child.tag == "matrix":
                vals = [float(t) for t in (child.text or "").split()]
                if len(vals) != 16:
                    warnings.append(f"{where}: <matrix> 应有 16 个数，实际 {len(vals)} 个，已忽略")
                    continue
                mat = np.array(vals, dtype=float).reshape(4, 4)
                m = m @ (mat if MATRIX_ORDER_DEFAULT == "row" else mat.T)
            elif child.tag == "translate":
                vals = [float(t) for t in (child.text or "").split()]
                if len(vals) == 3:
                    t = np.eye(4, dtype=float)
                    t[:3, 3] = vals
                    m = m @ t
            elif child.tag == "scale":
                vals = [float(t) for t in (child.text or "").split()]
                if len(vals) == 3:
                    s = np.eye(4, dtype=float)
                    s[0, 0], s[1, 1], s[2, 2] = vals
                    m = m @ s
            elif child.tag == "rotate":
                vals = [float(t) for t in (child.text or "").split()]
                if len(vals) == 4:
                    axis = np.array(vals[0:3], dtype=float)
                    norm = float(np.linalg.norm(axis))
                    if norm <= 1e-12:
                        continue
                    axis = axis / norm
                    ang = math.radians(vals[3])
                    ca, sa = math.cos(ang), math.sin(ang)
                    cross = np.array([[0.0, -axis[2], axis[1]],
                                      [axis[2], 0.0, -axis[0]],
                                      [-axis[1], axis[0], 0.0]], dtype=float)
                    r = np.eye(3) * ca + sa * cross + (1.0 - ca) * np.outer(axis, axis)
                    t = np.eye(4, dtype=float)
                    t[:3, :3] = r
                    m = m @ t
        except ValueError:
            warnings.append(f"{where}: <{child.tag}> 含非浮点数值，已忽略该 transform")
    return m


def _dae_collect_instances(scene_root: ET.Element, warnings: list, where: str, out: dict):
    """遍历 visual_scene 的 node 树，记录 geometry_id → [累积矩阵, ...]。"""

    def walk(node: ET.Element, parent_m: "np.ndarray"):
        m = parent_m @ _dae_node_matrix(node, warnings, where)
        for inst in node.findall("instance_geometry"):
            url = (inst.get("url") or "").lstrip("#")
            if url:
                out.setdefault(url, []).append(m)
        for sub in node.findall("node"):
            walk(sub, m)

    for scene in scene_root.iter("visual_scene"):
        for node in scene.findall("node"):
            walk(node, np.eye(4, dtype=float))


def _dae_position_array(geom: ET.Element):
    """按 <vertices><input semantic='POSITION'> 取顶点数组；失败时退化并提示。"""
    src_id = None
    for v in geom.iter("vertices"):
        for inp in v.findall("input"):
            if (inp.get("semantic") or "").upper() == "POSITION":
                src_id = (inp.get("source") or "").lstrip("#")
                break
        if src_id:
            break

    arrays = {}
    for src in geom.iter("source"):
        fa = src.find("float_array")
        if fa is None:
            continue
        try:
            vals = np.array((fa.text or "").split(), dtype=float)
        except ValueError:
            continue
        if vals.size:
            arrays[src.get("id") or ""] = vals

    if src_id and src_id in arrays and arrays[src_id].size >= 3:
        return arrays[src_id], src_id, "POSITION 语义"
    for sid, vals in arrays.items():
        if "position" in sid.lower() and vals.size >= 3:
            return vals, sid, "id 启发式（缺 <vertices> POSITION 语义）"
    for sid, vals in arrays.items():
        if vals.size >= 3:
            return vals, sid, "首个 float_array（无 POSITION 标记）"
    return None, None, "未找到顶点数组"


def _dae_triangles(geom: ET.Element):
    """
    从 <triangles>/<polylist>/<polygons> 取三角面索引（polylist 按 vcount 扇形三角化）。
    返回 (T,3) int 数组（索引指向 POSITION 数组）；取不到时返回空数组。
    """
    tris = []
    # 注意：<polylist>/<triangles> 是 <geometry><mesh> 的子元素，必须 iter 到后代
    prims = []
    for tag in ("triangles", "polylist", "polygons"):
        prims.extend(geom.iter(tag))
    for prim in prims:
        inputs = prim.findall("input")
        if not inputs:
            continue
        pos_off = None
        for inp in inputs:
            if (inp.get("semantic") or "").upper() in ("VERTEX", "POSITION"):
                pos_off = int(inp.get("offset") or 0)
                break
        if pos_off is None:
            continue
        try:
            stride = max(int(i.get("offset") or 0) for i in inputs) + 1
        except ValueError:
            continue
        p_el = prim.find("p")
        if p_el is None or not (p_el.text or "").strip():
            continue
        try:
            idx = np.array((p_el.text or "").split(), dtype=int)
        except ValueError:
            continue
        if idx.size < stride:
            continue
        idx = idx[: (idx.size // stride) * stride].reshape(-1, stride)[:, pos_off]

        if prim.tag == "triangles":
            n = (idx.size // 3) * 3
            tris.extend(idx[:n].reshape(-1, 3).tolist())
            continue
        vcount_el = prim.find("vcount")
        if vcount_el is None or not (vcount_el.text or "").strip():
            n = (idx.size // 3) * 3
            tris.extend(idx[:n].reshape(-1, 3).tolist())
            continue
        try:
            vc = np.array(vcount_el.text.split(), dtype=int)
        except ValueError:
            continue
        k = 0
        for n in vc:
            fan = idx[k:k + int(n)]
            k += int(n)
            for j in range(1, fan.size - 1):
                tris.append([int(fan[0]), int(fan[j]), int(fan[j + 1])])
    return np.array(tris, dtype=int) if tris else np.zeros((0, 3), dtype=int)


def dae_mesh(path: str, warnings: list, where: str):
    """
    解析 COLLADA .dae → {"batches":[(verts(N,3) 米, tris(T,3))], "vertices":N,
    "triangles":T, "unit_m":u, "min":.., "max":..}
    顶点已按 (场景节点矩阵链) × unit 烘焙到「模型局部米」。
    """
    root = _strip_namespaces(ET.parse(path).getroot())

    unit_m = 1.0
    asset = root.find("asset")
    if asset is None:
        warnings.append(f"{where}: COLLADA 缺少 <asset>，按米（unit=1.0）处理")
    else:
        unit_el = asset.find("unit")
        if unit_el is not None and unit_el.get("meter"):
            try:
                unit_m = float(unit_el.get("meter"))
            except ValueError:
                warnings.append(f"{where}: <unit meter> 非浮点数，按 1.0 处理")
        up_axis = asset.find("up_axis")
        if up_axis is not None and (up_axis.text or "").strip().upper() not in ("Z_UP", ""):
            warnings.append(f"{where}: up_axis={(up_axis.text or '').strip()}（非 Z_UP），"
                            "本工具不做 Y_UP→Z_UP 旋转，XY 足迹可能偏差")

    insts = {}
    _dae_collect_instances(root, warnings, where, insts)

    geoms = list(root.iter("geometry"))
    if not geoms:
        return None

    batches = []
    nverts_total = 0
    ntris_total = 0
    mins, maxs = [], []
    for geom in geoms:
        gid = geom.get("id") or ""
        vals, _sid, how = _dae_position_array(geom)
        if vals is None:
            warnings.append(f"{where}: geometry {gid!r} 找不到顶点数组，已跳过（{how}）")
            continue
        n = vals.size // 3
        pts = vals[: n * 3].reshape(n, 3)
        tris = _dae_triangles(geom)
        if tris.size:
            ok = np.all(tris < n, axis=1)
            if not ok.all():
                warnings.append(f"{where}: geometry {gid!r} 有 {int((~ok).sum())} 个三角面"
                                "索引越界，已丢弃")
                tris = tris[ok]
        mats = insts.get(gid)
        if not mats:
            warnings.append(f"{where}: geometry {gid!r} 未在 visual_scene 里实例化，"
                            f"按单位矩阵处理（{how}）")
            mats = [np.eye(4, dtype=float)]
        for m in mats:
            baked = apply_matrix(m, pts) * unit_m   # 顶点 → 节点矩阵链 → × unit
            batches.append((baked, tris))
            nverts_total += int(baked.shape[0])
            ntris_total += int(tris.shape[0])
            mins.append(baked.min(axis=0))
            maxs.append(baked.max(axis=0))

    if not mins:
        return None
    return {
        "path": os.path.abspath(path),
        "batches": batches,
        "vertices": nverts_total,
        "triangles": ntris_total,
        "unit_m": unit_m,
        "min": np.min(np.vstack(mins), axis=0),
        "max": np.max(np.vstack(maxs), axis=0),
    }


def stl_mesh(path: str, warnings: list, where: str):
    """解析 STL（二进制/ASCII 自动识别）→ 与 dae_mesh 同构的 dict。"""
    with open(path, "rb") as fh:
        data = fh.read()
    if len(data) >= 84:
        n_tri = struct.unpack("<I", data[80:84])[0]
        if n_tri > 0 and len(data) == 84 + 50 * n_tri:
            buf = np.frombuffer(data[84:84 + 50 * n_tri], dtype=np.uint8).reshape(n_tri, 50)
            verts = np.zeros((n_tri, 3, 3), dtype=float)
            for k in range(3):
                verts[:, k, :] = buf[:, 12 + 12 * k: 24 + 12 * k].copy().view("<f4")
            pts = verts.reshape(-1, 3)
            tris = np.arange(pts.shape[0], dtype=int).reshape(-1, 3)
            return {"path": os.path.abspath(path), "batches": [(pts, tris)],
                    "vertices": int(pts.shape[0]), "triangles": int(tris.shape[0]),
                    "unit_m": 1.0, "min": pts.min(axis=0), "max": pts.max(axis=0)}
    pts = []
    with open(path, "r", errors="replace") as fh:
        for line in fh:
            s = line.strip()
            if s.lower().startswith("vertex"):
                toks = s.split()
                if len(toks) >= 4:
                    try:
                        pts.append([float(toks[1]), float(toks[2]), float(toks[3])])
                    except ValueError:
                        pass
    if not pts or len(pts) % 3:
        warnings.append(f"{where}: STL 既不是合法二进制也不含完整 ASCII 三角面"
                        f"（已得 {len(pts)} 个顶点）")
        return None
    arr = np.array(pts, dtype=float)
    tris = np.arange(arr.shape[0], dtype=int).reshape(-1, 3)
    return {"path": os.path.abspath(path), "batches": [(arr, tris)],
            "vertices": int(arr.shape[0]), "triangles": int(tris.shape[0]),
            "unit_m": 1.0, "min": arr.min(axis=0), "max": arr.max(axis=0)}


def obj_mesh(path: str, warnings: list, where: str):
    """解析 Wavefront OBJ（v 行 + f 行，多边形按扇形三角化）→ 同构 dict。"""
    pts = []
    faces = []
    with open(path, "r", errors="replace") as fh:
        for line in fh:
            s = line.strip()
            if s.startswith("v "):
                toks = s.split()
                if len(toks) >= 4:
                    try:
                        pts.append([float(toks[1]), float(toks[2]), float(toks[3])])
                    except ValueError:
                        pass
            elif s.startswith("f "):
                verts = []
                for tok in s.split()[1:]:
                    try:
                        vi = int(tok.split("/")[0])
                    except ValueError:
                        continue
                    verts.append(vi - 1 if vi > 0 else vi)
                for j in range(1, len(verts) - 1):
                    faces.append([verts[0], verts[j], verts[j + 1]])
    if not pts:
        warnings.append(f"{where}: OBJ 里没有 'v' 顶点")
        return None
    arr = np.array(pts, dtype=float)
    tris = np.array(faces, dtype=int) if faces else np.zeros((0, 3), dtype=int)
    if tris.size:
        tris = tris[np.all((tris >= 0) & (tris < arr.shape[0]), axis=1)]
    if tris.shape[0] == 0:
        warnings.append(f"{where}: OBJ 没有可用 'f' 面，退化为顶点凸包足迹")
    return {"path": os.path.abspath(path), "batches": [(arr, tris)],
            "vertices": int(arr.shape[0]), "triangles": int(tris.shape[0]),
            "unit_m": 1.0, "min": arr.min(axis=0), "max": arr.max(axis=0)}


def mesh_cached(path: str, cache: dict, stats: dict, warnings: list, where: str):
    """网格解析入口（进程内缓存：同一模型被 include 多次时只解析一次）。"""
    key = os.path.abspath(path)
    if key in cache:
        stats["mesh_cache_hits"] += 1
        return cache[key]

    stats["mesh_cache_misses"] += 1
    ext = os.path.splitext(path)[1].lower()
    try:
        if ext == ".dae":
            res = dae_mesh(path, warnings, where)
        elif ext == ".stl":
            res = stl_mesh(path, warnings, where)
        elif ext == ".obj":
            res = obj_mesh(path, warnings, where)
        else:
            warnings.append(f"{where}: 不支持的网格后缀 {ext!r}（只支持 .dae/.stl/.obj）")
            res = None
    except Exception as exc:
        warnings.append(f"{where}: 网格解析异常 {type(exc).__name__}: {exc}")
        res = None
    cache[key] = res
    return res


# ================================================================ 模型 SDF 解析
def resolve_mesh_path(uri: str, model_dir: str) -> str:
    """把 model:// / file:// / 相对路径统一成本地绝对路径。"""
    u = uri.strip()
    if u.startswith("model://"):
        return os.path.join(DEFAULT_MODEL_ROOT, u[len("model://"):].lstrip("/"))
    if u.startswith("file://models/"):
        return os.path.join(DEFAULT_MODEL_ROOT, u[len("file://models/"):].lstrip("/"))
    if u.startswith("file://"):
        p = u[len("file://"):]
        return p if os.path.isabs(p) else os.path.join(model_dir, p)
    if os.path.isabs(u):
        return u
    return os.path.join(model_dir, u)


def _find_all_links(model_el: ET.Element):
    """递归收集 <link>（本世界都是单 link，按 SDF 语义递归更稳）。"""
    links = list(model_el.findall("link"))
    for sub in model_el.findall("model"):
        links.extend(_find_all_links(sub))
    return links


def load_model_collisions(model_name: str, model_root: str, cache: dict, stats: dict, warnings: list):
    """
    解析 <model_root>/<model_name>/model.sdf，返回该模型所有 collision：
      [{name, link, local_matrix(link@collision pose), geom:{...}}]
    任何一步失败都写进 stats["failures"] 并跳过，绝不猜测填充。
    """
    if model_name in cache:
        stats["model_cache_hits"] += 1
        return cache[model_name]

    stats["model_cache_misses"] += 1
    model_dir = os.path.join(model_root, model_name)
    sdf_path = os.path.join(model_dir, "model.sdf")
    if not os.path.isfile(sdf_path):
        reason = ("模型目录不存在" if not os.path.isdir(model_dir) else "缺 model.sdf")
        stats["failures"].append({"model": model_name, "stage": "加载模型 SDF",
                                  "reason": reason, "path": sdf_path})
        cache[model_name] = None
        return None
    try:
        root = ET.parse(sdf_path).getroot()
    except Exception as exc:
        stats["failures"].append({"model": model_name, "stage": "XML 解析 model.sdf",
                                  "reason": f"{type(exc).__name__}: {exc}", "path": sdf_path})
        cache[model_name] = None
        return None

    collisions = []
    for model_el in root.iter("model"):
        mpose_el = model_el.find("pose")
        model_pose_m = pose_to_matrix(*parse_pose_text(
            mpose_el.text if mpose_el is not None else None,
            f"{model_name}/model.sdf <model>.pose", warnings))
        for link in _find_all_links(model_el):
            lpose_el = link.find("pose")
            link_m = pose_to_matrix(*parse_pose_text(
                lpose_el.text if lpose_el is not None else None,
                f"{model_name}/model.sdf <link {link.get('name')}>.pose", warnings))
            base_m = model_pose_m @ link_m
            for col in link.findall("collision"):
                cname = col.get("name") or "collision"
                where = f"{model_name}/model.sdf collision {cname!r}"
                cpose_el = col.find("pose")
                cxyz, crpy = parse_pose_text(cpose_el.text if cpose_el is not None else None,
                                             where + ".pose", warnings)
                local_m = base_m @ pose_to_matrix(cxyz, crpy)

                geom_el = col.find("geometry")
                if geom_el is None:
                    stats["failures"].append({"model": model_name, "collision": cname,
                                              "stage": "几何解析", "reason": "collision 无 <geometry>"})
                    continue
                kinds = [c.tag for c in list(geom_el)]
                if not kinds:
                    stats["failures"].append({"model": model_name, "collision": cname,
                                              "stage": "几何解析", "reason": "<geometry> 为空"})
                    continue
                kind = kinds[0]
                geom = {"type": kind}

                if kind == "box":
                    size_el = geom_el.find("box/size")
                    geom["size"] = parse_vec3_text(size_el.text if size_el is not None else None,
                                                   (0.0, 0.0, 0.0), where + ".box.size", warnings)
                    if min(geom["size"]) <= 0.0:
                        stats["failures"].append({"model": model_name, "collision": cname,
                                                  "stage": "几何解析",
                                                  "reason": f"box size 非法 {geom['size']}"})
                        continue
                elif kind == "cylinder":
                    r_el = geom_el.find("cylinder/radius")
                    l_el = geom_el.find("cylinder/length")
                    try:
                        geom["radius"] = float((r_el.text or "").strip())
                        geom["length"] = float((l_el.text or "").strip())
                    except (AttributeError, ValueError):
                        stats["failures"].append({"model": model_name, "collision": cname,
                                                  "stage": "几何解析",
                                                  "reason": "cylinder radius/length 缺失或非法"})
                        continue
                elif kind == "sphere":
                    r_el = geom_el.find("sphere/radius")
                    try:
                        geom["radius"] = float((r_el.text or "").strip())
                    except (AttributeError, ValueError):
                        stats["failures"].append({"model": model_name, "collision": cname,
                                                  "stage": "几何解析",
                                                  "reason": "sphere radius 缺失或非法"})
                        continue
                elif kind == "mesh":
                    uri_el = geom_el.find("mesh/uri")
                    if uri_el is None or not (uri_el.text or "").strip():
                        stats["failures"].append({"model": model_name, "collision": cname,
                                                  "stage": "几何解析", "reason": "<mesh> 缺 <uri>"})
                        continue
                    uri = uri_el.text.strip()
                    scale_el = geom_el.find("mesh/scale")
                    geom["uri"] = uri
                    geom["scale"] = parse_vec3_text(
                        scale_el.text if scale_el is not None else None, (1.0, 1.0, 1.0),
                        where + ".mesh.scale", warnings)
                    fpath = resolve_mesh_path(uri, model_dir)
                    geom["file"] = fpath
                    if not os.path.isfile(fpath):
                        stats["failures"].append({"model": model_name, "collision": cname,
                                                  "stage": "网格解析",
                                                  "reason": f"网格文件不存在: {fpath}", "uri": uri})
                        geom["error"] = "网格文件不存在"
                    else:
                        mesh = mesh_cached(fpath, stats["mesh_cache"], stats, warnings, where)
                        if mesh is None:
                            stats["failures"].append({"model": model_name, "collision": cname,
                                                      "stage": "网格解析",
                                                      "reason": f"无法从 {fpath} 解出顶点", "uri": uri})
                            geom["error"] = "网格顶点解析失败"
                        else:
                            geom["mesh"] = mesh
                else:
                    stats["failures"].append({"model": model_name, "collision": cname,
                                              "stage": "几何解析",
                                              "reason": f"暂不支持的几何类型 <{kind}>"
                                                        "（只支持 box/cylinder/sphere/mesh）"})
                    continue

                collisions.append({"name": cname, "link": link.get("name"),
                                   "local_matrix": local_m, "geom": geom})

    if not collisions:
        stats["failures"].append({"model": model_name, "stage": "模型汇总",
                                  "reason": "该模型没有任何可用 collision"})
        cache[model_name] = None
        return None
    cache[model_name] = collisions
    return collisions


# ================================================================ 栅格化
def _cell_window(xmin, xmax, ymin, ymax, res, origin, shape):
    """几何世界 bbox → 需要检查的格子窗口 (r0, r1, c0, c1)（含端点，已裁剪）。"""
    h, w = shape
    ox, oy = origin
    c0 = int(max(0, math.floor((xmin - ox) / res)))
    c1 = int(min(w - 1, math.ceil((xmax - ox) / res)))
    r0 = int(max(0, math.floor((ymin - oy) / res)))
    r1 = int(min(h - 1, math.ceil((ymax - oy) / res)))
    return r0, r1, c0, c1


def _grid_centers(r0, r1, c0, c1, res, origin):
    """该窗口的格心坐标网格 (gx, gy)，形状 (r1-r0+1, c1-c0+1)。"""
    cols = np.arange(c0, c1 + 1)
    rows = np.arange(r0, r1 + 1)
    gx = origin[0] + (cols + 0.5) * res
    gy = origin[1] + (rows + 0.5) * res
    return np.meshgrid(gx, gy)


def rasterize_convex_polygon(poly: "np.ndarray", patch: "np.ndarray", r0, r1, c0, c1, res, origin):
    """把 CCW 凸多边形按「格心在内」写进 patch（patch 已按窗口对齐）。"""
    if poly.shape[0] < 3 or patch.size == 0:
        return
    gx, gy = _grid_centers(r0, r1, c0, c1, res, origin)
    inside = np.ones(gx.shape, dtype=bool)
    n = poly.shape[0]
    for i in range(n):
        ax, ay = poly[i]
        bx, by = poly[(i + 1) % n]
        inside &= ((bx - ax) * (gy - ay) - (by - ay) * (gx - ax)) >= 0.0
    patch |= inside


def rasterize_triangles(verts_xy: "np.ndarray", tris: "np.ndarray", patch: "np.ndarray",
                        r0, r1, c0, c1, res, origin, stats: dict):
    """三角面投影并集 → patch。完全退化的三角面（竖直面投影成线段）跳过并计数。"""
    if tris.size == 0 or patch.size == 0:
        return
    gx, gy = _grid_centers(r0, r1, c0, c1, res, origin)
    for tri in tris:
        p0, p1, p2 = verts_xy[tri[0]], verts_xy[tri[1]], verts_xy[tri[2]]
        area2 = (p1[0] - p0[0]) * (p2[1] - p0[1]) - (p2[0] - p0[0]) * (p1[1] - p0[1])
        if abs(area2) < EPS_AREA:
            stats["degenerate_tris"] += 1
            continue
        s = 1.0 if area2 > 0 else -1.0
        d0 = (p1[0] - p0[0]) * (gy - p0[1]) - (p1[1] - p0[1]) * (gx - p0[0])
        d1 = (p2[0] - p1[0]) * (gy - p1[1]) - (p2[1] - p1[1]) * (gx - p1[0])
        d2 = (p0[0] - p2[0]) * (gy - p2[1]) - (p0[1] - p2[1]) * (gx - p2[0])
        m = (d0 * s >= 0.0) & (d1 * s >= 0.0) & (d2 * s >= 0.0)
        if m.any():
            patch |= m


def dilate_bits(grid: "np.ndarray", bit: int, radius: int):
    """把 grid 的某个 bit 按方形半径 radius 膨胀（就地）。"""
    if radius <= 0:
        return
    acc = (grid & bit).astype(bool)
    if not acc.any():
        return
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
    grid[acc] |= np.uint8(bit)


# ================================================================ 主流程
def build(world_path: str, model_root: str, res: float, origin, bounds_max, dilate: int):
    t_start = time.time()
    warnings: list = []
    stats = {
        "mesh_cache": {},
        "_model_cache": {},
        "mesh_cache_hits": 0,
        "mesh_cache_misses": 0,
        "model_cache_hits": 0,
        "model_cache_misses": 0,
        "degenerate_tris": 0,
        "failures": [],
    }

    if origin[0] > bounds_max[0] or origin[1] > bounds_max[1]:
        raise SystemExit(f"错误：origin {origin} 不在 bounds_max {bounds_max} 的左下方，无法建网格")
    width = int(max(1, math.ceil((bounds_max[0] - origin[0]) / res - 1e-9)))
    height = int(max(1, math.ceil((bounds_max[1] - origin[1]) / res - 1e-9)))
    shape = (height, width)
    grid = np.zeros(shape, dtype=np.uint8)

    if not os.path.isfile(world_path):
        raise SystemExit(f"错误：world 文件不存在：{world_path}")
    try:
        world_root = ET.parse(world_path).getroot()
    except Exception as exc:
        raise SystemExit(f"错误：world 文件 XML 解析失败：{type(exc).__name__}: {exc}")
    world_el = world_root.find("world")
    if world_el is None:
        raise SystemExit(f"错误：{world_path} 里没有 <world> 元素")

    instances = []
    n_include_seen = 0
    n_include_skipped = 0
    n_collision_total = 0
    n_collision_ok = 0
    n_collision_mesh = 0
    ground_like = []          # 被 bit3 排除的「纯地面铺装」碰撞体（连理由一起输出）

    for model_el in world_el.findall("model"):
        inc_el = model_el.find("include")
        if inc_el is None:
            warnings.append(f"world 里 <model name={model_el.get('name')!r}> 没有 <include>"
                            "（内联几何），本工具只解析 include 模型，已跳过")
            n_include_skipped += 1
            continue
        n_include_seen += 1

        instance_name = model_el.get("name") or f"include_{n_include_seen}"
        uri_el = inc_el.find("uri")
        if uri_el is None or not (uri_el.text or "").strip():
            stats["failures"].append({"instance": instance_name, "stage": "world include",
                                      "reason": "<include> 缺 <uri>"})
            n_include_skipped += 1
            continue
        uri = uri_el.text.strip()
        model_name = uri.split("://", 1)[-1].strip("/").split("/")[0] or uri

        m_pose_el = model_el.find("pose")
        m_xyz, m_rpy = parse_pose_text(m_pose_el.text if m_pose_el is not None else None,
                                       f"world/model {instance_name}.pose", warnings)
        i_pose_el = inc_el.find("pose")
        i_xyz, i_rpy = parse_pose_text(i_pose_el.text if i_pose_el is not None else None,
                                       f"world/include {instance_name}.pose", warnings)
        world_m = pose_to_matrix(m_xyz, m_rpy) @ pose_to_matrix(i_xyz, i_rpy)

        if (abs(m_rpy[0]) > 1e-6 or abs(m_rpy[1]) > 1e-6
                or abs(i_rpy[0]) > 1e-6 or abs(i_rpy[1]) > 1e-6):
            warnings.append(f"{instance_name}: 世界位姿含非零 roll/pitch "
                            f"(roll={m_rpy[0] + i_rpy[0]:.6f}, pitch={m_rpy[1] + i_rpy[1]:.6f})，"
                            "XY 足迹按旋转后的投影（俯仰/横滚会略微放大足迹）")

        inc_scale_el = inc_el.find("scale")
        inc_scale = parse_vec3_text(inc_scale_el.text if inc_scale_el is not None else None,
                                    (1.0, 1.0, 1.0), f"world/include {instance_name}.scale", warnings)
        scale_m = np.eye(4, dtype=float)
        scale_m[0, 0], scale_m[1, 1], scale_m[2, 2] = inc_scale
        has_include_scale = inc_scale_el is not None

        collisions = load_model_collisions(model_name, model_root,
                                           stats["_model_cache"], stats, warnings)
        if collisions is None:
            n_include_skipped += 1
            continue

        inst = {
            "instance": instance_name,
            "model": model_name,
            "uri": uri,
            "world_pose": {"xyz": [float(v) for v in m_xyz], "rpy": [float(v) for v in m_rpy]},
            "include_pose": {"xyz": [float(v) for v in i_xyz], "rpy": [float(v) for v in i_rpy]},
            "include_scale": [float(v) for v in inc_scale],
            "collisions": [],
        }

        for col in collisions:
            geom = col["geom"]
            n_collision_total += 1
            if geom.get("error"):
                continue

            m_full = world_m @ (scale_m if has_include_scale else np.eye(4)) @ col["local_matrix"]
            kind = geom["type"]
            shape_kind = "polygon"
            mesh_info = {}

            try:
                if kind == "box":
                    sx, sy, sz = geom["size"]
                    hx, hy, hz = sx / 2.0, sy / 2.0, sz / 2.0
                    corners = np.array([[x * hx, y * hy, z * hz]
                                        for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)],
                                       dtype=float)
                    world_pts = apply_matrix(m_full, corners)
                    poly = convex_hull_2d(world_pts[:, :2])
                    desc = f"box {sx:.3f}x{sy:.3f}x{sz:.3f} m（8 角点投影凸包）"
                elif kind in ("cylinder", "sphere"):
                    if kind == "cylinder":
                        r, z_off = geom["radius"], geom["length"] / 2.0
                        desc = (f"cylinder r={r:.3f} L={geom['length']:.3f} m"
                                f"（{CIRCLE_SEGMENTS} 边形圆盘近似）")
                    else:
                        r, z_off = geom["radius"], geom["radius"]
                        desc = (f"sphere r={r:.3f} m（{CIRCLE_SEGMENTS} 边形圆盘近似，z 取 ±r）")
                    ang = np.linspace(0.0, 2.0 * math.pi, CIRCLE_SEGMENTS, endpoint=False)
                    ring = np.stack([r * np.cos(ang), r * np.sin(ang)], axis=1)
                    local_pts = np.vstack([
                        np.hstack([ring, np.full((CIRCLE_SEGMENTS, 1), -z_off)]),
                        np.hstack([ring, np.full((CIRCLE_SEGMENTS, 1), +z_off)]),
                    ])
                    world_pts = apply_matrix(m_full, local_pts)
                    poly = convex_hull_2d(world_pts[:, :2])
                elif kind == "mesh":
                    mesh = geom["mesh"]
                    scale = np.array(geom["scale"], dtype=float)
                    vb, tb, base = [], [], 0
                    for verts, tris in mesh["batches"]:
                        v = apply_matrix(m_full, verts * scale)
                        vb.append(v)
                        if tris.size:
                            tb.append(tris + base)
                        base += v.shape[0]
                    world_verts = np.vstack(vb)
                    tris_all = np.vstack(tb) if tb else np.zeros((0, 3), dtype=int)
                    world_pts = world_verts
                    desc = (f"mesh {os.path.basename(mesh['path'])} "
                            f"verts={mesh['vertices']} tris={mesh['triangles']} "
                            f"unit={mesh['unit_m']}")
                    mesh_info = {
                        "mesh_file": mesh["path"],
                        "mesh_vertices": int(mesh["vertices"]),
                        "mesh_triangles": int(mesh["triangles"]),
                        "mesh_unit_m": float(mesh["unit_m"]),
                        "mesh_scale": [float(v) for v in geom["scale"]],
                    }
                    if tris_all.shape[0] == 0:
                        warnings.append(f"{instance_name}/{col['name']}: mesh 无三角面，"
                                        "退化为顶点凸包足迹")
                        poly = convex_hull_2d(world_verts[:, :2])
                    else:
                        shape_kind = "triangles"
                        verts_xy = world_verts[:, :2]
                else:
                    raise ValueError(f"未知几何类型 {kind}")
            except Exception as exc:
                stats["failures"].append({"instance": instance_name, "model": model_name,
                                          "collision": col["name"], "stage": "几何展开",
                                          "reason": f"{type(exc).__name__}: {exc}"})
                continue

            mn3, mx3 = bbox3_of(world_pts)
            z_lo, z_hi = float(mn3[2]), float(mx3[2])
            in_low = (z_hi >= LOW_BAND[0]) and (z_lo <= LOW_BAND[1])
            in_high = z_hi > HIGH_BAND_MIN
            is_solid = z_hi > SOLID_Z_MIN     # bit3：地面以上有实体（剔除地板/地毯）

            r0, r1, c0, c1 = _cell_window(mn3[0], mx3[0], mn3[1], mx3[1], res, origin, shape)
            patch = np.zeros((max(0, r1 - r0 + 1), max(0, c1 - c0 + 1)), dtype=bool)
            if patch.size:
                if shape_kind == "polygon":
                    rasterize_convex_polygon(poly, patch, r0, r1, c0, c1, res, origin)
                else:
                    rasterize_triangles(verts_xy, tris_all, patch, r0, r1, c0, c1,
                                        res, origin, stats)
            n_cells = int(patch.sum())

            if n_cells:
                sub = grid[r0:r1 + 1, c0:c1 + 1]
                sub |= np.where(patch, np.uint8(0x01), np.uint8(0x00))      # bit0 任意 z
                if in_low:
                    sub |= np.where(patch, np.uint8(0x02), np.uint8(0x00))  # bit1 低矮带
                if in_high:
                    sub |= np.where(patch, np.uint8(0x04), np.uint8(0x00))  # bit2 高带
                if is_solid:
                    sub |= np.where(patch, np.uint8(0x08), np.uint8(0x00))  # bit3 地面以上实体

            if kind == "mesh":
                n_collision_mesh += 1
            inst["collisions"].append({
                "name": col["name"],
                "link": col["link"],
                "geom_type": kind,
                "geom_desc": desc,
                "is_mesh": kind == "mesh",
                "source_file": mesh_info.get("mesh_file") or
                               (geom.get("file") or os.path.join(model_root, model_name, "model.sdf")),
                "mesh_vertices": int(mesh_info.get("mesh_vertices", 0)),
                "mesh_triangles": int(mesh_info.get("mesh_triangles", 0)),
                "mesh_unit_m": mesh_info.get("mesh_unit_m"),
                "mesh_scale": mesh_info.get("mesh_scale", [1.0, 1.0, 1.0]),
                "bbox_world": {
                    "xmin": float(mn3[0]), "xmax": float(mx3[0]),
                    "ymin": float(mn3[1]), "ymax": float(mx3[1]),
                    "zmin": z_lo, "zmax": z_hi,
                },
                "low_band_hit": bool(in_low),
                "high_band_hit": bool(in_high),
                "solid_above_ground": bool(is_solid),
                "cells": n_cells,
            })
            n_collision_ok += 1
            if not is_solid:
                ground_like.append({
                    "instance": instance_name,
                    "model": model_name,
                    "collision": col["name"],
                    "z_range": [z_lo, z_hi],
                    "reason": f"z_max={z_hi:.3f} m ≤ {SOLID_Z_MIN} m：判定为地面铺装/地毯，"
                              "计入 bit0 但不计入 bit3",
                    "cells": n_cells,
                })

        instances.append(inst)

    # ---- 可选膨胀（默认 0：绝不给真值凭空加覆盖）
    if dilate > 0:
        for bit in (0x01, 0x02, 0x04, 0x08):
            dilate_bits(grid, bit, dilate)

    bit0 = int(((grid & 0x01) > 0).sum())
    bit1 = int(((grid & 0x02) > 0).sum())
    bit2 = int(((grid & 0x04) > 0).sum())
    bit3 = int(((grid & 0x08) > 0).sum())
    n_cells_total = int(shape[0] * shape[1])
    elapsed = time.time() - t_start

    stats_out = {
        "world_file": world_path,
        "model_root": model_root,
        "res": float(res),
        "origin": [float(origin[0]), float(origin[1])],
        "width": int(width),
        "height": int(height),
        "cells_total": n_cells_total,
        "include_total": n_include_seen,
        "include_ok": len(instances),
        "include_skipped": n_include_skipped,
        "collision_total": n_collision_total,
        "collision_ok": n_collision_ok,
        "collision_mesh": n_collision_mesh,
        "collision_non_mesh": n_collision_ok - n_collision_mesh,
        "models_resolved": len({i["model"] for i in instances}),
        "mesh_files_parsed": stats["mesh_cache_misses"],
        "mesh_cache_hits": stats["mesh_cache_hits"],
        "model_cache_hits": stats["model_cache_hits"],
        "degenerate_tris_skipped": stats["degenerate_tris"],
        "bit0_cells": bit0,
        "bit0_ratio": (bit0 / n_cells_total) if n_cells_total else 0.0,
        "bit1_low_band_cells": bit1,
        "bit1_ratio": (bit1 / n_cells_total) if n_cells_total else 0.0,
        "bit2_high_band_cells": bit2,
        "bit2_ratio": (bit2 / n_cells_total) if n_cells_total else 0.0,
        "bit3_solid_cells": bit3,
        "bit3_ratio": (bit3 / n_cells_total) if n_cells_total else 0.0,
        "failures": len(stats["failures"]),
        "dilate_cells": int(dilate),
        "elapsed_s": round(elapsed, 3),
        "low_band_m": [LOW_BAND[0], LOW_BAND[1]],
        "high_band_min_m": HIGH_BAND_MIN,
        "solid_z_min_m": SOLID_Z_MIN,
        "ground_like_collisions": ground_like,
    }
    return instances, grid, stats_out, stats["failures"], warnings


def write_outputs(out_dir: str, instances, grid, stats_out, failures, warnings,
                  world_path: str, model_root: str):
    os.makedirs(out_dir, exist_ok=True)
    grid_path = os.path.join(out_dir, "gt_grid.npy")
    meta_path = os.path.join(out_dir, "gt_grid.json")
    fp_path = os.path.join(out_dir, "footprints.json")

    np.save(grid_path, grid)
    gen_at = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

    meta = {
        "generated_at": gen_at,
        "tool": "harness/gt_footprints.py",
        "world_file": world_path,
        "model_root": model_root,
        "dtype": "uint8",
        "res": stats_out["res"],
        "origin": stats_out["origin"],
        "width": stats_out["width"],
        "height": stats_out["height"],
        "shape": [stats_out["height"], stats_out["width"]],
        "index_convention": "numpy 矩阵约定：行 0 = 世界 y 最小，列 0 = 世界 x 最小",
        "cell_center": "x = origin[0] + (col+0.5)*res ; y = origin[1] + (row+0.5)*res",
        "frame_note": "world 帧 == map 帧（机器人从世界原点 yaw=0 起图）",
        "bits": {
            "bit0": "有实体覆盖（任意 z，含地面铺装）——按需求定义，本世界被地板占满",
            "bit1": "低矮带覆盖：z 范围与 [0.05, 0.35] m 相交",
            "bit2": "高带覆盖：z 范围存在 > 0.35 m 的部分（诊断扩展位）",
            "bit3": "地面以上实体覆盖：z_max > 0.05 m（分类器实际使用的「真障碍」判据）",
        },
        "low_band_m": stats_out["low_band_m"],
        "solid_z_min_m": stats_out["solid_z_min_m"],
        "high_band_min_m": stats_out["high_band_min_m"],
        "raster_rule": ("格心落在碰撞体世界 XY 投影轮廓内即算覆盖；mesh 按三角面投影并集、"
                        f"box 按 8 角点凸包、cylinder/sphere 按 {CIRCLE_SEGMENTS} 边形圆盘；"
                        f"dilate={stats_out['dilate_cells']}"),
        "stats": stats_out,
    }
    with open(meta_path, "w", encoding="utf-8") as fh:
        json.dump(meta, fh, ensure_ascii=False, indent=2)

    fp = {
        "generated_at": gen_at,
        "world_file": world_path,
        "model_root": model_root,
        "coordinate_frames": {
            "world": "world 帧 == map 帧（机器人从世界原点 (0,0,0,yaw=0) 起图）",
            "pose_format": "SDF <pose> = x y z roll pitch yaw",
        },
        "low_band_m": stats_out["low_band_m"],
        "solid_z_min_m": stats_out["solid_z_min_m"],
        "raster": {"res": stats_out["res"], "origin": stats_out["origin"],
                   "shape": [stats_out["height"], stats_out["width"]],
                   "index_convention": meta["index_convention"],
                   "rule": meta["raster_rule"]},
        "stats": stats_out,
        "parse_failures": failures,
        "warnings": warnings,
        "instances": instances,
    }
    with open(fp_path, "w", encoding="utf-8") as fh:
        json.dump(fp, fh, ensure_ascii=False, indent=2)
    return grid_path, meta_path, fp_path


def print_summary(stats_out, failures, warnings, paths):
    print("=" * 70)
    print("世界真值 2D 足迹构建完成（离线解析，未启动任何 Gazebo/ROS 进程）")
    print("=" * 70)
    print(f"world 文件            : {stats_out['world_file']}")
    print(f"模型库                : {stats_out['model_root']}")
    print(f"解析 include          : {stats_out['include_total']} 个"
          f"（成功 {stats_out['include_ok']}，跳过 {stats_out['include_skipped']}）")
    print(f"解析 model 种类        : {stats_out['models_resolved']} 个"
          f"（模型 SDF 缓存命中 {stats_out['model_cache_hits']} 次）")
    print(f"collision 总数        : {stats_out['collision_total']} 个"
          f"（成功 {stats_out['collision_ok']}；mesh {stats_out['collision_mesh']}，"
          f"非 mesh {stats_out['collision_non_mesh']}）")
    print(f"网格文件实际解析      : {stats_out['mesh_files_parsed']} 组"
          f"（缓存命中 {stats_out['mesh_cache_hits']} 次）")
    print(f"解析失败              : {stats_out['failures']} 条")
    print("-" * 70)
    print(f"gt_grid               : {stats_out['width']}x{stats_out['height']} 格 @ "
          f"res={stats_out['res']} m，origin=({stats_out['origin'][0]}, {stats_out['origin'][1]})")
    print(f"  覆盖范围            : x∈[{stats_out['origin'][0]:.3f}, "
          f"{stats_out['origin'][0] + stats_out['width'] * stats_out['res']:.3f}]  "
          f"y∈[{stats_out['origin'][1]:.3f}, "
          f"{stats_out['origin'][1] + stats_out['height'] * stats_out['res']:.3f}]")
    print(f"  bit0 任意 z 实体     : {stats_out['bit0_cells']} 格"
          f"（{stats_out['bit0_ratio'] * 100:.2f}% 全域）")
    print(f"  bit1 低矮带 0.05~0.35: {stats_out['bit1_low_band_cells']} 格"
          f"（{stats_out['bit1_ratio'] * 100:.2f}%）")
    print(f"  bit2 高带 >0.35 m    : {stats_out['bit2_high_band_cells']} 格"
          f"（{stats_out['bit2_ratio'] * 100:.2f}%）")
    print(f"  bit3 地面以上实体    : {stats_out['bit3_solid_cells']} 格"
          f"（{stats_out['bit3_ratio'] * 100:.2f}%）← 分类器用这个")
    gl = stats_out["ground_like_collisions"]
    if gl:
        print(f"  注意：{len(gl)} 个碰撞体被判为「地面铺装/地毯」（z_max ≤ "
              f"{stats_out['solid_z_min_m']} m），计入 bit0 但不计入 bit3：")
        for g in gl:
            print(f"    - {g['instance']}/{g['collision']}  z∈[{g['z_range'][0]:.3f}, "
                  f"{g['z_range'][1]:.3f}]  {g['cells']} 格")
    print(f"耗时                  : {stats_out['elapsed_s']:.2f} s")
    print("-" * 70)
    print("输出：\n  " + "\n  ".join(paths))

    if failures:
        print("-" * 70)
        print(f"【解析失败清单（{len(failures)} 条，已显式跳过，未用任何猜测填充）】")
        for f in failures:
            who = f.get("instance") or f.get("model") or "?"
            col = f.get("collision")
            print(f"  - {who}{('/' + col) if col else ''} [{f.get('stage', '')}] {f.get('reason')}")
    if warnings:
        print("-" * 70)
        print(f"【警告（{len(warnings)} 条，全部写入 footprints.json）】")
        for w in warnings[:15]:
            print(f"  ! {w}")
        if len(warnings) > 15:
            print(f"  ... 其余 {len(warnings) - 15} 条见 footprints.json 的 warnings 字段")
    print("=" * 70)


def main(argv=None):
    global MATRIX_ORDER_DEFAULT
    ap = argparse.ArgumentParser(
        description="离线构建「世界真值 2D 足迹」位图（只读 SDF/网格文件，不启动任何 Gazebo/ROS 进程）",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    ap.add_argument("--world", default=DEFAULT_WORLD, help="world 文件路径（只读）")
    ap.add_argument("--model-root", default=DEFAULT_MODEL_ROOT, help="模型库根目录（只读）")
    ap.add_argument("--res", type=float, default=DEFAULT_RES, help="栅格分辨率 [m]")
    ap.add_argument("--out", default=DEFAULT_OUT, help="输出目录")
    ap.add_argument("--origin", type=float, nargs=2, default=list(DEFAULT_ORIGIN),
                    metavar=("OX", "OY"), help="栅格左下角世界坐标 [m]")
    ap.add_argument("--bounds-max", type=float, nargs=2, default=list(DEFAULT_BOUNDS_MAX),
                    metavar=("XMAX", "YMAX"), help="栅格至少覆盖到的右上角世界坐标 [m]")
    ap.add_argument("--dilate", type=int, default=0,
                    help="把每类覆盖按方形半径 N 格膨胀（默认 0，保持保守）")
    ap.add_argument("--collada-matrix-order", choices=("row", "col"), default=MATRIX_ORDER_DEFAULT,
                    help="COLLADA <matrix> 的主序（默认 row：实测与 world 位姿自洽；"
                         "col 为规范读法，会把冰箱/衣柜等埋进地板）")
    ap.add_argument("--quiet", action="store_true", help="只打印一行 JSON 统计")
    args = ap.parse_args(argv)
    MATRIX_ORDER_DEFAULT = args.collada_matrix_order

    if args.res <= 0:
        raise SystemExit("错误：--res 必须为正数")

    try:
        instances, grid, stats_out, failures, warnings = build(
            world_path=args.world, model_root=args.model_root, res=args.res,
            origin=tuple(args.origin), bounds_max=tuple(args.bounds_max), dilate=int(args.dilate),
        )
    except SystemExit:
        raise
    except Exception as exc:
        sys.stderr.write(f"错误：构建过程异常 {type(exc).__name__}: {exc}\n")
        return 1

    paths = write_outputs(args.out, instances, grid, stats_out, failures, warnings,
                          args.world, args.model_root)
    if args.quiet:
        keys = ("include_total", "include_ok", "include_skipped", "collision_total",
                "collision_ok", "collision_mesh", "bit0_cells", "bit1_low_band_cells",
                "bit2_high_band_cells", "bit3_solid_cells", "failures", "elapsed_s")
        print(json.dumps({k: stats_out[k] for k in keys}, ensure_ascii=False))
    else:
        print_summary(stats_out, failures, warnings, paths)
    return 0


if __name__ == "__main__":
    sys.exit(main())
