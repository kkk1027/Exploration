#!/usr/bin/env python3
"""vision_nav_params.py —— 方案A：给 nav2 代价地图注入"视觉层"的**可回退**参数补丁。

为什么要改文件
  nav2 的 costmap `plugins` 列表只在节点 configure 时构建，运行期 `ros2 param set` 无效，
  所以只能在启动前改参数文件。生产参数属于车端仓库，本机此前已用同类方式把 BT XML 路径
  改到本机 overlay（未提交，patch 记录在 patches/），这里沿用同一约定。

做法（纯文本、按段定位、可逆、注释与排版零破坏）
  local_costmap  : plugins 列表里插入 vision_voxel_layer    + 追加一个 VoxelLayer 块
  global_costmap : plugins 列表里插入 vision_obstacle_layer + 追加一个只增不减的 ObstacleLayer 块
  两个块用哨兵注释包起来，--restore 精确删除；每次写入前都做 YAML 解析校验，坏了不落盘。

用法
  python3 harness/vision_nav_params.py --check     # 打印目标文件与当前状态
  python3 harness/vision_nav_params.py --apply     # 注入（幂等）
  python3 harness/vision_nav_params.py --restore   # 撤销（幂等）
"""
import argparse
import json
import os
import re
import shutil
import subprocess
import sys

import yaml

TAG_L, TAG_G = 'vision_voxel_layer', 'vision_obstacle_layer'
SENT = '# ---- vision_fusion_patch:%s ----'

BLOCK_L = f'''{SENT % "begin(local)"}
      {TAG_L}:
        plugin: "nav2_costmap_2d::VoxelLayer"
        enabled: True
        publish_voxel_map: False
        origin_z: 0.0
        z_resolution: 0.05
        z_voxels: 12
        unknown_threshold: 12
        mark_threshold: 0
        combination_method: 1
        observation_sources: vision_cloud
        vision_cloud:
          topic: /vision/live_cloud
          data_type: "PointCloud2"
          marking: True
          clearing: True
          min_obstacle_height: 0.05
          max_obstacle_height: 0.60
          obstacle_max_range: 3.0
          obstacle_min_range: 0.0
          raytrace_max_range: 3.5
          raytrace_min_range: 0.0
{SENT % "end(local)"}
'''

BLOCK_G = f'''{SENT % "begin(global)"}
      {TAG_G}:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: True
        combination_method: 1
        observation_sources: vision_accum
        vision_accum:
          topic: /vision/obstacle_cloud
          data_type: "PointCloud2"
          marking: True
          clearing: False
          min_obstacle_height: 0.05
          max_obstacle_height: 0.60
          obstacle_max_range: 30.0
          obstacle_min_range: 0.0
          raytrace_max_range: 30.0
          raytrace_min_range: 0.0
{SENT % "end(global)"}
'''

SECTIONS = [('local_costmap', TAG_L, BLOCK_L), ('global_costmap', TAG_G, BLOCK_G)]


def target_path():
    """解析运行期真正被加载的那份参数文件。"""
    try:
        r = subprocess.run(['ros2', 'pkg', 'prefix', 'stemm_cartographer_exploration'],
                           capture_output=True, text=True, timeout=20)
        if r.returncode == 0:
            p = os.path.join(r.stdout.strip(), 'share', 'stemm_cartographer_exploration',
                             'config', 'stemm_safe_nav2_params.yaml')
            if os.path.exists(p):
                return os.path.realpath(p)
    except Exception:
        pass
    return os.path.realpath('/home/kbf/wheeltec_ros2/src/stemm_cartographer_exploration/'
                            'config/stemm_safe_nav2_params.yaml')


def section_range(lines, name):
    """返回顶层段 name: 的行区间 [start, end)。"""
    start = None
    for i, ln in enumerate(lines):
        if ln.rstrip() == f'{name}:':
            start = i
            continue
        if start is not None and ln[:1] not in (' ', '\t', '#', '\n', '') and ln.rstrip().endswith(':'):
            return start, i
    if start is None:
        raise SystemExit(f'✗ 找不到段 {name}:')
    return start, len(lines)


def find_plugins_line(lines, lo, hi):
    for i in range(lo, hi):
        if lines[i].lstrip().startswith('plugins: ['):
            return i
    raise SystemExit(f'✗ 段内找不到 plugins 列表行（行 {lo}-{hi}）')


def plugin_list_of(line):
    m = re.search(r'plugins:\s*\[(.*?)\]', line)
    if not m:
        raise SystemExit(f'✗ 解析不了 plugins 行: {line!r}')
    return [x.strip().strip('"\'') for x in m.group(1).split(',') if x.strip()]


def inject_tag(line, tag):
    """把 tag 插到 inflation_layer 之前（障碍层必须先于膨胀层），其余文本不动。"""
    if f'"{tag}"' in line:
        return line
    if '"inflation_layer"' in line:
        return line.replace('"inflation_layer"', f'"{tag}", "inflation_layer"', 1)
    return line.replace('plugins: [', f'plugins: ["{tag}", ', 1)


def remove_tag(line, tag):
    out = re.sub(r',\s*"%s"' % tag, '', line)
    out = re.sub(r'"%s"\s*,\s*' % tag, '', out)
    return out


def do_apply(lines):
    changed = False
    for name, tag, block in SECTIONS:
        lo, hi = section_range(lines, name)
        i = find_plugins_line(lines, lo, hi)
        if tag not in plugin_list_of(lines[i]):
            lines[i] = inject_tag(lines[i], tag)
            changed = True
        # 块：不存在才追加（追加到该段的 always_send_full_costmap 之前）
        key = 'begin(local)' if name == 'local_costmap' else 'begin(global)'
        if SENT % key not in '\n'.join(lines):
            anchor = None
            for j in range(lo, hi):
                if lines[j].lstrip().startswith('always_send_full_costmap'):
                    anchor = j
                    break
            if anchor is None:
                anchor = hi
            lines[anchor:anchor] = block.rstrip('\n').split('\n') + []
            changed = True
    return changed


def do_restore(lines):
    changed = False
    for name, tag, block in SECTIONS:
        key = 'begin(local)' if name == 'local_costmap' else 'begin(global)'
        b, e = SENT % key, SENT % key.replace('begin', 'end')
        while b in lines:
            i = lines.index(b)
            j = lines.index(e, i)
            del lines[i:j + 1]
            changed = True
        lo, hi = section_range(lines, name)
        i = find_plugins_line(lines, lo, hi)
        if tag in plugin_list_of(lines[i]):
            orig = JOURNAL.get(name)
            lines[i] = orig if orig is not None else remove_tag(lines[i], tag)
            changed = True
    return changed


def validate(text, expect_tags=True):
    try:
        d = yaml.safe_load(text)
    except Exception as e:
        return False, f'YAML 解析失败: {e}'
    if not expect_tags:
        return True, 'ok'
    for name, tag, _ in SECTIONS:
        sec = d.get(name) or {}
        inner = sec.get(name) or {}
        names = (inner.get('ros__parameters') or {}).get('plugins') or []
        if tag not in names:
            return False, f'{name}.plugins 未包含 {tag}'
    return True, 'ok'


JOURNAL = {}


def journal_path(path):
    return path + '.vision_journal.json'


def load_journal(path):
    global JOURNAL
    try:
        JOURNAL = json.load(open(journal_path(path), encoding='utf-8'))
    except Exception:
        JOURNAL = {}


def save_journal(path):
    if JOURNAL:
        json.dump(JOURNAL, open(journal_path(path), 'w', encoding='utf-8'), ensure_ascii=False, indent=1)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--apply', action='store_true')
    ap.add_argument('--restore', action='store_true')
    ap.add_argument('--check', action='store_true')
    a = ap.parse_args()
    path = target_path()
    text = open(path, encoding='utf-8').read()
    lines = text.split('\n')
    st = {tag: (tag in text) for _, tag, _ in SECTIONS}
    print(f'[vision_nav_params] 目标: {path}')
    print(f'  当前: {st}  哨兵块={text.count("vision_fusion_patch:begin")}')
    if a.check or not (a.apply or a.restore):
        try:
            d = yaml.safe_load(text)
            print('  YAML: ok')
        except Exception as e:
            print(f'  YAML: FAIL {e}')
            return 1
        for name, tag, _ in SECTIONS:
            names = d[name][name]['ros__parameters'].get('plugins')
            print(f'  {name}.plugins = {names}')
        return 0

    backup = path + '.vision_backup'
    if a.apply and not os.path.exists(backup):
        shutil.copy2(path, backup)
        print(f'  已备份 → {backup}')

    load_journal(path)
    if a.apply:
        # 记录注入前的原始 plugins 行（restore 时逐字节还原）
        for name, tag, _ in SECTIONS:
            lo, hi = section_range(lines, name)
            i = find_plugins_line(lines, lo, hi)
            if tag not in plugin_list_of(lines[i]) and name not in JOURNAL:
                JOURNAL[name] = lines[i]
    changed = do_apply(lines) if a.apply else do_restore(lines)
    if not changed:
        print('  无变化（已处于目标状态）')
        return 0
    new = '\n'.join(lines)
    ok, why = validate(new, expect_tags=a.apply)
    if not ok:
        print(f'  ✗ 校验失败，放弃写入（原文件未动）：{why}')
        return 3
    open(path, 'w', encoding='utf-8').write(new)
    if a.apply:
        save_journal(path)
    else:
        JOURNAL.clear()
        try:
            os.remove(journal_path(path))
        except OSError:
            pass
    print(f'  ✓ {"注入" if a.apply else "撤销"}完成')
    # 打印结果确认
    for name, tag, _ in SECTIONS:
        lo, hi = section_range(lines, name)
        i = find_plugins_line(lines, lo, hi)
        print(f'    {name}: {plugin_list_of(lines[i])}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
