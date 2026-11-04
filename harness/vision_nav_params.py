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
# ★ 2026-10-28 方案A：局部视觉层追加一条"只清不标"的自由射线源（默认关，--free-rays 打开）
FREE_RAYS = False
FREE_BLOCK = '''
          vision_free:
            topic: /vision/free_rays
            data_type: "PointCloud2"
            marking: False
            clearing: True
            min_obstacle_height: -0.50
            max_obstacle_height: 2.00
            obstacle_max_range: 0.0
            raytrace_max_range: 5.00
            raytrace_min_range: 0.00'''


def local_block():
    """局部视觉层块；FREE_RAYS 时在 observation_sources 里追加 vision_free 并补其配置。"""
    if not FREE_RAYS:
        return BLOCK_L
    # nav2 的 observation_sources 用**空格**分隔；写逗号会让它把 "vision_cloud," 当源名
    # （实测报 Invalid topic name 并让 local_costmap configure 失败）
    b = BLOCK_L.replace('observation_sources: vision_cloud',
                        'observation_sources: vision_cloud vision_free')
    end = SENT % 'end(local)'
    return b.replace(end, FREE_BLOCK.strip('\n') + '\n' + end)
SENT = '# ---- vision_fusion_patch:%s ----'

BLOCK_L = f'''{SENT % "begin(local)"}
      {TAG_L}:
        plugin: "nav2_costmap_2d::VoxelLayer"
        enabled: True
        publish_voxel_map: False
        origin_z: 0.0
        z_resolution: 0.05
        z_voxels: 16
        unknown_threshold: 16
        mark_threshold: 0
        combination_method: 1
        observation_sources: vision_cloud
        vision_cloud:
          topic: /vision/live_cloud
          data_type: "PointCloud2"
          marking: True
          clearing: True
          min_obstacle_height: 0.05
          # 0.80：cafe_table 桌面在 0.755(+0.02) m，取 0.60 会滤掉桌面点、
          # 只剩 4.2 cm 细立柱 —— 那样这套视觉层治不了"假口子"
          max_obstacle_height: 0.80
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
        # 2026-10-26 P9-④：原来这里订阅的是 /vision/obstacle_cloud（累积云，只增不减）
        # 且 clearing: False —— 家具被车挪开、或本来就是深度噪点/掠射面时，
        # 这些格子会**永久**留在全局代价地图里，把走廊堵死到本轮结束。
        # 实测 baser1-3 的旋转期障碍格持续增长（探针 rot_growth 事件 3 次/轮）即出自这一层。
        # 改成 live 云 + clearing，并让 raytrace 只到 5 m（深度图有效距离）——
        # 绝不能给 30 m：深度云只有近处点，30 m 射线会把远处真实墙体的记忆擦掉。
        observation_sources: vision_live
        vision_live:
          topic: /vision/live_cloud
          data_type: "PointCloud2"
          marking: True
          clearing: True
          min_obstacle_height: 0.05
          max_obstacle_height: 0.85
          obstacle_max_range: 5.0
          obstacle_min_range: 0.0
          raytrace_max_range: 5.0
          raytrace_min_range: 0.0
{SENT % "end(global)"}
'''

# P9-⑧（2026-10-27 用户批准"先修1"）：全局代价地图的视觉层改用**累积云**且不清除。
# 事故：s0ar1 车卡在健身器材上轮子空转，而 /map 与雷达都认为那里空（物体在扫描平面下）；
# 视觉其实看到了它，但全局层用的是 live 云 + clearing + 5 m raytrace —— 车一离开，
# 视觉障碍就被射线清掉，全局规划器随后直接横穿那里（用户看到的"红障碍被路径穿过去"）。
# 累积云已由 vision_obstacle_grid 做过"2 帧确认 + 膨胀 + 独立衰减(accum_decay_sec)"，
# 幽灵障碍问题由衰减窗口负责，不再靠 raytrace 清除。
BLOCK_G_ACCUM = f'''{SENT % "begin(global)"}
      {TAG_G}:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: True
        combination_method: 1
        # P9-⑧：累积云 + 不清除；不做 raytrace（否则会把累积记忆扫掉）
        observation_sources: vision_accum
        vision_accum:
          topic: /vision/obstacle_cloud
          data_type: "PointCloud2"
          marking: True
          clearing: False
          min_obstacle_height: 0.05
          max_obstacle_height: 0.85
          obstacle_max_range: 30.0
          obstacle_min_range: 0.0
          raytrace_max_range: 0.0
          raytrace_min_range: 0.0
{SENT % "end(global)"}
'''


BLOCK_G_LOWACCUM = f'''{SENT % "begin(global)"}
      {TAG_G}:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: True
        combination_method: 1
        # P9-⑧b：只把**低矮带**累积云（z 0.05~0.35 m，雷达扫描平面以下）写进全局代价地图，
        # 不清除、不做 raytrace —— 这是雷达唯一"看不见"的部分；高障碍交回雷达自己记。
        # 相比全带累积（§20，覆盖 -17.8%），代价最小、最对准"哑铃/器材底座/床头柜"这类事故。
        observation_sources: vision_low_accum
        vision_low_accum:
          topic: /vision/low_accum_cloud
          data_type: "PointCloud2"
          marking: True
          clearing: False
          min_obstacle_height: 0.02
          max_obstacle_height: 0.45
          obstacle_max_range: 30.0
          obstacle_min_range: 0.0
          raytrace_max_range: 0.0
          raytrace_min_range: 0.0
{SENT % "end(global)"}
'''


# P9-⑨（2026-10-27）：隔离变量 —— **live 云 + 不清除**。
# 为什么还要这一档：实测（nav_only + corridor_fixed 图、车停在健身区）同一时刻
#   /vision/live_cloud 的点 86% 在全局代价图上已是致命格 ⇒ 视觉层确实在喂数据；
#   但视觉累积记忆的 1441 格里，任一时刻只有 22.5% 致命、15.3% 膨胀、61.7% 又回到空闲，
#   最近 6 次航点导航的"最窄点"距视觉格只有 0.02~0.13 m、且此刻代价仅 58~94（可通行）
#   ⇒ Smac 只把致命格当碰撞，膨胀格可走 ⇒ 路径从视觉障碍的格子缝里穿过去。
# 怀疑主因是 live 源的 raytrace 把物体格子"同帧标上、又被更远点的射线擦掉"。
# livemark 只关掉 clearing（云的来源不变），是判定该主因的单一变量档。
BLOCK_G_LIVEMARK = f'''{SENT % "begin(global)"}
      {TAG_G}:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: True
        combination_method: 1
        # P9-⑨：live 云、只标记不清除、无 raytrace —— 隔离"射线擦除"这一项
        observation_sources: vision_live_mark
        vision_live_mark:
          topic: /vision/live_cloud
          data_type: "PointCloud2"
          marking: True
          clearing: False
          min_obstacle_height: 0.05
          max_obstacle_height: 0.85
          obstacle_max_range: 5.0
          obstacle_min_range: 0.0
          raytrace_max_range: 0.0
          raytrace_min_range: 0.0
{SENT % "end(global)"}
'''


MODE = 'live'


def sections():
    g = {'live': BLOCK_G, 'accum': BLOCK_G_ACCUM,
         'lowaccum': BLOCK_G_LOWACCUM,
         'livemark': BLOCK_G_LIVEMARK}.get(MODE, BLOCK_G)
    return [('local_costmap', TAG_L, local_block()), ('global_costmap', TAG_G, g)]


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
    for name, tag, block in sections():
        lo, hi = section_range(lines, name)
        i = find_plugins_line(lines, lo, hi)
        if tag not in plugin_list_of(lines[i]):
            lines[i] = inject_tag(lines[i], tag)
            changed = True
        # 块：不存在才追加（追加到该段的 always_send_full_costmap 之前）
        key = 'begin(local)' if name == 'local_costmap' else 'begin(global)'
        if SENT % key in '\n'.join(lines):
            # 已注入：若内容与目标模式不一致（例如 live -> accum），就地替换整块
            b, e = SENT % key, SENT % (key.replace('begin', 'end'))
            try:
                i = lines.index(b); j = lines.index(e, i)
            except ValueError:
                i = j = None
            want = block.rstrip('\n').split('\n')
            if i is not None and [x.rstrip() for x in lines[i:j + 1]] != [x.rstrip() for x in want]:
                lines[i:j + 1] = want
                changed = True
        else:
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
    for name, tag, block in sections():
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
    for name, tag, _ in sections():
        sec = d.get(name) or {}
        inner = sec.get(name) or {}
        names = (inner.get('ros__parameters') or {}).get('plugins') or []
        if tag not in names:
            return False, f'{name}.plugins 未包含 {tag}'
    return True, 'ok'


JOURNAL = {}


CACHE = os.path.join('/home/kbf/stemm_exploration_sim_ws', 'patches')


def journal_path(path):
    """备份/journal 都落在仿真仓库里 —— 车端仓库只留"被改的那一个文件"，
    不额外生产垃圾文件（与既有 .sim_backup 约定区分开）。"""
    os.makedirs(CACHE, exist_ok=True)
    return os.path.join(CACHE, 'stemm_safe_nav2_params.vision_journal.json')


def backup_path(path):
    os.makedirs(CACHE, exist_ok=True)
    return os.path.join(CACHE, 'stemm_safe_nav2_params.vision_backup.yaml')


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
    ap.add_argument('--global-mode',
                    choices=('live', 'accum', 'lowaccum', 'livemark'), default='live',
                    help='global_costmap 的视觉层源：live=实时云+清除（旧行为，实测路径会穿过视觉障碍）；'
                         'accum=全带累积云+不清除（P9-⑧，覆盖退化）；'
                         'lowaccum=仅低矮带累积（P9-⑧b）；'
                         'livemark=实时云+不清除（P9-⑨，隔离 raytrace 擦除）')
    ap.add_argument('--free-rays', action='store_true',
                    help='方案A：局部视觉层追加一条 marking=False/clearing=True 的自由射线源'
                         '（订阅 /vision/free_rays，raytrace 5 m）')
    a = ap.parse_args()
    global MODE, FREE_RAYS
    MODE = a.global_mode
    FREE_RAYS = a.free_rays
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
        for name, tag, _ in sections():
            names = d[name][name]['ros__parameters'].get('plugins')
            print(f'  {name}.plugins = {names}')
        return 0

    backup = backup_path(path)
    if a.apply and not os.path.exists(backup):
        shutil.copy2(path, backup)
        print(f'  已备份 → {backup}')

    load_journal(path)
    if a.apply:
        # 记录注入前的原始 plugins 行（restore 时逐字节还原）
        for name, tag, _ in sections():
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
    for name, tag, _ in sections():
        lo, hi = section_range(lines, name)
        i = find_plugins_line(lines, lo, hi)
        print(f'    {name}: {plugin_list_of(lines[i])}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
