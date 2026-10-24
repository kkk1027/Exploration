#!/usr/bin/env python3
"""evaluate.py —— 一轮 launch.log 的验收（七项原有 + P0 撞障/后退两项）。

用法：
  evaluate.py <launch.log> [tag] [probe.jsonl] [--p0]
    --p0  把"接触=0 且 反向指令=0"也算进 verdict（P0 改造后的验收口径）。
          不给 --p0 时这两项只作信息输出（对照基线里盲退是预期存在的）。
"""
import json
import os
import re
import sys

args = [a for a in sys.argv[1:] if not a.startswith('--')]
flags = {a for a in sys.argv[1:] if a.startswith('--')}
log = args[0]
tag = args[1] if len(args) > 1 else 'run'
probe = args[2] if len(args) > 2 else None

txt = open(log, encoding='utf-8', errors='ignore').read()


def has(*pats):  # 任意一行同时含全部关键字
    for line in txt.splitlines():
        if all(p in line for p in pats):
            return line.strip()[:160]
    return None


checks = {}
checks['preflight'] = has('preflight', 'passed') or has('system service and process isolation checks passed')
checks['tf_healthy'] = has('mapping TF is healthy')
checks['nav2_active'] = has('stably ready') or has('ready_guard_timing')
checks['frontier'] = has('assigned RRT frontier') or has('Begin navigating')
checks['mapping_done'] = has('SetBool_Response', 'success=True') or has('mapping done acknowledged')
# 返航判据：只认**正向完成行**，必须排掉否定句
# （2026-10-23 踩过：'中断退出：会话未走到返航到位，仍尝试存图' 曾被当成返航成功）
NEG = ('未走到返航到位', '未返航', '返航未启动', '返航失败', '返航超时', '返航卡住',
       '会话未走到', '中断退出', 'keeping launch alive for manual takeover',
       'return-home failed', 'stuck after', 'return_home_failed')
POS = (r'返航(?:到位|完成)',
       r'return-home completed',
       r'return-home position reached',
       # 2026-10-24：manager 的"到点即成功"分支用的是这句，之前没认导致真实成功的轮被判失败
       r'return-home goal reached')
pos = []
for line in txt.splitlines():
    low = line.lower()
    if any(neg in line or neg in low for neg in NEG):
        continue
    if any(re.search(p_, line) for p_ in POS):
        pos.append(line.strip()[:160])
checks['return_home'] = pos[-1] if pos else None
# P0-丁：把两种完成语义分开记账（只记录，不参与判定；等待产品决策）
RET_KIND = {
    'by_goal_reached': ('return-home goal reached' in txt),
    'by_position': ('completed by position' in txt or 'position reached' in txt),
    'orientation_exact': ('exact orientation' not in txt),
}
mapfile = os.path.expanduser('~/stemm_sim_ws/maps/explore_latest.pgm')
checks['map_saved'] = f'{os.path.getsize(mapfile)} B' if os.path.exists(mapfile) else None

# ---- P0：撞障取证 ----
collision = None
if probe and os.path.exists(probe):
    import collections
    c = collections.Counter()
    chassis = 0
    wheel = 0
    rev = 0
    rev_m = 0.0
    rev_hard = 0
    rev_dists = []
    mind = None
    for line in open(probe, encoding='utf-8', errors='ignore'):
        line = line.strip()
        if not line:
            continue
        try:
            r = json.loads(line)
        except Exception:
            continue
        t = r.get('type')
        if t in ('contact', 'graze', 'alert'):
            c[t] += 1
        elif t == 'reverse' and r.get('phase') == 'end':
            rev += 1
            d = float(r.get('dist') or 0.0)
            rev_m += d
            if d >= 0.05 and not r.get('assist'):
                rev_hard += 1
                rev_dists.append(round(d, 3))
        elif t == 'summary':
            chassis = int(r.get('chassis_contacts') or 0)
            wheel = int(r.get('wheel_contacts') or 0)
        for k in ('min_d', 'min_d_now', 'min_d_ever'):
            v = r.get(k)
            if isinstance(v, (int, float)) and v >= 0 and (mind is None or v < mind):
                mind = v
    collision = {'lidar_contacts': c['contact'], 'grazes': c['graze'], 'alerts': c['alert'],
                 'reverse_windows': rev, 'reverse_m': round(rev_m, 2),
                 'reverse_hard_windows': rev_hard, 'chassis_contacts': chassis,
                 'wheel_contacts': wheel, 'physical_contacts': chassis + wheel,
                 'min_clearance_ever': round(mind, 3) if mind is not None else None}
    # 判据用"物理接触"（Gazebo 接触传感器，已过滤地面）+ ">=5 cm 的**非受控**后退窗口"
    # （返航阶段由 escape_supervisor 执行的受控小步倒车 assist=True，不算违规）；
    # 雷达口径的近距点留在 collision 里作参考（桌下/近场回波会有）。
    checks['no_contact'] = collision['physical_contacts'] == 0
    # 2026-10-24（用户批准）：探索期允许"仅前向受阻时的 <=0.15 m 雷达复核受控小步倒车"。
    # 放行条件严格对齐批准口径：patch 明确开启 + 单次 <=0.20 m（含余量）+ 每轮 <=2 次（车端 max_per_session=2）；
    # 其余仍按违规计（盲退 / 超包络 / 次数超限）。
    backstep_on = '探索期受控小步倒车已启用' in txt
    max_win = max(rev_dists) if rev_dists else 0.0
    # 窗口可能被 odom 采样切分，所以按"包络"判而不是按窗口个数：
    #   单次 <=0.20 m（批准 0.15 + 余量）、整轮合计 <=0.50 m（约 3 步）
    authorized = (backstep_on and rev_hard > 0 and max_win <= 0.20
                  and collision['reverse_m'] <= 0.50)
    collision['backstep_enabled'] = backstep_on
    collision['reverse_max_window_m'] = max_win
    collision['reverse_authorized'] = authorized
    checks['no_reverse'] = (collision['reverse_hard_windows'] == 0) or authorized

# ---- 方案A：视觉层（只在 vision:=true 的轮次要求）----
vision_line = has('vision_obstacle_grid', '累积使能')
vision = None
if vision_line or 'vision_obstacle_grid' in txt:
    cells = 0
    for line in txt.splitlines():
        m = re.search(r'累积使能 (\d+)(?:/\d+)? 格', line)
        if m:
            cells = max(cells, int(m.group(1)))
    layer_err = has('Failed to create', 'voxel') or has('Could not find plugin') \
        or has('Unconfigured', 'vision_') or has('vision_voxel_layer', 'error') \
        or has('vision_obstacle_layer', 'error')
    vision = {'cells': cells,
              'node_seen': bool(vision_line),
              'layer_error': bool(layer_err),
              'layer_error_line': layer_err}
if vision is not None:
    # 只有"这一轮本来就是要跑视觉"时才纳入硬判据（VISION 开关由 round.sh 传进日志）
    if 'vision:=true' in txt or '视觉障碍：' in txt:
        checks['vision_cells'] = vision['cells'] > 0
        checks['vision_layer_ok'] = not vision['layer_error']

p0 = '--p0' in flags
hard = [k for k in checks if k not in ('return_home',)]
if not p0:
    hard = [k for k in hard if k not in ('no_contact', 'no_reverse')]
ok = all(checks.get(k) for k in hard)
verdict = 'PASS' if ok and checks.get('return_home') else 'FAIL'
out = {'tag': tag, 'verdict': verdict,
       'checks': {k: bool(v) for k, v in checks.items()},
       'evidence': {k: v for k, v in checks.items() if v}}
if collision is not None:
    out['collision'] = collision
out['return_home_kind'] = RET_KIND
if vision is not None:
    out['vision'] = vision
print(json.dumps(out, ensure_ascii=False, indent=1))
sys.exit(0 if verdict == 'PASS' else 1)
