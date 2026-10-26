#!/usr/bin/env python3
"""bench_ab.py —— 在**无 Gazebo 的 nav2 测试台**上做参数 A/B（同目标集合、同判据）。

用法：
  python3 harness/bench_ab.py                        # 默认：原厂 vs 调优，内置目标集
  python3 harness/bench_ab.py --goals 1.2,0.9 -1.5,-3.0
  python3 harness/bench_ab.py --configs tuned

每个配置：起 nav_sim.sh（约 10 s），逐个目标发 NavigateToPose，记录
  状态（SUCCEEDED/ABORTED/CANCELED/超时）、耗时、终位误差、对应 /plan 的最小净宽/窄段。
结果写成 markdown 表 + JSONL，便于复核。
"""
import argparse
import json
import math
import os
import signal
import subprocess
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node

WS = '/home/kbf/stemm_exploration_sim_ws'
CONFIGS = {
    # 名字: nav_sim.sh 的环境变量
    'orig': {'NAV_ARGS_OFF': '1'},
    'tuned': {'COST_INFL': '1', 'PLANNER_PATCH': '1', 'ALLOW_UNKNOWN': 'false',
              'MOTION_MODEL': 'REEDS_SHEPP', 'VX_MIN': '-0.08', 'VETO': 'false'},
    # 只提"窄缝代价"（规划器宁可绕远），不加 pad —— 实测 pad 0.29 会把整张图判死
    'cost8': {'COST_INFL': '1', 'COST_PEN': '8.0', 'PLANNER_PATCH': '1', 'ALLOW_UNKNOWN': 'false',
              'MOTION_MODEL': 'REEDS_SHEPP', 'VX_MIN': '-0.08', 'VETO': 'false'},
    # 结构性禁止：内切 0.235+0.29=0.525 ⇒ 自由宽度 <1.05 m 的缝直接致命（不是"贵"）
    'pad29': {'COST_INFL': '1', 'PAD': '0.29', 'PLANNER_PATCH': '1', 'ALLOW_UNKNOWN': 'false',
              'MOTION_MODEL': 'REEDS_SHEPP', 'VX_MIN': '-0.08', 'VETO': 'false'},
    'pad29_veto': {'COST_INFL': '1', 'PAD': '0.29', 'PLANNER_PATCH': '1', 'ALLOW_UNKNOWN': 'false',
                   'MOTION_MODEL': 'REEDS_SHEPP', 'VX_MIN': '-0.08', 'VETO': 'true'},
    'tuned_veto': {'COST_INFL': '1', 'PLANNER_PATCH': '1', 'ALLOW_UNKNOWN': 'false',
                   'MOTION_MODEL': 'REEDS_SHEPP', 'VX_MIN': '-0.08', 'VETO': 'true'},
}
# 内置目标集：既有开阔点，也有历史上卡过车的窄角（地图 explore_latest.yaml 坐标系）
DEFAULT_GOALS = [(1.2, 0.9), (2.6, 1.6), (-1.5, -3.0), (-2.1, 1.4),
                 (3.9, -2.4), (-4.2, -4.6), (0.5, 3.6), (-6.3, -4.7)]


class Runner(Node):
    def __init__(self):
        super().__init__('bench_ab_runner')
        self.cli = ActionClient(self, NavigateToPose, '/navigate_to_pose')

    def wait_ready(self, timeout=120.0):
        if not self.cli.wait_for_server(timeout_sec=timeout):
            return False
        # 关键：action server 先出现、代价地图还没 active 时，BT 会**立刻拒绝**目标
        # （实测首个目标 0.0 s 就被拒，计划 0 条）。等两张代价地图真有发布者再给目标。
        need = ['/global_costmap/costmap', '/local_costmap/costmap', '/scan', '/map']
        t0 = time.time()
        while time.time() - t0 < 60.0:
            topics = dict(self.get_topic_names_and_types())
            pubs = self.count_publishers
            if all(t in topics and pubs(t) > 0 for t in need):
                time.sleep(6.0)          # 再给 lifecycle 收尾一点时间
                return True
            time.sleep(1.0)
        return False

    def go(self, gx, gy, timeout):
        goal = NavigateToPose.Goal()
        ps = PoseStamped()
        ps.header.frame_id = 'map'
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = float(gx)
        ps.pose.position.y = float(gy)
        ps.pose.orientation.w = 1.0
        goal.pose = ps
        t0 = time.time()
        fut = self.cli.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=25.0)
        gh = fut.result()
        if gh is None or not gh.accepted:
            time.sleep(4.0)                    # 链路刚 active 时偶发拒绝：等一拍再试一次
            fut = self.cli.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=25.0)
            gh = fut.result()
            if gh is None or not gh.accepted:
                return {'status': 'REJECTED', 'sec': round(time.time() - t0, 1)}
        res_fut = gh.get_result_async()
        while time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.2)
            if res_fut.done():
                break
        sec = round(time.time() - t0, 1)
        if not res_fut.done():
            gh.cancel_goal_async()
            return {'status': 'TIMEOUT', 'sec': sec}
        st = res_fut.result().status
        name = {GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
                GoalStatus.STATUS_ABORTED: 'ABORTED',
                GoalStatus.STATUS_CANCELED: 'CANCELED'}.get(st, str(st))
        rec = {'status': name, 'sec': sec}
        # 计时/成功的同时把"计划体检"读回来（plan_audit.jsonl 里按时间窗过滤）
        return rec


def start_stack(cfg_env, logdir):
    env = dict(os.environ)
    env.update(cfg_env)
    env['LOG'] = logdir
    os.makedirs(logdir, exist_ok=True)
    logf = open(os.path.join(logdir, 'launch.log'), 'w')
    p = subprocess.Popen(['bash', 'harness/nav_sim.sh'], cwd=WS, env=env,
                         stdout=logf, stderr=subprocess.STDOUT, start_new_session=True)
    return p, logf


def stop_stack(p, logf):
    try:
        os.killpg(os.getpgid(p.pid), signal.SIGTERM)
    except Exception:                           # noqa: BLE001
        pass
    time.sleep(3)
    try:
        os.killpg(os.getpgid(p.pid), signal.SIGKILL)
    except Exception:                           # noqa: BLE001
        pass
    try:
        logf.close()
    except Exception:                           # noqa: BLE001
        pass


def audit_rows(logdir):
    p = os.path.join(logdir, 'plan_audit.jsonl')
    out = []
    if os.path.exists(p):
        for line in open(p, encoding='utf-8', errors='ignore'):
            line = line.strip()
            if line:
                try:
                    out.append(json.loads(line))
                except Exception:               # noqa: BLE001
                    pass
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--configs', default='orig,tuned')
    ap.add_argument('--goals', default='')
    ap.add_argument('--timeout', type=float, default=40.0)
    ap.add_argument('--out', default='')
    a = ap.parse_args()
    goals = DEFAULT_GOALS
    if a.goals:
        goals = [tuple(float(v) for v in pair.split(',')) for pair in a.goals.split()]
    names = [c.strip() for c in a.configs.split(',') if c.strip()]
    rclpy.init()
    node = Runner()
    results = {}
    stamp = time.strftime('%H%M%S')
    for name in names:
        env = CONFIGS.get(name)
        if env is None:
            print(f'✗ 未知配置 {name}（可选：{", ".join(CONFIGS)}）')
            continue
        logdir = os.path.join(WS, 'logs', f'bench_{stamp}_{name}')
        print(f'=== {name}: 起栈 → {logdir}')
        proc, logf = start_stack(env, logdir)
        if not node.wait_ready(timeout=150.0):
            print(f'✗ {name}: NavigateToPose 未就绪，跳过')
            stop_stack(proc, logf)
            continue
        rows = []
        for gx, gy in goals:
            n0 = len(audit_rows(logdir))
            rec = node.go(gx, gy, a.timeout)
            time.sleep(1.0)
            aud = audit_rows(logdir)[n0:]
            mins = [r['min_w'] for r in aud if isinstance(r.get('min_w'), (int, float))]
            hard = [r.get('hard_pts') or 0 for r in aud]
            rec.update({'goal': [gx, gy], 'plans': len(aud),
                        'plan_min_w': min(mins) if mins else None,
                        'plan_hard_pts': max(hard) if hard else None,
                        'veto': sum(1 for r in aud if r.get('bad'))})
            rows.append(rec)
            print('  goal %6.2f,%6.2f → %-9s %5.1fs  计划 %d 条 最窄 %s m  比车窄点 %s  窄处计划 %s'
                  % (gx, gy, rec['status'], rec['sec'], rec['plans'],
                     rec['plan_min_w'], rec['plan_hard_pts'], rec['veto']))
        results[name] = rows
        stop_stack(proc, logf)
        time.sleep(2)
    node.destroy_node()
    rclpy.shutdown()

    # ---- 汇总 ----
    L = ['# nav2 参数 A/B（无 Gazebo 台架）', '']
    hdr = ['配置', '成功', '超时/中止/被撤销', '平均耗时 s', '计划最窄中位数 m', '有"比车窄"目标的次数']
    L.append('| ' + ' | '.join(hdr) + ' |')
    L.append('| ' + ' | '.join(['---'] * len(hdr)) + ' |')
    for name, rows in results.items():
        ok = sum(1 for r in rows if r['status'] == 'SUCCEEDED')
        bad = sum(1 for r in rows if r['status'] != 'SUCCEEDED')
        secs = [r['sec'] for r in rows]
        mins = sorted([r['plan_min_w'] for r in rows if isinstance(r['plan_min_w'], (int, float))])
        med = mins[len(mins) // 2] if mins else None
        hard = sum(1 for r in rows if (r['plan_hard_pts'] or 0) > 0)
        L.append('| %s | %d/%d | %d | %.1f | %s | %d |'
                 % (name, ok, len(rows), bad, sum(secs) / max(1, len(secs)),
                    ('%.2f' % med) if med is not None else '-', hard))
    L.append('')
    L.append('逐目标明细见 JSON。判据：成功=SUCCEEDED；"比车窄"=计划的采样点里存在净宽 < 0.47 m。')
    txt = '\n'.join(L) + '\n'
    out = a.out or os.path.join(WS, 'logs', f'bench_{stamp}_summary.md')
    open(out, 'w', encoding='utf-8').write(txt)
    json.dump({k: v for k, v in results.items()},
              open(os.path.join(WS, 'logs', f'bench_{stamp}_detail.json'), 'w', encoding='utf-8'),
              ensure_ascii=False, indent=1)
    print(txt)
    print(f'[bench_ab] 写出 {out}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
