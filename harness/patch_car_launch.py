#!/usr/bin/env python3
"""patch_car_launch.py —— 给车端 launch 注入**可回退**的运行参数（仅仿真用）。

背景
  `return_home_orientation_grace_sec`（返航末段"等 Nav2 精确终态"的宽限，默认 6 s）
  在车端是 `declare_parameter` 默认值 + `__init__` 里一次性缓存，**既没有 launch 入口、
  也没有参数回调**，所以 `ros2 param set` 改了也不生效。要"暂时放宽看效果"只能改 launch。

做法（沿用 vision_nav_params.py 的约定）
  * 纯文本、带哨兵注释、只加 2 行；
  * journal 记录注入前的行，`--restore` 逐字节还原（md5 必须一致）；
  * 目标文件由 `ros2 pkg prefix stemm_cartographer_exploration` 解析，不猜路径。

用法
  python3 harness/patch_car_launch.py --check [--grace 22.0]
  python3 harness/patch_car_launch.py --apply [--grace 22.0]
  python3 harness/patch_car_launch.py --restore
"""
import argparse
import json
import os
import shutil
import subprocess
import sys

ANCHOR = "            'return_home_max_retries': 3,\n"
ANCHOR2 = "            'explore_startup_delay_sec': 8.0,\n"
SENT2_B = "            # ---- sim_patch(rrt_fallback_wait) ----\n"
SENT2_E = "            # ---- sim_patch(end2) ----\n"
SENT_B = "            # ---- sim_patch(return_home_orientation_grace) ----\n"
SENT_E = "            # ---- sim_patch(end) ----\n"
CACHE = "/home/kbf/stemm_exploration_sim_ws/patches"


def target():
    try:
        r = subprocess.run(['ros2', 'pkg', 'prefix', 'stemm_cartographer_exploration'],
                           capture_output=True, text=True, timeout=20)
        if r.returncode == 0:
            p = os.path.join(r.stdout.strip(), 'share', 'stemm_cartographer_exploration',
                             'launch', 'stemm_cartographer_auto_mapping.launch.py')
            if os.path.exists(p):
                return os.path.realpath(p)
    except Exception:                     # noqa: BLE001
        pass
    return os.path.realpath('/home/kbf/wheeltec_ros2/src/stemm_cartographer_exploration/'
                            'launch/stemm_cartographer_auto_mapping.launch.py')


def journal_path():
    os.makedirs(CACHE, exist_ok=True)
    return os.path.join(CACHE, 'car_launch.journal.json')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--apply', action='store_true')
    ap.add_argument('--restore', action='store_true')
    ap.add_argument('--check', action='store_true')
    ap.add_argument('--grace', type=float, default=22.0)
    ap.add_argument('--rrt-wait', type=float, default=12.0,
                    help='V2：manager 等仲裁合格候选的窗口秒数（0=不注入）')
    a = ap.parse_args()
    path = target()
    txt = open(path, encoding='utf-8').read()
    injected = SENT_B in txt
    injected2 = SENT2_B in txt
    print(f'[patch_car_launch] 目标: {path}')
    print(f'  当前: 已注入(grace)={injected} 已注入(rrt_wait)={injected2}')
    if a.check or not (a.apply or a.restore):
        return 0

    if a.apply:
        new = txt
        if not injected:
            if ANCHOR not in new:
                print('  ✗ 找不到锚点（车端 launch 变了？）；放弃，不改文件')
                return 3
            block = (SENT_B
                     + f"            'return_home_orientation_grace_sec': {a.grace},\n"
                     + SENT_E)
            new = new.replace(ANCHOR, ANCHOR + block, 1)
        if not injected2 and a.rrt_wait > 0:
            if ANCHOR2 not in new:
                print('  ✗ 找不到 rrt_fallback_wait 锚点；只注入 grace')
            else:
                block2 = (SENT2_B
                          + f"            'rrt_fallback_wait_sec': {a.rrt_wait},\n"
                          + SENT2_E)
                new = new.replace(ANCHOR2, ANCHOR2 + block2, 1)
        if new == txt:
            print('  已注入，跳过')
            return 0
        # 语法校验：注入后必须能被 ast 解析
        try:
            compile(new, path, 'exec')
        except SyntaxError as e:
            print(f'  ✗ 注入后语法错误，放弃：{e}')
            return 3
        backup = os.path.join(CACHE, 'car_launch.pristine.py')
        if not os.path.exists(backup):
            shutil.copy2(path, backup)
        json.dump({'anchor': ANCHOR, 'grace': a.grace, 'rrt_wait': a.rrt_wait},
                  open(journal_path(), 'w', encoding='utf-8'), ensure_ascii=False, indent=1)
        open(path, 'w', encoding='utf-8').write(new)
        print(f'  ✓ 已注入 grace={a.grace} rrt_wait={a.rrt_wait}')
        return 0

    # restore
    if not injected and not injected2:
        print('  无注入可撤销')
        return 0
    new = txt
    for b, e in ((SENT_B, SENT_E), (SENT2_B, SENT2_E)):
        while b in new:
            i = new.index(b)
            j = new.index(e, i) + len(e)
            new = new[:i] + new[j:]
    open(path, 'w', encoding='utf-8').write(new)
    try:
        os.remove(journal_path())
    except OSError:
        pass
    print('  ✓ 已撤销注入')
    return 0


if __name__ == '__main__':
    sys.exit(main())
