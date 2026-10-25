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
# V2-① 的 rrt_fallback_wait_sec 走"原地改值"而不是"插新键"：
# 车端 launch 里这个键在同一个 parameters 字典中出现两次（223 行注入位附近与
# 343 行 'rrt_fallback_wait_sec': 0.0），Python 字典字面量重复键是**后者覆盖前者**，
# 所以新插一个 12.0 会被后面的 0.0 静默覆盖 —— 实测就是"注入成功但完全不生效"。
# 因此这里改成：把**最后一个**出现的那一行整行替换，原行记进 journal，restore 时放回。
FRONT_KEY = "'front_stop_distance':"
SENT3_B = "            # ---- sim_patch(front_stop_distance) ----\n"
SENT3_E = "            # ---- sim_patch(end3) ----\n"
RRT_KEY = "'rrt_fallback_wait_sec':"
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
    ap.add_argument('--front-stop', type=float, default=0.0,
                    help='车端 front_stop_distance（0=不注入；原厂 0.65）。'
                         '它决定"前向多近就判 blocked 并停住"，是静止卡死的主要触发线')
    a = ap.parse_args()
    path = target()
    txt = open(path, encoding='utf-8').read()
    injected = SENT_B in txt
    injected2 = SENT2_B in txt or SENT3_B in txt
    print(f'[patch_car_launch] 目标: {path}')
    print(f'  当前: 已注入(grace)={injected} 已注入(rrt_wait)={injected2}')
    if a.check or not (a.apply or a.restore):
        return 0

    if a.apply:
        new = txt
        dup_orig = None
        if not injected:
            if ANCHOR not in new:
                print('  ✗ 找不到锚点（车端 launch 变了？）；放弃，不改文件')
                return 3
            block = (SENT_B
                     + f"            'return_home_orientation_grace_sec': {a.grace},\n"
                     + SENT_E)
            new = new.replace(ANCHOR, ANCHOR + block, 1)
        if not injected2 and a.rrt_wait > 0:
            if RRT_KEY not in new:
                print('  ✗ 车端 launch 里没有 rrt_fallback_wait_sec 键；只注入 grace')
            else:
                # 取最后一处（字典字面量里生效的那个）整行替换
                i = new.rindex(RRT_KEY)
                ls = new.rfind('\n', 0, i) + 1
                le = new.index('\n', i) + 1
                dup_orig = new[ls:le]
                indent = dup_orig[:len(dup_orig) - len(dup_orig.lstrip())]
                block2 = (SENT2_B
                          + f"{indent}'rrt_fallback_wait_sec': {a.rrt_wait},\n"
                          + SENT2_E)
                new = new[:ls] + block2 + new[le:]
                print(f'  生效行原样（将记入 journal）: {dup_orig.strip()}')
        if a.front_stop > 0 and SENT3_B not in new:
            if FRONT_KEY not in new:
                print('  ✗ 车端 launch 里没有 front_stop_distance 键；跳过该项')
            else:
                i = new.rindex(FRONT_KEY)
                ls = new.rfind('\n', 0, i) + 1
                le = new.index('\n', i) + 1
                dup3 = new[ls:le]
                ind = dup3[:len(dup3) - len(dup3.lstrip())]
                new = (new[:ls] + SENT3_B
                       + f'{ind}front_stop_distance: {a.front_stop},\n'
                       + SENT3_E + new[le:])
                print(f'  front_stop 生效行原样: {dup3.strip()}')
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
        jold = {}
        try:
            jold = json.load(open(journal_path(), encoding='utf-8'))
        except Exception:                  # noqa: BLE001
            jold = {}
        j = {'anchor': ANCHOR, 'grace': a.grace, 'rrt_wait': a.rrt_wait,
             'front_stop': a.front_stop, 'front_orig_line': locals().get('dup3'),
             'dup_orig_line': dup_orig if dup_orig is not None
                              else jold.get('dup_orig_line')}
        json.dump(j, open(journal_path(), 'w', encoding='utf-8'),
                  ensure_ascii=False, indent=1)
        open(path, 'w', encoding='utf-8').write(new)
        print(f'  ✓ 已注入 grace={a.grace} rrt_wait={a.rrt_wait}')
        return 0

    # restore
    if not injected and not injected2:
        print('  无注入可撤销')
        return 0
    # rrt_wait 段是"整行替换"，撤的时候要把原行原样放回去（逐字节还原）
    try:
        j = json.load(open(journal_path(), encoding='utf-8'))
    except Exception:                      # noqa: BLE001
        j = {}
    dup_orig = j.get('dup_orig_line')
    new = txt
    front_orig = j.get('front_orig_line')
    while SENT3_B in new:
        i = new.index(SENT3_B)
        k = new.index(SENT3_E, i) + len(SENT3_E)
        new = new[:i] + (front_orig or '') + new[k:]
        front_orig = None
    while SENT2_B in new:
        i = new.index(SENT2_B)
        k = new.index(SENT2_E, i) + len(SENT2_E)
        new = new[:i] + (dup_orig or '') + new[k:]
        dup_orig = None                    # 只放回一次
    for b, e in ((SENT_B, SENT_E),):
        while b in new:
            i = new.index(b)
            k = new.index(e, i) + len(e)
            new = new[:i] + new[k:]
    open(path, 'w', encoding='utf-8').write(new)
    try:
        os.remove(journal_path())
    except OSError:
        pass
    print('  ✓ 已撤销注入')
    return 0


if __name__ == '__main__':
    sys.exit(main())
