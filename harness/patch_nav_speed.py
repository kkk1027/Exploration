#!/usr/bin/env python3
"""patch_nav_speed.py —— 车端 nav2 参数里**速度上限**的可回退注入（仅仿真用）。

背景（2026-10-25 P0）：探索期卡住的主要形态是"车以 vx_max=0.5 m/s 冲向死角，
front 掉到 0.34 m 后被安全门限按住"。速度是**一次性固定参数决策**（不做场景调参），
所以把它做成可注入项，方便 A/B 出"0.5 还是 0.35"。

做法与 patch_car_launch.py 一致：整行替换 + journal + `--restore` 逐字节还原。
用法：
    python3 harness/patch_nav_speed.py --apply --vx-max 0.35
    python3 harness/patch_nav_speed.py --restore
    python3 harness/patch_nav_speed.py --check
"""
import argparse
import json
import os
import shutil
import sys

TARGET = ('/home/kbf/wheeltec_ros2/src/stemm_cartographer_exploration/config/'
          'stemm_safe_nav2_params.yaml')
CACHE = '/home/kbf/stemm_exploration_sim_ws/patches'
SENT_B = '      # ---- sim_patch(vx_max) ----\n'
SENT_E = '      # ---- sim_patch(end_vx_max) ----\n'
KEY = '      vx_max: '


def journal():
    os.makedirs(CACHE, exist_ok=True)
    return os.path.join(CACHE, 'nav_speed.journal.json')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--apply', action='store_true')
    ap.add_argument('--restore', action='store_true')
    ap.add_argument('--check', action='store_true')
    ap.add_argument('--vx-max', type=float, default=0.35)
    a = ap.parse_args()
    if not os.path.exists(TARGET):
        print(f'✗ 找不到 {TARGET}')
        return 3
    txt = open(TARGET, encoding='utf-8').read()
    injected = SENT_B in txt
    print(f'[patch_nav_speed] 目标: {TARGET}')
    print(f'  当前: 已注入={injected}')
    if a.check or not (a.apply or a.restore):
        return 0

    if a.apply:
        if injected:
            print('  已注入，跳过')
            return 0
        i = txt.index(KEY)
        ls = txt.rfind('\n', 0, i) + 1
        le = txt.index('\n', i) + 1
        orig = txt[ls:le]
        if orig.strip().startswith('#'):
            print(f'  ✗ 找到的 vx_max 行是注释：{orig.strip()}；放弃')
            return 3
        new = (txt[:ls] + SENT_B + f'      vx_max: {a.vx_max}\n' + SENT_E
               + txt[le:])
        try:
            compile(new, TARGET, 'exec') if TARGET.endswith('.py') else None
        except SyntaxError as exc:            # noqa: BLE001
            print(f'  ✗ 注入后语法错误，放弃：{exc}')
            return 3
        bak = os.path.join(CACHE, 'nav_speed.pristine.yaml')
        if not os.path.exists(bak):
            shutil.copy2(TARGET, bak)
        json.dump({'key': KEY, 'vx_max': a.vx_max, 'orig_line': orig},
                  open(journal(), 'w', encoding='utf-8'), ensure_ascii=False, indent=1)
        open(TARGET, 'w', encoding='utf-8').write(new)
        print(f'  ✓ 已注入 vx_max={a.vx_max}（原行：{orig.strip()}）')
        return 0

    if not injected:
        print('  无注入可撤销')
        return 0
    try:
        j = json.load(open(journal(), encoding='utf-8'))
    except Exception:                         # noqa: BLE001
        j = {}
    orig = j.get('orig_line') or f'{KEY}0.5\n'
    i = txt.index(SENT_B)
    k = txt.index(SENT_E, i) + len(SENT_E)
    open(TARGET, 'w', encoding='utf-8').write(txt[:i] + orig + txt[k:])
    try:
        os.remove(journal())
    except OSError:
        pass
    print('  ✓ 已撤销注入')
    return 0


if __name__ == '__main__':
    sys.exit(main())
