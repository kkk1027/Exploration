#!/usr/bin/env python3
"""patch_planner.py —— 全局规划器的两处**可回退**开关（用户 2026-10-26 批准 ①②）。

1) `allow_unknown: false`：不再把"没建图的地方"当可行。
   现场证据：车在 (-7.28,-5.10) 死角卡死，nav2 给的计划最小净宽只有 0.40~0.60 m，
   而那个角在车身后侧**没建图**（雷达有 110°~240° 盲区）—— allow_unknown=true 时
   规划器把黑暗区当自由空间，直接把人往里带；进去后前向安全闸把车按住，出不来。
2) `motion_model_for_search: REEDS_SHEPP`：允许计划里包含一小段倒车。
   原来 DUBIN 是**只前向**的：一旦车头贴死，规划器永远只能"往前想办法"，
   而 vx_min 又不能后退 ⇒ 结构性死锁。REEDS_SHEPP 让计划本身带倒车段，
   配合受限 vx_min（短距小速度）才能真正执行。

可回退：pristine 快照 + journal + `--restore` 逐字节还原（与其它 patch 脚本同款）。
用法：
  python3 harness/patch_planner.py --check
  python3 harness/patch_planner.py --apply --allow-unknown false --motion REEDS_SHEPP --reverse-penalty 3.0
  python3 harness/patch_planner.py --restore
"""
import argparse
import json
import os
import shutil
import sys

TARGET = ('/home/kbf/wheeltec_ros2/src/stemm_cartographer_exploration/config/'
          'stemm_safe_nav2_params.yaml')
CACHE = '/home/kbf/stemm_exploration_sim_ws/patches'
PRISTINE = os.path.join(CACHE, 'planner.pristine.yaml')
SENT_B = '    # ---- sim_patch(planner) ----\n'
SENT_E = '    # ---- sim_patch(end_planner) ----\n'


def journal():
    os.makedirs(CACHE, exist_ok=True)
    return os.path.join(CACHE, 'planner.journal.json')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--apply', action='store_true')
    ap.add_argument('--restore', action='store_true')
    ap.add_argument('--check', action='store_true')
    ap.add_argument('--allow-unknown', choices=['true', 'false'], default='false')
    ap.add_argument('--motion', default='REEDS_SHEPP',
                    choices=['DUBIN', 'REEDS_SHEPP', 'GRID_A_STAR', 'STATE_LATTICE'])
    ap.add_argument('--reverse-penalty', type=float, default=3.0)
    a = ap.parse_args()
    if not os.path.exists(TARGET):
        print(f'✗ 找不到 {TARGET}')
        return 3
    txt = open(TARGET, encoding='utf-8').read()
    print(f'[patch_planner] 目标: {TARGET}')
    print(f'  当前: 已注入={SENT_B in txt}')
    if a.check or not (a.apply or a.restore):
        cur_au = [l.strip() for l in txt.splitlines() if l.strip().startswith('allow_unknown')]
        cur_mm = [l.strip() for l in txt.splitlines() if l.strip().startswith('motion_model_for_search')]
        print('   现有 allow_unknown 行:', cur_au[:2])
        print('   现有 motion_model 行:', cur_mm[:2])
        return 0

    if a.restore:
        _alt = [p for p in (PRISTINE, os.path.join(CACHE, 'costmap_infl.pristine.yaml'),
                            os.path.join(CACHE, 'nav_speed.pristine.yaml')) if os.path.exists(p)]
        if not _alt:
            print('  无 pristine 备份可还原')
            return 0
        # 优先用"最早的真原厂"：nav_speed 的 pristine 一定是在所有注入之前拍的
        src = os.path.join(CACHE, 'nav_speed.pristine.yaml')
        if not os.path.exists(src):
            src = _alt[0]
        shutil.copy2(src, TARGET)
        try:
            os.remove(journal())
        except OSError:
            pass
        print('  ✓ 已按真原厂备份逐字节还原')
        return 0

    if not os.path.exists(PRISTINE):
        shutil.copy2(TARGET, PRISTINE)
        print(f'  已保存快照：{PRISTINE}')
    pristine = open(PRISTINE, encoding='utf-8').read()
    new = pristine
    repl = []
    # allow_unknown
    lines = new.splitlines(keepends=True)
    out = []
    for ln in lines:
        st = ln.strip()
        if st.startswith('allow_unknown:'):
            indent = ln[:len(ln) - len(ln.lstrip())]
            out.append(f'{indent}allow_unknown: {a.allow_unknown}'
                       f'    # sim_patch：不再把未建图区当可行（{a.allow_unknown}）\n')
            repl.append('allow_unknown')
            continue
        if st.startswith('motion_model_for_search:'):
            indent = ln[:len(ln) - len(ln.lstrip())]
            out.append(f'{indent}motion_model_for_search: "{a.motion}"'
                       f'    # sim_patch：允许计划里含倒车段（原 DUBIN=只前向）\n')
            repl.append('motion_model')
            continue
        out.append(ln)
    new = ''.join(out)
    if 'motion_model' in repl:
        # reverse_penalty 紧跟在 motion_model 行后插入（SmacPlannerHybrid 支持）
        lines = new.splitlines(keepends=True)
        out = []
        for ln in lines:
            out.append(ln)
            if ln.strip().startswith('motion_model_for_search:'):
                indent = ln[:len(ln) - len(ln.lstrip())]
                out.append(f'{indent}reverse_penalty: {a.reverse_penalty}'
                           f'    # sim_patch：倒车要比前进"贵"，避免计划里到处倒车\n')
        new = ''.join(out)
    if not repl:
        print('  ✗ 没找到 allow_unknown / motion_model_for_search 锚点；放弃')
        return 3
    new = SENT_B + new + SENT_E
    try:
        import yaml
        yaml.safe_load(new)
    except Exception as exc:                    # noqa: BLE001
        print(f'  ✗ 注入后 YAML 非法，放弃：{exc}')
        return 3
    json.dump({'allow_unknown': a.allow_unknown, 'motion': a.motion,
               'reverse_penalty': a.reverse_penalty, 'replaced': repl},
              open(journal(), 'w', encoding='utf-8'), ensure_ascii=False, indent=1)
    open(TARGET, 'w', encoding='utf-8').write(new)
    print(f'  ✓ 已注入：allow_unknown={a.allow_unknown}  motion_model={a.motion}'
          f'  reverse_penalty={a.reverse_penalty}（替换 {len(repl)} 处）')
    return 0


if __name__ == '__main__':
    sys.exit(main())
