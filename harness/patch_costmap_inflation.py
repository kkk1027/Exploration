#!/usr/bin/env python3
"""patch_costmap_inflation.py —— 代价地图"膨胀/惩罚"的可回退注入（治"专挑窄处走"）。

用户 2026-10-26 指示：**小车明明有更宽的路，却偏偏规划要穿过略宽于车身的窄处**，
要求"增大膨胀半径或者想其他办法，让规划的路径永远不经过略大于车身的窄处"。

先讲清机制（避免又调一个空旋钮）：
  * nav2 的 InflationLayer 只在**内切半径**（本例 footprint 内切 ≈0.235 m）内给致命代价；
    再往外按 `253·exp(-cost_scaling·(d-内切))` 衰减到 `inflation_radius`。
  * 所以"膨胀半径 0.45"并不禁止 0.6 m 宽的缝：缝中央离两侧各 0.3 m，
    代价约 `253·exp(-3·0.065)≈207` —— 只是"贵"，不是"不通"。规划器权衡长度后照样钻。
  * 想让窄缝**结构性**变贵/变不通，两条都要动：
      ① `inflation_radius` 放大（0.40→1.00）+ `cost_scaling_factor` 调小（3.0→1.2）：
         1 m 内都保持高代价 → 绕远一点反而更便宜；
      ② `footprint_padding` 加厚（0→0.10）：内切半径 0.235→0.335，
         于是净宽 < 0.67 m 的缝直接致命，规划器连试都不会试。

可回退：哨兵注释 + journal + `--restore` 逐字节还原（与其它 patch 脚本同款）。
用法：
  python3 harness/patch_costmap_inflation.py --check
  python3 harness/patch_costmap_inflation.py --apply --global-radius 1.00 --global-scaling 1.2 --pad 0.10
  python3 harness/patch_costmap_inflation.py --restore
"""
import argparse
import json
import os
import shutil
import sys

TARGET = ('/home/kbf/wheeltec_ros2/src/stemm_cartographer_exploration/config/'
          'stemm_safe_nav2_params.yaml')
CACHE = '/home/kbf/stemm_exploration_sim_ws/patches'
PRISTINE = os.path.join(CACHE, 'costmap_infl.pristine.yaml')
SENT_B = '    # ---- sim_patch(costmap_infl) ----\n'
SENT_E = '    # ---- sim_patch(end_costmap_infl) ----\n'
# global_costmap 的 inflation 块（第一处 cost_scaling_factor: 3.0 之后紧跟 inflation_radius: 0.4）
G_SCAL = '        cost_scaling_factor: 3.0\n'
G_RAD = '        inflation_radius: 0.4\n'


def journal():
    os.makedirs(CACHE, exist_ok=True)
    return os.path.join(CACHE, 'costmap_infl.journal.json')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--apply', action='store_true')
    ap.add_argument('--restore', action='store_true')
    ap.add_argument('--check', action='store_true')
    ap.add_argument('--global-radius', type=float, default=1.00)
    ap.add_argument('--global-scaling', type=float, default=1.2)
    ap.add_argument('--pad', type=float, default=0.10,
                    help='footprint_padding（两个 costmap 都加）；0=不加')
    a = ap.parse_args()
    if not os.path.exists(TARGET):
        print(f'✗ 找不到 {TARGET}')
        return 3
    txt = open(TARGET, encoding='utf-8').read()
    injected = SENT_B in txt
    print(f'[patch_costmap_inflation] 目标: {TARGET}')
    print(f'  当前: 已注入={injected}  文件中 footprint_padding 出现 {txt.count("footprint_padding")} 次')
    if a.check or not (a.apply or a.restore):
        return 0

    if a.restore:
        if not os.path.exists(PRISTINE):
            print('  无 pristine 备份可还原（说明从未 apply 过）')
            return 0
        shutil.copy2(PRISTINE, TARGET)
        try:
            os.remove(journal())
        except OSError:
            pass
        print('  ✓ 已按 pristine 逐字节还原')
        return 0

    if not os.path.exists(PRISTINE):
        shutil.copy2(TARGET, PRISTINE)
        print(f'  已保存原始备份：{PRISTINE}')
    pristine = open(PRISTINE, encoding='utf-8').read()

    new = pristine
    if G_SCAL not in new or G_RAD not in new:
        print('  ✗ 找不到 global inflation 锚点（cost_scaling_factor: 3.0 / inflation_radius: 0.4）')
        return 3
    # 只在 global_costmap 段内替换：用"global_costmap:"之后的第一处
    gi = new.index('global_costmap:')
    seg = new[gi:]
    seg = seg.replace(G_SCAL, f'        cost_scaling_factor: {a.global_scaling}\n', 1)
    seg = seg.replace(G_RAD, f'        inflation_radius: {a.global_radius}\n', 1)
    new = new[:gi] + seg
    # footprint_padding：插在两个 costmap 的 inflation_layer 块前
    if a.pad and a.pad > 0:
        # 必须插在 `ros__parameters:` 的下一行（6 空格缩进），否则不是节点参数
        for anchor in ('local_costmap:\n  local_costmap:', 'global_costmap:\n  global_costmap:'):
            i = new.index(anchor)
            k = new.index('ros__parameters:', i)
            j = new.index('\n', k) + 1
            new = new[:j] + f'      footprint_padding: {a.pad}\n' + new[j:]
    # 哨兵：标记本次注入（evaluate/审计用来判断是否生效）
    new = SENT_B + new + SENT_E
    try:
        import yaml
        d = yaml.safe_load(new)
        gc = (d.get('global_costmap') or {}).get('global_costmap') or {}
        gc = gc.get('ros__parameters') or {}
        lc = (d.get('local_costmap') or {}).get('local_costmap') or {}
        lc = lc.get('ros__parameters') or {}
        gi = gc.get('inflation_layer') or {}
        print('  注入后：global inflation_radius=%s cost_scaling=%s  '
              'footprint_padding local/global=%s/%s'
              % (gi.get('inflation_radius'), gi.get('cost_scaling_factor'),
                 lc.get('footprint_padding'), gc.get('footprint_padding')))
    except Exception as exc:                    # noqa: BLE001
        print(f'  ✗ 注入后 YAML 非法，放弃：{exc}')
        return 3
    json.dump({'global_radius': a.global_radius, 'global_scaling': a.global_scaling,
               'pad': a.pad}, open(journal(), 'w', encoding='utf-8'), ensure_ascii=False, indent=1)
    open(TARGET, 'w', encoding='utf-8').write(new)
    print('  ✓ 已注入')
    return 0


if __name__ == '__main__':
    sys.exit(main())
