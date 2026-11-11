#!/usr/bin/env python3
"""nav_params.py —— 车端 nav2 参数的**唯一**可回退注入器（一次生成，逐字节还原）。

为什么合并（2026-10-26 事故）：
  vx_max / vx_min / 代价地图膨胀 / 规划器 四处注入原本是三个脚本，各自保存一份
  pristine 快照、各自 --restore。它们改的是**同一个** YAML，于是互相踩：
  一次 `patch_planner.py --restore` 把导航速度注入连带抹掉（现场症状：
  `FollowPath.vx_min` 仍是 0.0、`GridBased.reverse_penalty` 仍是 1.2）。
  另外 ros 的 YAML 解析**先出现者优先**，所以"追加"式注入有时根本不生效。

现在的规矩：
  * 只有一份真原厂快照 `patches/stemm_safe_nav2_params.pristine.yaml`（首次 apply 时创建）；
  * 每次 apply 都从快照**重新生成**整份文件（不叠加历史修改），因此天然幂等；
  * 开关全部显式：不给的项保持原厂值；
  * `--restore` 直接拷回快照，逐字节一致。

开关：
  --vx-max 0.35            MPPI 前进限速
  --vx-min -0.08           MPPI 倒车下限（受限倒车；不给=原厂 0.0）
  --infl-radius 1.0        全局膨胀半径
  --infl-scaling 1.2       全局代价衰减系数（越小衰减越慢=离障碍远才便宜）
  --pad 0.10               footprint_padding（两个 costmap，内切半径加厚）
  --allow-unknown false    全局规划器是否允许穿越未建图区域
  --motion REEDS_SHEPP     运动模型（DUBIN=只前向 / REEDS_SHEPP=允许倒车段）
  --reverse-penalty 3.0    RS 模型的倒车惩罚（>1；越大越不爱倒车）
用法：
  python3 harness/nav_params.py --check
  python3 harness/nav_params.py --apply --vx-max 0.35 --vx-min -0.08 \
      --infl-radius 1.0 --infl-scaling 1.2 --pad 0.10 \
      --allow-unknown false --motion REEDS_SHEPP --reverse-penalty 3.0
  python3 harness/nav_params.py --restore
"""
import argparse
import json
import os
import shutil
import sys

TARGET = ('/home/kbf/wheeltec_ros2/src/stemm_cartographer_exploration/config/'
          'stemm_safe_nav2_params.yaml')
CACHE = '/home/kbf/stemm_exploration_sim_ws/patches'
PRISTINE = os.path.join(CACHE, 'stemm_safe_nav2_params.pristine.yaml')
SENT = '    # ---- sim_patch(nav_params) ----\n'


def journal():
    return os.path.join(CACHE, 'nav_params.journal.json')


def ensure_pristine():
    """真原厂快照：优先用 git 里那一份（未被注入的 HEAD 版本），否则用现存最早备份。"""
    if os.path.exists(PRISTINE):
        return PRISTINE
    for cand in (os.path.join(CACHE, 'nav_speed.pristine.yaml'),):
        if os.path.exists(cand):
            shutil.copy2(cand, PRISTINE)
            return PRISTINE
    shutil.copy2(TARGET, PRISTINE)
    return PRISTINE


def patch_text(txt, a):
    """从原厂文本生成目标文本；返回 (新文本, 生效项列表)。"""
    out = []
    done = []
    # footprint_padding：按**精确的三行锚点**插（costmap 节点自己的 ros__parameters），
    # 不能用"哪个 ros__parameters 先出现"——同一段里还有 local_costmap_client /
    # local_costmap_rclcpp_node 这些内部节点的块，会一起被塞进去（实测一次塞了 4 份）。
    if a.pad is not None:
        for sec in ('local_costmap', 'global_costmap'):
            anchor = f'{sec}:\n  {sec}:\n    ros__parameters:\n'
            if anchor in txt:
                txt = txt.replace(
                    anchor,
                    anchor + f'      footprint_padding: {a.pad}    # sim_patch：内切半径加厚\n', 1)
                done.append('pad_' + sec)
    lines = txt.splitlines(keepends=True)
    in_global = False
    in_local = False
    pad_slots = []          # 待插入 footprint_padding 的段（只允许两个 costmap 各一次）
    for ln in lines:
        st = ln.strip()
        ind = ln[:len(ln) - len(ln.lstrip())]
        if st.startswith('global_costmap:'):
            in_global, in_local = True, False
            pad_slots.append('global')
        elif st.startswith('local_costmap:'):
            in_local, in_global = True, False
            pad_slots.append('local')
        elif st and not ln[0].isspace() and not st.startswith('#'):
            # 离开 costmap 段后必须复位，否则后面的其它节点也会被塞 footprint_padding
            in_global = in_local = False
        # 记录当前处于哪个 costmap 段（第一层缩进里的 inflation 块按段判断）
        if st.startswith('footprint:') and a.footprint is not None:
            out.append(f'{ind}footprint: "{a.footprint}"    # sim_patch(footprint)：统一口径（车体+安全余量）\n')
            done.append('footprint')
            continue
        if st.startswith('vx_max:') and a.vx_max is not None:
            out.append(f'{ind}vx_max: {a.vx_max}    # sim_patch\n')
            done.append('vx_max')
            continue
        if st.startswith('vx_min:') and a.vx_min is not None:
            out.append(f'{ind}vx_min: {a.vx_min}    # sim_patch：受限倒车（原厂 0.0 = 物理上不能退）\n')
            done.append('vx_min')
            continue
        if st.startswith('inflation_radius:') and a.infl_radius is not None and in_global:
            out.append(f'{ind}inflation_radius: {a.infl_radius}    # sim_patch(global)\n')
            done.append('infl_radius')
            continue
        if st.startswith('cost_scaling_factor:') and a.infl_scaling is not None and in_global:
            out.append(f'{ind}cost_scaling_factor: {a.infl_scaling}    # sim_patch(global)\n')
            done.append('infl_scaling')
            continue
        if st.startswith('downsample_costmap:') and a.smac_downsample and a.smac_downsample > 1:
            out.append(f'{ind}downsample_costmap: true    # sim_patch：降采样压搜索规模\n')
            done.append('downsample_costmap')
            continue
        if st.startswith('downsampling_factor:') and a.smac_downsample and a.smac_downsample > 1:
            out.append(f'{ind}downsampling_factor: {int(a.smac_downsample)}    # sim_patch\n')
            done.append('downsampling_factor')
            continue
        if st.startswith('allow_unknown:') and a.allow_unknown is not None:
            out.append(f'{ind}allow_unknown: {a.allow_unknown}    # sim_patch：不把没建图的区域当可行\n')
            done.append('allow_unknown')
            continue
        if st.startswith('motion_model_for_search:') and a.motion is not None:
            out.append(f'{ind}motion_model_for_search: "{a.motion}"    # sim_patch\n')
            done.append('motion')
            continue
        if st.startswith('cost_penalty:') and a.cost_penalty is not None:
            out.append(f'{ind}cost_penalty: {a.cost_penalty}    # sim_patch：窄缝贵到不愿钻\n')
            done.append('cost_penalty')
            continue
        if st.startswith('reverse_penalty:') and a.reverse_penalty is not None:
            out.append(f'{ind}reverse_penalty: {a.reverse_penalty}    # sim_patch：倒车更贵\n')
            done.append('reverse_penalty')
            continue
        # S4（2026-11-11）：切换 BT —— 挂上 GoalUpdater 后，后置否决可以「原地改向」
        # 而不是「取消重派」。必须配合已重编的 nav2_behavior_tree（跨目标污染已修）。
        if st.startswith('default_nav_to_pose_bt_xml:') and a.bt_xml is not None:
            out.append(f'{ind}default_nav_to_pose_bt_xml: "{a.bt_xml}"    # sim_patch(S4)：挂 GoalUpdater\n')
            done.append('bt_xml')
            continue
        out.append(ln)
        # footprint_padding：插在 costmap 的 ros__parameters: 之后（两段各一次）
    return SENT + ''.join(out), done


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--apply', action='store_true')
    ap.add_argument('--restore', action='store_true')
    ap.add_argument('--check', action='store_true')
    ap.add_argument('--vx-max', type=float, default=None)
    ap.add_argument('--vx-min', type=float, default=None)
    ap.add_argument('--smac-downsample', type=int, default=None,
                    help='SmacPlannerHybrid 降采样倍数（2~4）；>1 时同时打开 downsample_costmap')
    ap.add_argument('--infl-radius', type=float, default=None)
    ap.add_argument('--infl-scaling', type=float, default=None)
    ap.add_argument('--pad', type=float, default=None)
    ap.add_argument('--footprint', default=None,
                    help='车体轮廓（统一口径用）：形如 "[[-0.18,-0.485],[-0.18,0.485],'
                         '[0.32,0.485],[0.32,-0.485]]"。两张 costmap 一起改，逐字节可回退。')
    ap.add_argument('--allow-unknown', choices=['true', 'false'], default=None)
    ap.add_argument('--motion', default=None,
                    choices=['DUBIN', 'REEDS_SHEPP', 'GRID_A_STAR', 'STATE_LATTICE'])
    ap.add_argument('--reverse-penalty', type=float, default=None)
    ap.add_argument('--bt-xml', default=None,
                    help='S4：覆盖 default_nav_to_pose_bt_xml（用于挂 GoalUpdater）')
    ap.add_argument('--cost-penalty', type=float, default=None,
                    help='Smac 的 cost_penalty：代价地图的代价按"每米 × 系数"累加，'
                         '系数调大就会宁可绕远也不穿高代价的窄缝')
    a = ap.parse_args()
    if not os.path.exists(TARGET):
        print(f'✗ 找不到 {TARGET}')
        return 3
    print(f'[nav_params] 目标: {TARGET}')
    cur = open(TARGET, encoding='utf-8').read()
    print(f'  已注入={SENT in cur}  pristine 存在={os.path.exists(PRISTINE)}')
    if a.check or not (a.apply or a.restore):
        for key in ('vx_max', 'vx_min', 'footprint_padding', 'allow_unknown',
                    'motion_model_for_search', 'reverse_penalty',
                    'downsample_costmap', 'downsampling_factor'):
            rows = [l.strip() for l in cur.splitlines() if l.strip().startswith(key + ':')]
            print(f'   {key:24s} {rows[:3]}')
        return 0

    if a.restore:
        src = ensure_pristine()
        shutil.copy2(src, TARGET)
        try:
            os.remove(journal())
        except OSError:
            pass
        print('  ✓ 已按真原厂快照逐字节还原')
        return 0

    pristine_path = ensure_pristine()
    pristine = open(pristine_path, encoding='utf-8').read()
    new, done = patch_text(pristine, a)
    if not done:
        print('  ✗ 没有命中任何锚点（参数名可能变了）')
        return 3
    try:
        import yaml
        yaml.safe_load(new)
    except Exception as exc:                    # noqa: BLE001
        print(f'  ✗ 注入后 YAML 非法，放弃：{exc}')
        return 3
    json.dump({k: v for k, v in vars(a).items() if k not in ('apply', 'restore', 'check')},
              open(journal(), 'w', encoding='utf-8'), ensure_ascii=False, indent=1)
    open(TARGET, 'w', encoding='utf-8').write(new)
    print(f'  ✓ 已生效 {len(done)} 项：{", ".join(sorted(set(done)))}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
