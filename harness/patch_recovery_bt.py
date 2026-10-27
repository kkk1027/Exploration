#!/usr/bin/env python3
"""patch_recovery_bt.py —— Nav2 恢复动作/重规划节拍的**可回退**开关（两项各自独立）。

背景（2026-10-26 查证）：车端 BT `stemm_no_backup_nav_to_pose.xml` 里
  * `<Wait wait_duration="2"/>` 占着原本 `<Spin/>` 的位置 —— 恢复动作**不改变位姿**；
  * `<RateController hz="0.3">` 包着 `ComputePathToPose` —— 全局路径 **3.3 s** 才重算一次。
于是"车贴死在通道里"时：控制器不能倒车(vx_min=0)、恢复只会清代价地图+干等、
重规划算出的是同一条路 → 死锁稳定复现（用户截图里的"路径不刷新"）。

**关键实测（2026-10-26 晚）**：把 Spin 装回去后，有一轮在 (-9.10,-0.99) 的墙角
被顶住时，原地旋转命令让车体与墙**物理接触 94 次**（base 5 轮全为 0）——
即"恢复时改变位姿"在**已经被卡死**的场景里会拿接触换位移。
因此两项必须是**独立开关**：`--spin on|off` + `--replan-hz`，可单独回退归因。

做法（幂等、逐字节回退）：
  * 首次 apply 时把原文件存成 patches/recovery_bt.pristine.xml（唯一真源）；
  * 每次 apply 都从 pristine 重新构造目标文本（不叠加历史修改）；
  * --restore 直接用 pristine 覆盖回来，逐字节一致。
用法：
  python3 harness/patch_recovery_bt.py --check
  python3 harness/patch_recovery_bt.py --apply --spin on --spin-dist 0.6 --spin-time 6.0 --replan-hz 1.0
  python3 harness/patch_recovery_bt.py --apply --spin off --replan-hz 1.0     # 只提节拍
  python3 harness/patch_recovery_bt.py --restore
"""
import argparse
import json
import os
import shutil
import sys

BT = ('/home/kbf/stemm_explore_ws/install/stemm_nav2_manager/share/stemm_nav2_manager/'
      'config/stemm_no_backup_nav_to_pose.xml')
CACHE = '/home/kbf/stemm_exploration_sim_ws/patches'
PRISTINE = os.path.join(CACHE, 'recovery_bt.pristine.xml')
# 注意：XML 注释里**不能出现 `--`**，所以哨兵不能写成 `<!-- ---- x ---- -->`
SENT_B = '<!-- sim_patch recovery_bt begin -->\n'
SENT_E = '<!-- sim_patch recovery_bt end -->\n'
WAIT2 = '        <Wait wait_duration="2"/>\n'
HZ_ORIG = '0.3'


def journal():
    os.makedirs(CACHE, exist_ok=True)
    return os.path.join(CACHE, 'recovery_bt.journal.json')


def build(pristine, spin, spin_dist, spin_time, replan_hz, backup=None):
    """从 pristine 构造目标文本：只做两件（可分别关掉的）替换。"""
    new = pristine
    if spin:
        if WAIT2 not in new:
            raise SystemExit('✗ pristine 里找不到 `<Wait wait_duration="2"/>`；无法装回 Spin')
        new = new.replace(
            WAIT2,
            SENT_B + f'        <Spin spin_dist="{spin_dist}" time_allowance="{spin_time}"/>\n' + SENT_E,
            1)
    elif backup is not None and backup > 0:
        # 统一口径（2026-10-27）：恢复动作里必须有"能改变位置"的一项。原来只剩 Wait，
        # 车在窄处只会原地打转；这里把空闲的 <Wait 2/> 位替换成 BackUp(+保留 Wait)。
        if WAIT2 not in new:
            raise SystemExit('✗ pristine 里找不到 `<Wait wait_duration="2"/>`；无法插入 BackUp')
        new = new.replace(
            WAIT2,
            SENT_B + f'        <BackUp backup_dist="{backup}" backup_speed="0.05"'
                     ' time_allowance="20"/>\n' + SENT_E + WAIT2,
            1)
    new = new.replace(f'<RateController hz="{HZ_ORIG}">', f'<RateController hz="{replan_hz}">', 1)
    import xml.etree.ElementTree as ET
    ET.fromstring(new)                      # XML 非法就别写进安装目录
    return new


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--apply', action='store_true')
    ap.add_argument('--restore', action='store_true')
    ap.add_argument('--check', action='store_true')
    ap.add_argument('--spin', choices=['on', 'off'], default='on')
    ap.add_argument('--backup', type=float, default=0.0,
                    help='在恢复子树里插入 BackUp（米，0=不插；仅 --spin off 时生效）')
    ap.add_argument('--spin-dist', type=float, default=0.6,
                    help='受限自转角（rad）；原厂 BT 是 1.57')
    ap.add_argument('--spin-time', type=float, default=6.0)
    ap.add_argument('--replan-hz', type=float, default=1.0)
    a = ap.parse_args()
    real = os.path.realpath(BT)
    if not os.path.exists(real):
        print(f'✗ 找不到 BT 文件：{BT} -> {real}')
        return 3
    txt = open(real, encoding='utf-8').read()
    state = {'spin': 'Spin' if SENT_B in txt else 'Wait(2)',
             'replan_hz': '1.0' if '<RateController hz="1.0">' in txt else HZ_ORIG}
    print(f'[recovery_bt] 目标: {real}')
    print(f'  当前: 恢复动作={state["spin"]}  重规划={state["replan_hz"]} Hz  '
          f'(pristine 存在={os.path.exists(PRISTINE)})')
    if a.check or not (a.apply or a.restore):
        return 0

    if a.restore:
        if not os.path.exists(PRISTINE):
            print('  无 pristine 备份可还原（说明从未 apply 过）')
            return 0
        shutil.copy2(PRISTINE, real)
        try:
            os.remove(journal())
        except OSError:
            pass
        print('  ✓ 已按 pristine 逐字节还原（恢复动作=Wait(2)、重规划=0.3 Hz）')
        return 0

    if not os.path.exists(PRISTINE):
        shutil.copy2(real, PRISTINE)
        print(f'  已保存原始备份：{PRISTINE}')
    pristine = open(PRISTINE, encoding='utf-8').read()
    spin = (a.spin == 'on')
    new = build(pristine, spin, a.spin_dist, a.spin_time, a.replan_hz,
                None if spin else a.backup)
    json.dump({'file': real, 'spin': spin, 'spin_dist': a.spin_dist,
               'spin_time': a.spin_time, 'replan_hz': a.replan_hz,
               'backup': None if spin else a.backup},
              open(journal(), 'w', encoding='utf-8'), ensure_ascii=False, indent=1)
    open(real, 'w', encoding='utf-8').write(new)
    print('  ✓ 已写入：恢复动作=%s  重规划=%.1f Hz'
          % ('Spin(%.2f rad/%.0fs)' % (a.spin_dist, a.spin_time) if spin
             else ('BackUp(%.2fm)+Wait(2)' % a.backup if (a.backup or 0) > 0 else 'Wait(2)'),
             a.replan_hz))
    return 0


if __name__ == '__main__':
    sys.exit(main())
