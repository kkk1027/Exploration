#!/usr/bin/env python3
"""patch_recovery_bt.py —— 让 Nav2 的"恢复动作"重新具备改变位姿的能力，并调重规划节拍。

背景（2026-10-26 查证）：车端 BT `stemm_no_backup_nav_to_pose.xml` 里
  * `<Wait wait_duration="2"/>` 占据了原本 `<Spin/>` 的位置 —— 恢复动作**不再改变位姿**；
  * `<RateController hz="0.3">` 包着 `ComputePathToPose` —— 全局路径 **3.3 s** 才重算一次。
于是"车贴死在通道里"时：控制器不能倒车(vx_min=0)、恢复只会清代价地图+干等、
重规划算出的是同一条路 → 死锁稳定复现（用户截图里的"路径不刷新"）。

做法：对**安装目录里的 BT 文件**做两处可回退替换（哨兵注释 + journal + 逐字节还原）：
  1) `<Wait wait_duration="2"/>`  → `<Spin spin_dist="R" time_allowance="T"/>`
  2) `<RateController hz="0.3">`  → `<RateController hz="1.0">`
用法：
  python3 harness/patch_recovery_bt.py --check
  python3 harness/patch_recovery_bt.py --apply [--spin-dist 0.6] [--spin-time 6.0] [--replan-hz 1.0]
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
# 注意：XML 注释里**不能出现 `--`**，所以哨兵不能写成 `<!-- ---- x ---- -->`
# （曾经因此让注入后被 ET.fromstring 判为 not well-formed，注入直接放弃）。
SENT_B = '<!-- sim_patch recovery_bt begin -->\n'
SENT_E = '<!-- sim_patch recovery_bt end -->\n'


def journal():
    os.makedirs(CACHE, exist_ok=True)
    return os.path.join(CACHE, 'recovery_bt.journal.json')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--apply', action='store_true')
    ap.add_argument('--restore', action='store_true')
    ap.add_argument('--check', action='store_true')
    ap.add_argument('--spin-dist', type=float, default=0.6,
                    help='受限自转角（rad）。原来 BT 里是 1.57，这里取 0.6 让步进更小')
    ap.add_argument('--spin-time', type=float, default=6.0)
    ap.add_argument('--replan-hz', type=float, default=1.0)
    a = ap.parse_args()
    real = os.path.realpath(BT)
    if not os.path.exists(real):
        print(f'✗ 找不到 BT 文件：{BT} -> {real}')
        return 3
    txt = open(real, encoding='utf-8').read()
    injected = SENT_B in txt
    print(f'[patch_recovery_bt] 目标: {real}')
    hz_key = 'hz="%s"' % a.replan_hz
    print('  当前: 已注入=%s  (Wait 恢复=%d 处, 重规划目标 %s 已存在=%s)'
          % (injected, txt.count('<Wait wait_duration='), hz_key, hz_key in txt))
    if a.check or not (a.apply or a.restore):
        return 0

    if a.apply:
        if injected:
            print('  已注入，跳过')
            return 0
        new = txt
        # 1) Wait(2) → Spin（只替换 wait_duration="2" 那个，保留 Wait(1)）
        old_wait = '        <Wait wait_duration="2"/>\n'
        if old_wait not in new:
            print('  ✗ 找不到 `<Wait wait_duration="2"/>` 锚点；放弃')
            return 3
        new = new.replace(old_wait,
                          SENT_B
                          + f'        <Spin spin_dist="{a.spin_dist}" time_allowance="{a.spin_time}"/>\n'
                          + SENT_E, 1)
        # 2) 重规划节拍
        old_hz = '<RateController hz="0.3">'
        if old_hz in new:
            new = new.replace(old_hz, f'<RateController hz="{a.replan_hz}">', 1)
        else:
            print('  ! 没找到 RateController hz="0.3"，只改了恢复动作')
        # XML 语法校验
        try:
            import xml.etree.ElementTree as ET
            ET.fromstring(new)
        except Exception as exc:               # noqa: BLE001
            print(f'  ✗ 注入后 XML 非法，放弃：{exc}')
            return 3
        bak = os.path.join(CACHE, 'recovery_bt.pristine.xml')
        if not os.path.exists(bak):
            shutil.copy2(real, bak)
        json.dump({'file': real, 'spin_dist': a.spin_dist, 'spin_time': a.spin_time,
                   'replan_hz': a.replan_hz, 'old_hz': old_hz if old_hz in new else None},
                  open(journal(), 'w', encoding='utf-8'), ensure_ascii=False, indent=1)
        open(real, 'w', encoding='utf-8').write(new)
        print(f'  ✓ 已注入：Spin({a.spin_dist} rad / {a.spin_time}s) + 重规划 {a.replan_hz} Hz')
        return 0

    if not injected:
        print('  无注入可撤销')
        return 0
    try:
        j = json.load(open(journal(), encoding='utf-8'))
    except Exception:                          # noqa: BLE001
        j = {}
    new = txt
    i = new.index(SENT_B)
    k = new.index(SENT_E, i) + len(SENT_E)
    new = new[:i] + '        <Wait wait_duration="2"/>\n' + new[k:]
    old_hz = j.get('old_hz') or '<RateController hz="0.3">'
    if j.get('replan_hz'):
        new = new.replace(f'<RateController hz="{j["replan_hz"]}">', old_hz, 1)
    open(real, 'w', encoding='utf-8').write(new)
    try:
        os.remove(journal())
    except OSError:
        pass
    print('  ✓ 已撤销注入')
    return 0


if __name__ == '__main__':
    sys.exit(main())
