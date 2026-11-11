#!/usr/bin/env python3
"""s4_check.py —— 验证 S4 的三个前置条件是否真的就位。

S4 的目的是：后置否决从「取消目标 + 重派」改成「原地改向 + 不取消」。
它需要三个条件同时成立，缺一个就会静默退回旧行为。本脚本逐条实测：

  ① nav2_behavior_tree 里编进了 2026-11-11 的修复
     （goal_updater_node.cpp 里换目标时清空 last_goal_received_）
  ② BT XML 里挂了 GoalUpdater，且 output_goal 写回 {goal}
     （写回 {goal} 才能让 GlobalUpdatedGoal 察觉并重规划）
  ③ /goal_update 话题**真的有订阅者** —— 这是唯一能证明 BT 真的加载了
     GoalUpdater 的运行期证据。没有订阅者时仲裁器会自动退回 migrate。

用法：
  python3 harness/s4_check.py                    # 只看静态条件
  python3 harness/s4_check.py --live             # 额外检查运行中的 /goal_update 订阅者
"""
import argparse
import os
import re
import subprocess
import sys
import xml.etree.ElementTree as ET


ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
# ★ S4 实际生效的是 manager 的 exploration_behavior_tree 参数指定的树，
#   不是 bt_navigator 的 default_nav_to_pose_bt_xml（后者会被前者盖掉）。
BT_XML = ('/home/kbf/wheeltec_ros2/src/stemm_cartographer_exploration/config/'
          'stemm_cartographer_nav_to_pose_goalupdater.xml')
BT_CAR = ('/home/kbf/wheeltec_ros2/src/stemm_cartographer_exploration/config/'
          'stemm_cartographer_nav_to_pose.xml')
SRC_CPP = ('/home/kbf/wheeltec_ros2/src/navigation2-humble/nav2_behavior_tree/'
           'plugins/decorator/goal_updater_node.cpp')
OVL_SO = ('/home/kbf/wheeltec_ros2/install/nav2_behavior_tree/lib/'
          'libnav2_goal_updater_node_bt_node.so')
LAUNCH = ('/home/kbf/wheeltec_ros2/src/stemm_cartographer_exploration/launch/'
          'stemm_cartographer_auto_mapping.launch.py')

fails = []


def check(name, ok, detail=''):
    print(('  PASS  ' if ok else '  FAIL  ') + name + (('  ' + detail) if detail else ''))
    if not ok:
        fails.append(name)


def cond1():
    print('① 编译产物含修复补丁')
    if not os.path.exists(SRC_CPP):
        check('源文件存在', False, SRC_CPP)
        return
    src = open(SRC_CPP, encoding='utf-8', errors='ignore').read()
    check('源码有 goal_cache_valid_ 缓存标记', 'goal_cache_valid_' in src)
    check('源码在换目标时清空缓存',
          'last_goal_received_ = geometry_msgs::msg::PoseStamped{}' in src)
    check('prev 记录的是输出而非输入',
          'prev 记录的是**本拍最终输出**' in src)
    check('插件库已编译', os.path.exists(OVL_SO), OVL_SO)
    if os.path.exists(OVL_SO):
        so_mtime = os.path.getmtime(OVL_SO)
        cpp_mtime = os.path.getmtime(SRC_CPP)
        check('插件库比源码新（改动已编入）', so_mtime >= cpp_mtime,
              'so=%.0f cpp=%.0f' % (so_mtime, cpp_mtime))


def cond2():
    print('② BT XML 挂了 GoalUpdater 且写回 {goal}')
    if not os.path.exists(BT_XML):
        check('BT 文件存在', False, BT_XML)
        return
    try:
        root = ET.parse(BT_XML).getroot()
    except ET.ParseError as exc:
        check('BT XML 可解析', False, str(exc))
        return
    check('BT XML 可解析', True)
    gus = [e for e in root.iter() if e.tag == 'GoalUpdater']
    check('存在 GoalUpdater 节点', len(gus) == 1, 'count=%d' % len(gus))
    if gus:
        a = gus[0].attrib
        check('input_goal 读 {goal}', a.get('input_goal') == '{goal}',
              str(a.get('input_goal')))
        # 关键：必须写回 {goal}，否则 GlobalUpdatedGoal 看不到变化
        check('output_goal 写回 {goal}（GlobalUpdatedGoal 才可见）',
              a.get('output_goal') == '{goal}', str(a.get('output_goal')))
    # 与车端 BT 对比：除 GoalUpdater 外不应有差异
    if os.path.exists(BT_CAR):
        def tags(p):
            d = {}
            for e in ET.parse(p).getroot().iter():
                d[e.tag] = d.get(e.tag, 0) + 1
            return d
        ta, tb = tags(BT_CAR), tags(BT_XML)
        extra = {k: tb[k] - ta.get(k, 0) for k in tb if tb[k] != ta.get(k, 0)}
        check('相对车端 BT 只多 GoalUpdater',
              list(extra.keys()) == ['GoalUpdater'] and extra['GoalUpdater'] == 1,
              str(extra))


def cond2b():
    print('②b launch 已把探索树做成可覆盖参数')
    if not os.path.exists(LAUNCH):
        check('launch 文件存在', False, LAUNCH)
        return
    txt = open(LAUNCH, encoding='utf-8', errors='ignore').read()
    check('manager 参数用 LaunchConfiguration 而非硬编码',
          "LaunchConfiguration(\n                'exploration_behavior_tree'" in txt
          or "'exploration_behavior_tree'," in txt)
    check('已声明同名 launch 参数',
          "DeclareLaunchArgument(\n            'exploration_behavior_tree'" in txt)
    # 关键：默认值必须是真实路径，不能是空串
    m = re.search(r"'exploration_behavior_tree',\s*\n\s*default_value=([^,]+),", txt)
    check('默认值不是空串（空串会让 nav2 加载失败）',
          bool(m) and "''" not in m.group(1) and '""' not in m.group(1),
          m.group(1).strip() if m else '(未找到)')


def cond3(live):
    print('③ 运行期证据')
    if not live:
        print('        （--live 可在仿真运行中检查 /goal_update 订阅者）')
        return
    try:
        out = subprocess.run(
            ['bash', '-lc',
             'source /opt/ros/humble/setup.bash >/dev/null 2>&1; '
             'source /home/kbf/wheeltec_ros2/install/setup.bash >/dev/null 2>&1; '
             'ros2 topic info /goal_update 2>/dev/null'],
            capture_output=True, text=True, timeout=25)
        m = re.search(r'Subscription count:\s*(\d+)', out.stdout)
        n = int(m.group(1)) if m else -1
        check('/goal_update 有订阅者（BT 真的加载了 GoalUpdater）', n >= 1,
              'count=%d' % n)
    except Exception as exc:                        # noqa: BLE001
        check('/goal_update 查询成功', False, str(exc))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--live', action='store_true')
    a = ap.parse_args()
    cond1()
    cond2()
    cond2b()
    cond3(a.live)
    print()
    if fails:
        print('FAILED %d 项：%s' % (len(fails), fails))
        return 1
    print('ALL PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
