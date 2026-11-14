#!/usr/bin/env python3
"""test_return_home_no_interrupt.py —— 返航前不许打断正在执行的导航（静态，<1 秒）。

用户要求（2026-11-14）
----------------------
"返航前必须要确认是否有导航任务，若有，则等待此轮导航结束、到点后才开始返航，
而不是中断正在进行的任务。"

为什么这条必须锁住
------------------
`p14vP_sw2` 实测：车在 0.33 m/s 巡航开往 `(6.82, -9.22)`（还差 0.99 m），
`set_mapping_done` 一到就 `cancel_goal_async()`，车在 **0.057 m** 内急停。
急停点 `(6.244, -10.022)` 既不是候选点也不是目标点，随后返航起点净空校验
（`return_home_start_clearance` 实际 0.40 m）不通过 → 3 次重试打满 →
`return_home_failed_waiting_manual` → 干耗到整轮超时。

而 `session_driver` 里 **本来就有**一条 2026-10-26 加的同类规则
（"正在执行的候选不许被打断 —— 等它到/取消/失败"），它却从未生效过：

    _goal_in_progress() 读 o.get('nav_goal_active')
    manager 的 _publish_state_impl() payload 里**从来没有这个字段**
    ⇒ 恒为 None ⇒ 恒为 False ⇒ 规则是死代码

这正是本仓库反复出现的失效类型：**语法正确、跑得起来、日志正常，就是没接上**。
所以把"有没有接上"写成静态断言，而不是靠下次读日志才发现。
"""
import ast
import os
import sys

MGR = ('/home/kbf/wheeltec_ros2/src/stemm_nav2_manager/'
       'stemm_nav2_manager/manager.py')
LAUNCH = ('/home/kbf/wheeltec_ros2/src/stemm_cartographer_exploration/launch/'
          'stemm_cartographer_auto_mapping.launch.py')
DRIVER = ('/home/kbf/stemm_exploration_sim_ws/src/stemm_cartographer_exploration_sim/'
          'scripts/session_driver')

_ok = []
_bad = []


def chk(cond, label, detail=''):
    (_ok if cond else _bad).append((label, detail))


def src(p):
    return open(p, encoding='utf-8').read()


def func_seg(path, name):
    s = src(path)
    tree = ast.parse(s)
    for n in ast.walk(tree):
        if isinstance(n, ast.FunctionDef) and n.name == name:
            return ast.get_source_segment(s, n) or ''
    return ''


def main():
    m = src(MGR)
    d = src(DRIVER)
    lc = src(LAUNCH)

    # ---- A. 参数存在且默认开启（用户要求是"必须"，默认就该是 True） ----
    chk("declare_parameter('return_home_wait_active_goal', True)" in m,
        'A return_home_wait_active_goal 声明且默认 True')
    chk("declare_parameter('return_home_wait_active_goal_sec'" in m,
        'A 等待有上限参数 return_home_wait_active_goal_sec')
    chk("'return_home_wait_active_goal': True" in lc,
        'A launch 显式写 True（可审计、可回退）',
        'launch 没写就只能靠节点默认值，A/B 时看不见')

    # ---- B. set_mapping_done 不再无条件取消活跃目标 ----
    seg = func_seg(MGR, 'set_mapping_done_callback')
    chk(seg != '', 'B 找到 set_mapping_done_callback')
    if seg:
        chk('return_home_wait_active_goal' in seg,
            'B 取消活跃目标前检查了 return_home_wait_active_goal')
        chk('mapping_done_waiting_active_goal' in seg,
            'B 等待态有明确 state 名（供 session_driver 识别）')
        # 顺序：先判断"要不要等"，再 cancel。若 cancel 出现在 if 之前就是没生效。
        i_wait = seg.find('return_home_wait_active_goal')
        i_cancel = seg.find('cancel_goal_async')
        chk(i_wait > 0 and i_cancel > i_wait,
            'B "等待"分支排在 cancel_goal_async 之前',
            '顺序反了等于没改')

    # ---- C. 到点后真的发起返航 ----
    seg = func_seg(MGR, 'goal_result_callback')
    chk(seg != '', 'C 找到 goal_result_callback')
    if seg:
        chk('mapping_done_active_goal_reached' in seg,
            'C SUCCEEDED 分支里有"到点后再返航"的出口')
        i = seg.find('mapping_done_active_goal_reached')
        chk(i > 0 and '_dispatch_pending_home_goal' in seg[i:i + 1200],
            'C 该出口真的调用了 _dispatch_pending_home_goal（不只是改状态）')
        chk('mapping_done_active_goal_failed' in seg,
            'C 目标失败时也转入返航（而不是走探索期的脱困策略）')

    # ---- D. 超时兜底：不能无限等 ----
    seg = func_seg(MGR, '_auto_explore_callback_impl')
    chk(seg != '', 'D 找到 _auto_explore_callback_impl')
    if seg:
        chk('return_home_wait_active_goal_sec' in seg,
            'D 周期回调里有超时判定（否则目标卡死就把整轮拖死）')
        chk('mapping_done_active_goal_timeout' in seg,
            'D 超时后回退到"取消 + 返航"的老行为')

    # ---- E. 根因：nav_goal_active 必须真的发布出去 ----
    seg = func_seg(MGR, '_publish_state_impl')
    chk(seg != '', 'E 找到 _publish_state_impl')
    if seg:
        chk("'nav_goal_active'" in seg,
            'E nav_state payload 发布 nav_goal_active',
            '缺这一项 ⇒ session_driver 的 _goal_in_progress 恒为 False')

    # ---- F. session_driver 侧：别再抢着 cancel-all ----
    seg = func_seg(DRIVER, '_goal_in_progress')
    chk(seg != '', 'F 找到 session_driver._goal_in_progress')
    if seg:
        chk('nav_goal_active' in seg,
            'F _goal_in_progress 仍以 nav_goal_active 为判据（与 E 对齐）')
    chk('mapping_done_waiting_active_goal' in d,
        'F session_driver 认识了 manager 的等待态')
    # 兜底重发那段必须排除等待态，否则会 cancel-all 掉正在执行的目标
    i = d.find('not _waiting_active_goal')
    chk(i > 0,
        'F 兜底重发/cancel-all 被 _waiting_active_goal 挡住')
    if i > 0:
        chk(d.find('_cancel_all_goals()') > i,
            'F cancel-all 排在该门禁之后',
            '顺序反了等于没挡')

    for label, detail in _bad:
        print('  ✗ %s  %s' % (label, detail))
    print('test_return_home_no_interrupt: %d 通过, %d 失败'
          % (len(_ok), len(_bad)))
    return 0 if not _bad else 1


if __name__ == '__main__':
    sys.exit(main())
