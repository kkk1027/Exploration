#!/usr/bin/env python3
"""test_driver_hooks.py —— 关键"接线"是否真的接上了（静态，<1 秒）。

今天连着出了两次同类事故：**代码写好了、但根本没接上**，而且都查不出来：

1. `region_complexity()` 从来没返回 `occ_frac_r2`，而 `gate_adjusted_penalty`
   用 `f.get('occ_frac_r2', 0.0)` 取值 → 软惩罚一直是死代码（默认 0，永不触发）。
2. `session_driver._on_nav_state` 的真实参数名是 `msg`，而补丁按 `m` 去替换，
   replace 静默没生效 → `self.nav` 永远是空 dict，"按车端四向测距挪一步"整条
   路径退化成"按地图选点"，还差点把车顶到地图上没画出来的低矮障碍上。

这类失效的共同点：**语法正确、跑得起来、日志也正常，就是没起作用**。
所以把"有没有接上"变成静态断言：查关键词是否同时出现在该出现的位置。

用法：python3 harness/test_driver_hooks.py
"""
import ast
import sys

DRIVER = ('/home/kbf/stemm_exploration_sim_ws/src/stemm_cartographer_exploration_sim/'
          'scripts/session_driver')
ARBITER = ('/home/kbf/stemm_exploration_sim_ws/src/stemm_cartographer_exploration_sim/'
           'scripts/frontier_arbiter')


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
    d = src(DRIVER)
    fails = []

    def check(name, cond, why=''):
        print(f'{"✓" if cond else "✗"} {name}' + ((f'  ← {why}') if (not cond and why) else ''))
        if not cond:
            fails.append(name)

    # 1) nav_state 必须真的被解析成 dict（今天在这里踩过坑）
    seg = func_seg(DRIVER, '_on_nav_state')
    check('nav_state 解析已接线', 'json.loads' in seg and 'self.nav = o' in seg,
          '缺 json.loads 或 self.nav 赋值 → 四向测距永远是空的')

    # 2) 返航兜底必须受"车没动"约束
    fallback = [l for l in d.splitlines()
                if '_moved_recently()' in l and 'not' in l]
    check('返航兜底受"车没动"约束', bool(fallback),
          '缺 not self._moved_recently() → 会掐掉正在正常执行的返航目标')

    # 3) 接管点必须用四向测距，且明确排除 rear（激光盲区）
    seg = func_seg(DRIVER, '_sensor_standoff')
    check('接管点走四向测距', 'left_min_range' in seg and 'right_min_range' in seg)
    check('接管点不选 rear（盲区）',
          'rear_min_range' not in seg and "'rear'" not in seg,
          'rear 在 nav_state 里是 99=无数据，选它等于盲开')

    # 4) 三向都窄时不能回落到"按地图选点"
    seg = func_seg(DRIVER, '_pick_standoff_if_tight')
    check('三向皆窄不回落地图选点',
          'return None' in seg and 'all(k in self.nav' in seg)

    # 5) 仲裁器的 occ_frac_r2 必须真的返回（否则软惩罚是死代码）
    seg = func_seg(ARBITER, 'region_complexity')
    check('region_complexity 返回 occ_frac_r2', "'occ_frac_r2'" in seg,
          'gate_adjusted_penalty 取不到 → 软惩罚永远 0')

    # 6) 后置 veto 绝不能 cancel-all（只能取消 EXECUTING 的那一个）
    seg = func_seg(ARBITER, '_do_veto_cancel')
    check('veto 只取消单个 goal_id', '_active_uuid[0]' in seg)

    if fails:
        print(f'  ✗ {len(fails)} 项接线缺失：{fails}')
        print('结果：FAIL')
        return 1
    print('结果：PASS（7 项接线检查全部通过）')
    return 0


if __name__ == '__main__':
    sys.exit(main())
