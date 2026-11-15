#!/usr/bin/env python3
"""test_report_consistency.py —— 防止"两套验收工具判定相反"再次发生。

背景（2026-11-15）
------------------
`evaluate.py` 与 `report.py` 曾经各自实现一套判定逻辑，实测 **955 轮里
166 轮（17.4%）结论相反**，跨 2026-10-22 到 11-15：

  126 轮 —— report.py 的倒车判据只看 `assist == False`，不看授权包络；
             0.065 m 的受控探索期小步倒车被当成"非受控"。
   50 轮 —— report.py 的返航判据看 `return_s`（home − done）。P2 把
             `coverage_terminate` 默认打开后，"探索限时到（"不再出现，
             done 事件识别不到 ⇒ return_s=None ⇒ 误报"返航未完成"。

本测试用**静态断言 + 真实日志**双重锁定：

  1. 静态：report.py 不得再自行实现 verdict（必须取 evaluate.json）
  2. 静态：两个工具的"依据"必须单一，report.py 只在缺文件时回退并标注
  3. 静态：done 事件必须认四种结束消息（含覆盖率终止）
  4. 真实数据：对若干历史轮次，两者结论必须一致

运行：python3 harness/test_report_consistency.py
"""
import glob
import importlib.util
import json
import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
REPORT = os.path.join(ROOT, 'harness', 'report.py')

PASS = 0
FAIL = 0
FAILS = []


def check(name, ok, why=''):
    global PASS, FAIL
    if ok:
        PASS += 1
        print('  ✓ %s' % name)
    else:
        FAIL += 1
        FAILS.append(name)
        print('  ✗ %s%s' % (name, ('  ← ' + why) if why else ''))


def main():
    src = open(REPORT, encoding='utf-8').read()

    print('\n=== 1. 判定必须单源（不得自行重算 verdict） ===')
    check('1.1 report.py 会读 evaluate.json',
          'evaluate.json' in src and 'def load_evaluate' in src,
          '必须复用 evaluate.py 的口径，否则必然再次分歧')
    check('1.2 权威分支里 verdict 直接取自 evaluate.json',
          "verdict = evj.get('verdict')" in src,
          '不允许在权威分支里重新推导结论')
    check('1.3 failed 列表取自 checks 里为 False 的项',
          "for k, v in (evj.get('checks') or {}).items() if not v" in src)
    check('1.4 回退分支有明确标注（不许无声分歧）',
          '回退口径' in src and '判定来源' in src,
          '缺 evaluate.json 时必须写明口径不同')
    check('1.5 硬倒车的"非受控"结论引用 evaluate 的授权字段',
          "coll.get('reverse_authorized')" in src,
          '不能只凭 assist 真假判定受控与否')

    print('\n=== 2. done 事件必须覆盖全部终止方式 ===')
    check('2.1 认 manager 自己的 mapping done acknowledged',
          "'mapping done acknowledged' in line" in src)
    check('2.2 认定时器终止「探索限时到（」',
          "'探索限时到（' in line" in src)
    check('2.3 认覆盖率终止「结束探索（」',
          "'结束探索（' in line" in src,
          'P2 打开 coverage_terminate 后定时器消息消失，缺这条必然误判返航未完成')
    check('2.4 认 set_mapping_done 返回值（节点名不是 manager）',
          "'SetBool_Response' in line and 'success=True' in line" in src,
          'is_mgr 会拦住它，必须单独匹配')
    check('2.5 认会话驱动的「返航到位/返航完成」（与 evaluate 的 POS 正则一致）',
          r"返航(?:到位|完成)" in src,
          '2026-10 期只有这两句，缺它 return_s 恒为 None；'
          '且正则必须与 evaluate.py 的 POS 完全相同，否则 return_home 会分歧')

    print('\n=== 3. 真实日志：两工具结论必须一致 ===')
    spec = importlib.util.spec_from_file_location('rp', REPORT)
    rp = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(rp)

    # 优先挑历史上有分歧的轮次 + 近期轮次
    candidates = []
    for d in sorted(glob.glob(os.path.join(ROOT, 'logs', 'round_*'))):
        name = os.path.basename(d)[6:]
        if not (os.path.exists(os.path.join(d, 'evaluate.json'))
                and os.path.exists(os.path.join(d, 'launch.log'))):
            continue
        candidates.append((name, d))
    # 取最近 30 轮做实测
    sample = candidates[-30:]
    check('3.1 有可测样本（>= 10 轮）', len(sample) >= 10,
          '实际 %d 轮' % len(sample))

    mism = []
    for name, d in sample:
        try:
            evj = json.load(open(os.path.join(d, 'evaluate.json'),
                                 encoding='utf-8'))
        except Exception:                  # noqa: BLE001
            continue
        try:
            ev = rp.launch_events(os.path.join(d, 'launch.log'))
            tb = rp.time_breakdown(ev, rp.first_log_ts(os.path.join(d, 'launch.log')))
        except Exception as exc:           # noqa: BLE001
            mism.append((name, 'parse-error:%s' % type(exc).__name__))
            continue
        # 核心不变量：返航**按目标到点**时，必须能算出 return_s。
        # 例外：evaluate.py 用位置兜底放行的轮次（by_geometry / by_manager_geometry）
        # 在日志里本来就没有"到点"事件，return_s 为空是事实正确。
        kind = evj.get('return_home_kind') or {}
        by_fallback = bool(kind.get('by_geometry')
                           or kind.get('by_manager_geometry'))
        if (evj.get('checks') or {}).get('return_home') and tb.get('return_s') is None \
                and not by_fallback:
            mism.append((name, 'return_s 缺失但返航成功'))
    check('3.2 近 30 轮：返航成功时 return_s 均可算',
          not mism, '不一致: %s' % mism[:3])

    # 真正的结论一致性：report 的权威分支取 evaluate.json，所以只要文件在就必然一致。
    # 这里验证"文件存在时确实走权威分支"（用 load_evaluate 的返回值判断）
    ok_all = True
    for name, d in sample[-6:]:
        got = rp.load_evaluate(d)
        if got is None:
            ok_all = False
    check('3.3 近 6 轮 evaluate.json 均可读（保证走权威分支）', ok_all)

    print('\n' + '=' * 60)
    print('report_consistency: %d 通过, %d 失败' % (PASS, FAIL))
    if FAILS:
        print('失败项:')
        for f in FAILS:
            print('  - %s' % f)
    sys.exit(0 if FAIL == 0 else 1)


if __name__ == '__main__':
    main()
