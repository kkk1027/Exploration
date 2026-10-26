#!/usr/bin/env python3
"""test_harness_setu.py —— harness 脚本的 `set -u` × `source setup.bash` 地雷。

事故（2026-10-26）：`harness/batches/run_p0_shots.sh` 首行 `set -u`，内部用
  ( source /opt/ros/humble/setup.bash >/dev/null 2>&1 ... )
挂残渣探针。ROS 的 setup.bash 会展开未绑定变量（AMENT_TRACE_SETUP_FILES 等），
在 `set -u` 下报"未绑定的变量"并让**整个子 shell 静默退出**——stderr 又被重定向掉，
所以 p1/p2/p3/base 若干轮的 `costmap/residue.jsonl` 一根采样都没有，却没有任何报错。
本测试静态扫描 harness/**/*.sh：凡出现 `set -u`（或 -euo）的脚本，其 source ROS
setup 的位置前面必须显式 `set +u`。
"""
import glob
import os
import re
import sys

ROOT = '/home/kbf/stemm_exploration_sim_ws'
ROS_SETUP = re.compile(r'source\s+\S*(?:ros|setup\.bash)')
SETU = re.compile(r'^\s*set\s+-[a-z]*u', re.M)


def main():
    bad = []
    scanned = 0
    for path in sorted(glob.glob(os.path.join(ROOT, 'harness', '**', '*.sh'), recursive=True)):
        txt = open(path, encoding='utf-8', errors='replace').read()
        scanned += 1
        if not SETU.search(txt):
            continue
        lines = txt.splitlines()
        for i, line in enumerate(lines):
            if not ROS_SETUP.search(line) or line.lstrip().startswith('#'):
                continue
            # 往前 4 行内是否有 set +u
            window = '\n'.join(lines[max(0, i - 4):i + 1])
            if 'set +u' not in window:
                bad.append(f'{os.path.relpath(path, ROOT)}:{i + 1}  source ROS setup 但没 set +u')
    for b in bad:
        print('  ✗', b)
    print(f'[test_harness_setu] 扫描 {scanned} 个脚本，问题 {len(bad)} 处')
    return 1 if bad else 0


if __name__ == '__main__':
    sys.exit(main())
