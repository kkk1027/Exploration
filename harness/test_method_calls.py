#!/usr/bin/env python3
"""test_method_calls.py —— 静态回归守卫：节点脚本里的"自调用"签名与内部属性名。

为什么需要它（真实事故，2026-10-26 17:21 提交 aeef06c 引入、34+ 轮后才被发现）：
`escape_supervisor.maybe_reverse_assist` 里写 `self.explore_wedged(now)`，而定义是
`def explore_wedged(self)`；同函数里还读 `self._stuck_since`（真名 `stuck_since`）。
两个错误都在**探索期第一次 tick** 触发 → 节点当场 Traceback 退出 → 整轮没有脱困监督，
而 launch 是 `required=False` 风格：不会失败退出，只是少了一个节点，**日志里只有一行
`[ERROR] process has died`**。这类"少一个节点"的故障会把所有 A/B 结论污染掉。

本脚本用 AST 检查两件事（都只针对同文件内的 self.* 自调用，误报极低）：
  A. 位置参数个数与定义签名是否匹配；
  B. 读取 `self._xxx` 但文件里从未 `self._xxx = ...` 赋值（内部属性名拼错）。

用法：
  python3 harness/test_method_calls.py                     # 默认检查两个大脚本
  python3 harness/test_method_calls.py <file> [<file>...]
"""
import ast
import os
import sys

ROOT = '/home/kbf/stemm_exploration_sim_ws'
SCRIPTS = os.path.join(ROOT, 'src/stemm_cartographer_exploration_sim/scripts')
DEFAULT = [os.path.join(SCRIPTS, 'escape_supervisor'),
           os.path.join(SCRIPTS, 'frontier_arbiter')]


def method_sigs(cls):
    """name -> (min_pos, max_pos) ; *args 时 max=None"""
    out = {}
    for fn in cls.body:
        if not isinstance(fn, (ast.FunctionDef, ast.AsyncFunctionDef)):
            continue
        a = fn.args
        pos = [x.arg for x in list(a.posonlyargs) + list(a.args)]
        if pos and pos[0] in ('self', 'cls'):
            pos = pos[1:]
        ndef = len(a.defaults)
        min_pos = len(pos) - ndef
        max_pos = None if (a.vararg is not None) else len(pos)
        out[fn.name] = (min_pos, max_pos, fn.lineno, pos)
    return out


def assigned_attrs(cls):
    got = set()
    for node in ast.walk(cls):
        if isinstance(node, (ast.Assign, ast.AnnAssign, ast.AugAssign)):
            tgts = node.targets if isinstance(node, ast.Assign) else [node.target]
            for t in tgts:
                for sub in ast.walk(t):
                    if isinstance(sub, ast.Attribute) and isinstance(sub.value, ast.Name) \
                            and sub.value.id == 'self':
                        got.add(sub.attr)
    return got


def check(path):
    src = open(path, encoding='utf-8').read()
    tree = ast.parse(src, filename=path)
    bad = []
    for cls in [n for n in ast.walk(tree) if isinstance(n, ast.ClassDef)]:
        sigs = method_sigs(cls)
        attrs = assigned_attrs(cls)
        known = set(attrs) | set(sigs)
        for node in ast.walk(cls):
            # A. self.method(...) 位置参数个数
            if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute) \
                    and isinstance(node.func.value, ast.Name) and node.func.value.id == 'self':
                name = node.func.attr
                if name in sigs:
                    minp, maxp, defline, params = sigs[name]
                    n = len(node.args)
                    star = False
                    for k in node.keywords:          # 关键字实参也算"传了这个参数"
                        if k.arg is None:
                            star = True
                        elif k.arg in params:
                            n += 1
                    if star:
                        continue
                    if n < minp or (maxp is not None and n > maxp):
                        bad.append((node.lineno, 'argcount',
                                    'self.%s() 传了 %d 个位置参数，定义在 %s 行需要 %d%s 个'
                                    % (name, n, defline, minp,
                                       '~%d' % maxp if maxp is not None else '+')))
            # B. 读 self._xxx 但从未赋值
            if isinstance(node, ast.Attribute) and isinstance(node.value, ast.Name) \
                    and node.value.id == 'self':
                nm = node.attr
                if nm.startswith('_') and not nm.startswith('__') and nm not in known:
                    if isinstance(node.ctx, ast.Load):
                        bad.append((node.lineno, 'undefined_attr',
                                    'self.%s 从未在文件里赋值（疑似拼写错误）' % nm))
    # 去重（同一行同一问题只报一次）
    seen = set()
    uniq = []
    for ln, kind, msg in bad:
        if (ln, kind, msg) in seen:
            continue
        seen.add((ln, kind, msg))
        uniq.append((ln, kind, msg))
    return uniq


def main():
    files = sys.argv[1:] or DEFAULT
    total = 0
    for f in files:
        if not os.path.exists(f):
            print('  ✗ 文件不存在：%s' % f)
            total += 1
            continue
        bad = check(f)
        if bad:
            total += len(bad)
            print('✗ %s' % os.path.basename(f))
            for ln, kind, msg in sorted(bad):
                print('   L%-5d [%s] %s' % (ln, kind, msg))
        else:
            print('✓ %s' % os.path.basename(f))
    if total:
        print('\n共 %d 处问题（这类问题会让节点当场崩溃、整轮少一个节点，务必先修）' % total)
        return 1
    print('\n自调用签名与内部属性名检查通过')
    return 0


if __name__ == '__main__':
    sys.exit(main())
