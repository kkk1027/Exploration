#!/usr/bin/env python3
"""time_breakdown.py —— 从 launch.log 提取探索时间占用分布。
分段口径：启动(launch→Nav2就绪) / 选点等待(目标终态→下次派发) / 导航中(派发→终态)
          / 返航(set_mapping_done→返航完成) / 其余=解析缺口
"""
import re, sys

def ts_of(line):
    m = re.search(r'\[(\d{10}\.\d+)\]', line)
    return float(m.group(1)) if m else None

def analyze(path, label):
    ev = []
    for line in open(path, encoding='utf-8', errors='ignore'):
        t = ts_of(line)
        if t is None: continue
        if 'stably ready' in line: ev.append((t, 'ready'))
        elif 'navigation goal dispatched' in line: ev.append((t, 'dispatch'))
        elif 'Goal succeeded' in line: ev.append((t, 'succ'))
        elif 'Goal canceled' in line: ev.append((t, 'cancel'))
        elif 'Goal failed' in line: ev.append((t, 'fail'))
        elif 'mapping done acknowledged' in line: ev.append((t, 'done'))
        elif '返航完成' in line or '返航到位' in line: ev.append((t, 'home'))
        elif 'escape trace' in line: ev.append((t, 'escape'))
        elif 'blocked' in line and 'manager' in line: ev.append((t, 'blocked'))
        elif 'retreat' in line and 'manager' in line: ev.append((t, 'retreat'))
    if not ev: print(f'{label}: 无事件'); return
    t0 = ev[0][0]
    ready = next((t for t, k in ev if k == 'ready'), None)
    disp = [t for t, k in ev if k == 'dispatch']
    term = [(t, k) for t, k in ev if k in ('succ', 'cancel', 'fail')]
    done = next((t for t, k in ev if k == 'done'), None)
    home = next((t for t, k in ev if k == 'home'), None)
    nav = 0.0; navw = 0; rec = 0.0
    for dt in disp:
        later = [t for t, k in term if t >= dt]
        if later:
            end = min(later)
            nav += end - dt; navw += 1
            # 恢复时间（启发式）：该导航段内首次 blocked → 末次 escape trace 的跨度
            bl = [t for t, k in ev if k == 'blocked' and dt <= t <= end]
            es = [t for t, k in ev if k == 'escape' and dt <= t <= end]
            if bl and es:
                rec += max(es) - min(bl)
    exp_start = disp[0] if disp else ready
    exp_end = done or (term[-1][0] if term else None)
    span = (exp_end - exp_start) if (exp_start and exp_end) else 0
    gaps = max(0.0, span - nav)
    startup = (ready - t0) if ready else 0
    ret = (home - done) if (done and home) else None
    esc = sum(1 for _, k in ev if k == 'escape')
    blk = sum(1 for _, k in ev if k == 'blocked')
    rtr = sum(1 for _, k in ev if k == 'retreat')
    print(f'\n=== {label} ===')
    print(f'  目标派发 {len(disp)} 次；成功 {sum(1 for _,k in term if k=="succ")}、'
          f'取消 {sum(1 for _,k in term if k=="cancel")}、失败 {sum(1 for _,k in term if k=="fail")}')
    print(f'  启动(launch→Nav2就绪)        {startup:7.1f} s')
    print(f'  探索总时长(首目标→结束建图)  {span:7.1f} s')
    print(f'    其中 导航中(派发→终态)     {nav:7.1f} s  ({nav/span*100 if span else 0:.1f}%)')
    print(f'    其中 选点等待(终态→下次派发) {gaps:7.1f} s  ({gaps/span*100 if span else 0:.1f}%)')
    print(f'    其中 恢复动作(blocked→escape) {rec:7.1f} s  ({rec/span*100 if span else 0:.1f}%)  [占导航段内]')
    print(f'  返航(结束建图→返航完成)      {ret if ret is None else round(ret,1)} s')
    print(f'  事件：escape {esc} 次 / blocked {blk} 次 / retreat {rtr} 次')

for p, lab in [(sys.argv[1], sys.argv[2])] if len(sys.argv) > 2 else []:
    analyze(p, lab)
