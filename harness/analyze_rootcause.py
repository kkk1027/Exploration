#!/usr/bin/env python3
"""analyze_rootcause.py —— **只读**根因统计：卡死到底卡在哪一方。

对每个"卡死事件"（来自 digest.md 的秒级表），在对应轮次的 launch.log 时间窗内检索
四类证据，判断责任方：
  车端安全闸 : front obstacle too close / safety_stop / blocked_waiting_clearance
  车端目标闸 : goal rejected.*unreachable / too close to occupied / goal_precheck_blocked
  nav2 规划器: failed to generate a valid path / exceeded maximum iterations / GridBased
  nav2 控制  : Failed to make progress / controller / MPPI / recovery
再统计：卡点聚集度（0.5 m 栅格去重）、类型分布、有活跃目标比例、front 分布。
用法：python3 harness/analyze_rootcause.py [--glob 'logs/round_*']
"""
import argparse, glob, json, math, os, re, statistics, sys
from collections import Counter, defaultdict

W='/home/kbf/stemm_exploration_sim_ws'
RE_EV=re.compile(r'\|\s*\d+\s*\|\s*(\d+)\s*\|\s*(\d+)\s*\|\s*(\d+)\s*\|\s*\(([-\d.]+),([-\d.]+)\)\s*\|\s*([^|]+)\|\s*([^|]+)\|')
PAT={
 'car_safety': re.compile(r'front obstacle too close|safety_stop'),
 'car_goal_gate': re.compile(r'goal rejected|too close to occupied|goal_precheck_blocked|unreachable'),
 'nav2_planner': re.compile(r'failed to generate a valid path|exceeded maximum iterations|GridBased'),
 'nav2_control': re.compile(r'Failed to make progress|controller_server|MPPI|recovery_started'),
}

def main():
    ap=argparse.ArgumentParser(); ap.add_argument('--glob', default='logs/round_*')
    a=ap.parse_args()
    dirs=sorted({os.path.dirname(p) for p in glob.glob(os.path.join(W,a.glob,'digest.md'))})
    ev_total=0; by_type=Counter(); by_side=Counter(); front_vals=[]
    spots=defaultdict(int); active_yes=0; rounds=0; ev_per_round=[]
    for d in dirs:
        lp=os.path.join(d,'launch.log')
        if not os.path.exists(lp): continue
        tl=os.path.join(d,'timeline.jsonl')
        if not os.path.exists(tl): continue
        rounds+=1
        txt=open(lp,errors='ignore').read()
        rows=[]
        for line in open(tl,errors='ignore'):
            line=line.strip()
            if line:
                try: rows.append(json.loads(line))
                except Exception: pass
        rows=[r for r in rows if r.get('x') is not None]; rows.sort(key=lambda r:r['t'])
        n=0
        for m in RE_EV.finditer(open(os.path.join(d,'digest.md'),errors='ignore').read()):
            t0,t1,dur=int(m.group(1)),int(m.group(2)),int(m.group(3))
            x,y=float(m.group(4)),float(m.group(5)); typ=m.group(6).strip(); act=m.group(7).strip()
            if x==0.0 and y==0.0: continue
            ev_total+=1; n+=1
            by_type['卡住(cmd≠0)' if '卡住' in typ else '空等(cmd=0)']+=1
            if '是' in act: active_yes+=1
            spots[(round(x*2)/2, round(y*2)/2)]+=1
            win=rows and [r for r in rows if t0<=r['t']<=t1+2] or []
            fr=[r['front'] for r in win if isinstance(r.get('front'),(int,float))]
            if fr: front_vals.append(statistics.median(fr))
            seg=txt[txt.find(str(t0)): ] if False else None
            # 时间窗内证据（用 epoch 前缀粗筛：从 t0 到 t1 的日志行）
            hit=Counter()
            for line in txt.splitlines():
                mm=re.search(r'\[(\d{10}\.\d+)\]', line)
                if not mm: continue
                tt=float(mm.group(1))
                if not (t0-2 <= tt <= t1+2): continue
                for k,p in PAT.items():
                    if p.search(line): hit[k]+=1
            if not hit:
                by_side['无日志证据（窗口内）']+=1
            else:
                for k in hit: by_side[k]+=1
        ev_per_round.append(n)
    print(f'[样本] {rounds} 轮（有 timeline+launch.log）、{ev_total} 个卡死事件（已剔除 (0,0) 启动噪声）')
    print('[类型]', dict(by_type))
    print('[有活跃目标] %d/%d = %.0f%%' % (active_yes, ev_total, 100.0*active_yes/max(1,ev_total)))
    print('[窗口内责任方证据]（一个事件可命中多类）', dict(by_side))
    if front_vals:
        fv=sorted(front_vals)
        print('[front 中位数分布 m] p10=%.2f p50=%.2f p90=%.2f  <0.6m 占比=%.0f%%'
              % (fv[len(fv)//10], fv[len(fv)//2], fv[9*len(fv)//10],
                 100.0*sum(1 for v in fv if v<0.6)/len(fv)))
    top=sorted(spots.items(), key=lambda kv:-kv[1])[:8]
    print('[卡点聚集度] 不同 0.5 m 格 %d 个 / %d 事件；Top8: %s'
          % (len(spots), ev_total, ['(%.1f,%.1f)×%d'%(k[0],k[1],v) for k,v in top]))
    print('[每轮事件数] 均值 %.2f 中位 %s 最大 %s'
          % (statistics.mean(ev_per_round), statistics.median(ev_per_round), max(ev_per_round)))
    return 0
if __name__=='__main__': sys.exit(main())
