#!/usr/bin/env python3
"""analyze_narrow_family.py —— **只读**：收敛分析「真卡死少数派」（卡点自由宽度 < 车宽）与接触事件的关系。

对全部轮次：
 1) 从 digest.md 取"卡住"事件点，用轮末地图的距离场算自由宽度，筛出 < 车宽(0.47 m) 的点；
 2) 从 probe.jsonl 取接触/擦碰事件（contact/graze）的时刻与位置，算其所在处自由宽度；
 3) 统计：接触事件有多少落在这类"窄点"附近（<=1.0 m），以及窄点的重复度/时长/状态。
用法：python3 harness/analyze_narrow_family.py
"""
import glob, json, math, os, re, statistics, sys
from collections import Counter, deque
import numpy as np

W='/home/kbf/stemm_exploration_sim_ws'; CAR_W=0.47
RE_EV=re.compile(r'\|\s*\d+\s*\|\s*(\d+)\s*\|\s*(\d+)\s*\|\s*(\d+)\s*\|\s*\(([-\d.]+),([-\d.]+)\)\s*\|\s*([^|]+)\|\s*([^|]+)\|')

def load_pgm(p):
    d=open(p,'rb').read(); i=0; t=[]
    while len(t)<4:
        while d[i:i+1].isspace(): i+=1
        j=i
        while not d[j:j+1].isspace(): j+=1
        t.append(d[i:j]); i=j
    i+=1; w,h=int(t[1]),int(t[2])
    return np.frombuffer(d[i:i+w*h],dtype=np.uint8).reshape(h,w),w,h

def field(occ,res,cap=1.5):
    h,w=occ.shape; INF=1<<20
    d=np.full((h,w),INF,dtype=np.int32); q=deque()
    ys,xs=np.nonzero(occ)
    for j,i in zip(ys.tolist(),xs.tolist()):
        d[j,i]=0; q.append((j,i))
    cc=int(cap/res)
    while q:
        j,i=q.popleft(); b=d[j,i]+1
        if b>cc: continue
        for dj,di in ((1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)):
            nj,ni=j+dj,i+di
            if 0<=nj<h and 0<=ni<w and d[nj,ni]>b:
                d[nj,ni]=b; q.append((nj,ni))
    return d.astype(np.float32)*res

def main():
    narrow=[]; contacts=[]; rounds=0
    for d in sorted(glob.glob(os.path.join(W,'logs','round_*'))):
        mp=os.path.join(d,'map.pgm'); yp=os.path.join(d,'map.yaml'); dg=os.path.join(d,'digest.md')
        if not (os.path.exists(mp) and os.path.exists(yp) and os.path.exists(dg)): continue
        import yaml
        y=yaml.safe_load(open(yp)); res=y['resolution']; ox,oy=y['origin'][0],y['origin'][1]
        img,w,h=load_pgm(mp); fd=field(img<100,res)
        rounds+=1
        def width(x,yy):
            i=int((x-ox)/res); j=h-1-int((yy-oy)/res)
            if not (0<=i<w and 0<=j<h): return None
            return 2.0*float(fd[j,i])
        tag=os.path.basename(d).split('_',2)[-1]
        for m in RE_EV.finditer(open(dg,errors='ignore').read()):
            t0,t1,dur=int(m.group(1)),int(m.group(2)),int(m.group(3))
            x,yy=float(m.group(4)),float(m.group(5)); typ=m.group(6).strip()
            if '卡住' not in typ or (x==0.0 and yy==0.0): continue
            wd=width(x,yy)
            if wd is not None and wd<CAR_W: narrow.append((tag,t0,dur,x,yy,round(wd,2)))
        pr=os.path.join(d,'probe.jsonl')
        if os.path.exists(pr):
            for line in open(pr,errors='ignore'):
                try: r=json.loads(line)
                except Exception: continue
                if r.get('type') not in ('contact','graze'): continue
                t=r.get('t')
                pos=r.get('pos') or r.get('pos_xy') or None
                if not pos or len(pos)<2: continue
                wd=width(float(pos[0]),float(pos[1]))
                contacts.append((tag,float(t) if t else 0.0,float(pos[0]),float(pos[1]),
                                 None if wd is None else round(wd,2)))
    print('[样本] %d 轮；窄点(<%.2fm) %d 个；接触/擦碰事件 %d 个'
          % (rounds,CAR_W,len(narrow),len(contacts)))
    durs=[n[2] for n in narrow]
    if durs: print('[窄点时长] 均值 %.1fs 中位 %s 最大 %s；>=30s %d/%d'
                   % (statistics.mean(durs),statistics.median(durs),max(durs),
                      sum(1 for v in durs if v>=30),len(durs)))
    spots=Counter((round(n[3]*2)/2, round(n[4]*2)/2) for n in narrow)
    print('[窄点聚集] 不同 0.5m 格 %d；Top6 %s' % (len(spots),
          ['(%.1f,%.1f)×%d'%(k[0],k[1],v) for k,v in spots.most_common(6)]))
    by_round=Counter(n[0] for n in narrow)
    print('[窄点轮次] 出现在 %d 个轮次；每轮最多 %d 个' % (len(by_round), max(by_round.values())))
    if contacts:
        near=0
        for (tg,t,x,yy,wd) in contacts:
            for n in narrow:
                if n[0]==tg and abs(n[1]-t)<=15 and math.hypot(n[3]-x,n[4]-yy)<=1.0:
                    near+=1; break
        cw=[c[4] for c in contacts if c[4] is not None]
        print('[接触点宽度] 有值 %d 个；<车宽 %d 个；中位 %s m'
              % (len(cw), sum(1 for v in cw if v<CAR_W),
                 round(statistics.median(cw),2) if cw else None))
        print('[接触↔窄点] 落在窄点 1.0 m/±15 s 内的接触事件：%d/%d = %.0f%%'
              % (near,len(contacts),100.0*near/max(1,len(contacts))))
        cr=Counter(c[0] for c in contacts)
        print('[接触分布] 涉及 %d 个轮次；Top5 %s' % (len(cr), cr.most_common(5)))
    return 0
if __name__=='__main__': sys.exit(main())
