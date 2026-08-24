#!/usr/bin/env python3
"""Sharper anomaly metrics: kink = heading change > 80 deg within 1 m of arc; reverse = > 120 deg within 2 m of arc."""
import json, math, sys
from collections import Counter
def wrap(a):
    while a > math.pi: a -= 2*math.pi
    while a < -math.pi: a += 2*math.pi
    return a
def classify(wp):
    flags=set()
    n=len(wp)
    # waypoints are [x,y,psi,kappa,v]; spacing 0.5 m -> s = 0.5*i
    for i in range(1,n):
        for win,lab,th in ((2,"kink",math.radians(80)),(4,"reverse",math.radians(120))):
            j=max(0,i-win)
            if abs(wrap(wp[i][2]-wp[j][2]))>th: flags.add(lab)
    return flags
def main(path):
    frames=[json.loads(l) for l in open(path) if '"ev":"frame"' in l]
    n=len(frames); nvalid=sum(f["valid"] for f in frames)
    flagged=[(f,classify(f["wp"])) for f in frames if f["valid"]]
    flagged=[(f,fl) for f,fl in flagged if fl]
    moving=[(f,fl) for f,fl in flagged if f.get("v",0)>=0.5]
    print(f"{path}: frames={n} valid={nvalid} anomalous={len(flagged)} {dict(Counter(c for _,fl in flagged for c in fl))} | while moving(v>=0.5): {len(moving)} {dict(Counter(c for _,fl in moving for c in fl))}")
    eps=[]
    for f,fl in moving:
        if eps and f["t"]-eps[-1]["t_end"]<1.0: eps[-1]["t_end"]=f["t"]; eps[-1]["n"]+=1; eps[-1]["flags"]|=fl
        else: eps.append(dict(t0=f["t"],t_end=f["t"],n=1,flags=set(fl),x=f["x"],y=f["y"],live=f.get("live_added",0)))
    for e in eps[:25]: print(f"    t={e['t0']:7.1f}-{e['t_end']:7.1f} n={e['n']:4d} {sorted(e['flags'])} at=({e['x']:.1f},{e['y']:.1f}) live_added={e['live']}")
if __name__=="__main__":
    for p in sys.argv[1:]: main(p)
