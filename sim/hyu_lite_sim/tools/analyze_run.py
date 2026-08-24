#!/usr/bin/env python3
"""analyze_run.py <run dir> [track.csv] -- analyse a lite_regression.sh run: corridor position of the GT car per lap,
stack CTE per lap, steering command stats, frozen-map completeness."""
import csv, math, os, sys, glob
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

OUT = sys.argv[1]
TRACK = sys.argv[2] if len(sys.argv) > 2 else '/home/race/fsk/src/sim/eufs_sim/eufs_tracks/csv/small_track.csv'
AXLE = 0.91

def read_bag(path):
    storage = rosbag2_py.StorageOptions(uri=path, storage_id='sqlite3')
    conv = rosbag2_py.ConverterOptions('', '')
    reader = rosbag2_py.SequentialReader(); reader.open(storage, conv)
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    msgs = {k: [] for k in types}
    while reader.has_next():
        topic, data, t = reader.read_next()
        msgs[topic].append((t * 1e-9, deserialize_message(data, get_message(types[topic]))))
    return msgs

def yaw_of(q):
    return math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))

bagdir = glob.glob(os.path.join(OUT, 'bag*'))[0]
msgs = read_bag(bagdir)
print(f"bag topics: { {k: len(v) for k, v in msgs.items()} }")

gt = [(t, m.pose.pose.position.x, m.pose.pose.position.y, yaw_of(m.pose.pose.orientation),
       m.twist.twist.linear.x) for t, m in msgs.get('/ground_truth/state', [])]
laps = [(t, m.data) for t, m in msgs.get('/planning/lap_count', [])]
states = [(t, m.data) for t, m in msgs.get('/planning/state', [])]
cte = [(t, m.data) for t, m in msgs.get('/planning/cte', [])]
cmd = [(t, m.drive.steering_angle, m.drive.speed) for t, m in msgs.get('/vehicle/cmd', [])]
src = [(t, m.data) for t, m in msgs.get('/planning/path_source', [])]
status = [(t, m.data) for t, m in msgs.get('/localization/status', [])]

# --- track geometry (csv frame == sim world frame) ---
rows = list(csv.DictReader(open(TRACK)))
cs = [r for r in rows if r['tag'] == 'car_start'][0]
sx, sy, syaw = float(cs['x']), float(cs['y']), float(cs['direction'])
# lite sim world origin = car_start pose (base_footprint at (0,0), heading 0)
def W(r):
    x, y = float(r['x']) - sx, float(r['y']) - sy
    c, s_ = math.cos(-syaw), math.sin(-syaw)
    return (c*x - s_*y, s_*x + c*y)
blue = [W(r) for r in rows if r['tag'] == 'blue']
yellow = [W(r) for r in rows if r['tag'] == 'yellow']
allcones = blue + yellow + [W(r) for r in rows if r['tag'] == 'big_orange']

def nearest(p, pts):
    return min(math.hypot(p[0]-q[0], p[1]-q[1]) for q in pts)

def chain(pts, start):
    rem = pts[:]; cur = min(rem, key=lambda q: math.hypot(q[0]-start[0], q[1]-start[1])); out=[cur]; rem.remove(cur)
    while rem:
        nxt = min(rem, key=lambda q: math.hypot(q[0]-cur[0], q[1]-cur[1])); out.append(nxt); rem.remove(nxt); cur = nxt
    return out

def poly_area(poly):
    return 0.5*sum(poly[i][0]*poly[(i+1)%len(poly)][1] - poly[(i+1)%len(poly)][0]*poly[i][1] for i in range(len(poly)))

def inside(p, poly):
    x, y = p; n = len(poly); c = False
    for i in range(n):
        x1, y1 = poly[i]; x2, y2 = poly[(i+1) % n]
        if (y1 > y) != (y2 > y):
            xi = x1 + (y - y1) * (x2 - x1) / (y2 - y1)
            if x < xi: c = not c
    return c

blue_poly = chain(blue, (0.0, 0.0)); yellow_poly = chain(yellow, (0.0, 0.0))
if abs(poly_area(blue_poly)) >= abs(poly_area(yellow_poly)):
    outer, inner = blue_poly, yellow_poly
else:
    outer, inner = yellow_poly, blue_poly
def in_track(p):
    return inside(p, outer) and not inside(p, inner)

def lap_of(t):
    lap = 0
    for (tt, l) in laps:
        if tt <= t: lap = l
    return lap

# lap boundaries from the lap counter
lap_t = {}
for tt, l in laps:
    lap_t.setdefault(l, tt)
# per-lap corridor stats on the GT rear axle AND the base point
per = {}
for (t, x, y, yaw, v) in gt:
    lap = lap_of(t)
    ax, ay = x + AXLE*math.cos(yaw), y + AXLE*math.sin(yaw)
    db, dy = nearest((ax, ay), blue), nearest((ax, ay), yellow)
    # normalised corridor position: 0 centre, +1 ON blue cone, -1 ON yellow cone
    n = (dy - db) / max(1e-6, dy + db)
    clear = min(db, dy)
    d = per.setdefault(lap, {'n': [], 'clear': [], 'v': [], 'out': [], 'out_base': [], 't0': t, 't1': t})
    d['n'].append(n); d['clear'].append(clear); d['v'].append(v); d['t1'] = t
    d['out'].append(0 if in_track((ax, ay)) else 1)
    d['out_base'].append(0 if in_track((x, y)) else 1)

print("\nGT rear axle vs track (outer/inner cone polygons; n: 0 centre, +-1 on a cone)")
print("lap  dur[s]  n_samp  mean|n|  p95|n|  max|n|  frac|n|>0.6  frac_out(axle)  frac_out(base)  min_clear[m]  p05_clear  mean_v")
for lap in sorted(per):
    d = per[lap]; ns = sorted(abs(a) for a in d['n']); cl = sorted(d['clear'])
    if not ns: continue
    p95 = ns[int(0.95*(len(ns)-1))]; p05c = cl[int(0.05*(len(cl)-1))]
    frac = sum(1 for a in ns if a > 0.6)/len(ns)
    fo = sum(d['out'])/len(d['out']); fob = sum(d['out_base'])/len(d['out_base'])
    print(f"{lap:3d}  {d['t1']-d['t0']:6.1f}  {len(ns):6d}  {sum(ns)/len(ns):6.3f}  {p95:6.3f}  {ns[-1]:6.3f}  {frac:12.3f}  {fo:14.3f}  {fob:14.3f}  {cl[0]:12.2f}  {p05c:9.2f}  {sum(d['v'])/len(d['v']):6.2f}")

# stack CTE (frenet d vs raceline) per lap
cte_per = {}
for t, d in cte:
    cte_per.setdefault(lap_of(t), []).append(abs(d))
print("\nstack /planning/cte per lap (|d| vs raceline; valid once GLOBAL):")
for lap in sorted(cte_per):
    v = cte_per[lap]; v2 = sorted(v)
    print(f"  lap {lap}: n={len(v)} rms={math.sqrt(sum(a*a for a in v)/len(v)):.3f} p95={v2[int(0.95*(len(v2)-1))]:.3f} max={v2[-1]:.3f}")

# steering command stats per lap
st_per = {}
for t, s, sp in cmd:
    st_per.setdefault(lap_of(t), []).append(abs(s))
print("\n/vehicle/cmd |steer| per lap: mean, frac>=0.50 (PP lock), frac>=0.335 (bridge 90deg clip):")
for lap in sorted(st_per):
    v = st_per[lap]
    print(f"  lap {lap}: n={len(v)} mean={sum(v)/len(v):.3f} sat52={sum(1 for a in v if a>=0.50)/len(v):.3f} over335={sum(1 for a in v if a>=0.335)/len(v):.3f}")

print("\nlap counter:", laps[:8])
print("path_source changes:", [(round(t,1), s) for i,(t,s) in enumerate(src) if i==0 or src[i-1][1]!=s][:10])
print("slam status changes:", [(round(t,1), s) for i,(t,s) in enumerate(status) if i==0 or status[i-1][1]!=s][:10])
print("state changes:", [(round(t,1), s) for i,(t,s) in enumerate(states) if i==0 or states[i-1][1]!=s][:12])

# --- frozen map completeness ---
mp_path = os.path.join(OUT, 'frozen_map.csv')
if os.path.exists(mp_path):
    mp = list(csv.DictReader(open(mp_path)))
    gtc = [r for r in rows if r['tag'] != 'car_start']
    def T(r): return (float(r['x'])-sx, float(r['y'])-sy)
    go = [T(r) for r in gtc if r['tag']=='big_orange']; mo=[(float(r['x']),float(r['y'])) for r in mp if r['tag']=='big_orange']
    if len(mo) == 4 and len(go) == 4:
        ox = sum(p[0] for p in mo)/4 - sum(p[0] for p in go)/4; oy = sum(p[1] for p in mo)/4 - sum(p[1] for p in go)/4
    else:
        ox, oy = -1.28, 0.0
    mpts = [((float(r['x']), float(r['y'])), r['tag']) for r in mp if r['tag'] != 'car_start']
    missing = []; uncol = []; ok = 0
    for r in gtc:
        p = (T(r)[0]+ox, T(r)[1]+oy)
        best = min(((math.hypot(p[0]-q[0], p[1]-q[1]), tg) for (q, tg) in mpts), key=lambda a: a[0])
        if best[0] > 0.8: missing.append((r['tag'], round(p[0],1), round(p[1],1), round(best[0],2)))
        elif best[1] == 'unknown' and r['tag'] != 'unknown': uncol.append((r['tag'], round(p[0],1), round(p[1],1)))
        else: ok += 1
    print(f"\nfrozen map ({os.path.basename(mp_path)}): {len(mp)-1} landmarks; GT cones {len(gtc)}: matched+coloured {ok}, unknown-colour {len(uncol)}, missing {len(missing)}")
    print("  missing:", missing)
    print("  unknown-colour:", uncol)
    gpts = [(T(r)[0]+ox, T(r)[1]+oy) for r in gtc]
    ghosts = [(tg, tuple(round(v,1) for v in q)) for (q, tg) in mpts if tg in ('blue','yellow','big_orange','orange') and min(math.hypot(q[0]-g[0], q[1]-g[1]) for g in gpts) > 0.8]
    print("  coloured landmarks with no GT cone within 0.8 m (ghosts):", ghosts)
