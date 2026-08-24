import math
# 실제 마운트/센서
H_LIDAR=0.5402; CONE_H=0.45; CONE_W=0.23   # m
BEAMS=[-15+2*i for i in range(16)]          # RS16: -15..+15, 2deg
HOR_RES_DEG=0.2                             # 10Hz(600rpm) azimuth res
MIN_PTS_MAIN=3; MIN_PTS_SPARSE=2
def beams_on_cone(R):
    eb=math.degrees(math.atan2(-H_LIDAR,R))          # base(지면) elevation
    et=math.degrees(math.atan2(CONE_H-H_LIDAR,R))    # top elevation
    lo,hi=min(eb,et),max(eb,et)
    return [b for b in BEAMS if lo<=b<=hi]
def pts_per_beam(R):
    ang=math.degrees(2*math.atan2(CONE_W/2,R))       # 콘 방위폭(deg)
    return ang/HOR_RES_DEG
print(f"라이다 h={H_LIDAR} m, 콘 H={CONE_H}/W={CONE_W} m, RS16 2deg/{HOR_RES_DEG}deg, min_pts main{MIN_PTS_MAIN}/sparse{MIN_PTS_SPARSE}")
print(f"{'R(m)':>5} {'beams':>6} {'pts/beam':>9} {'total_pts':>10} {'검출':>18}")
for R in [1.5,2,3,4,5,6,7,8,9,10,11,12,14,16,18,20,22]:
    nb=len(beams_on_cone(R)); ppb=pts_per_beam(R); tot=nb*ppb
    if nb>=2 and tot>=MIN_PTS_MAIN*2: tag="견고(2+빔)"
    elif nb>=1 and tot>=MIN_PTS_MAIN: tag="가능(main)"
    elif nb>=1 and tot>=MIN_PTS_SPARSE: tag="sparse만"
    else: tag="불가"
    print(f"{R:>5.1f} {nb:>6} {ppb:>9.1f} {tot:>10.1f} {tag:>18}")
