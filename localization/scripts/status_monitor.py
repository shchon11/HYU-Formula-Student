#!/usr/bin/env python3
"""Live SBG Ellipse-D raw-GNSS dashboard.

One-glance terminal panel of the receiver itself, built from the raw GNSS
outputs only -- the device EKF (ekf_nav / ekf_euler / ekf_rot_accel) is NOT
used here:

  * /sbg/gps_pos  fix type + solution status, lat/lon/alt (MSL + undulation),
                  1-sigma N/E/D, sats used/tracked, interference (IFM) /
                  spoofing / OSNMA flags, constellations+signals in the
                  solution, RTK base id + differential age
  * /sbg/gps_hdt  dual-antenna heading/pitch with 1-sigma, measured baseline
                  against the configured lever-arm baseline, sats
  * /sbg/gps_vel  Doppler speed / course
  * /sbg/imu_data raw |accel|, body rates, temperature
  * /ntrip_client/rtcm, /sbg/status  correction flow + aiding flags

    ros2 run hyu_localization status_monitor      # after build
    python3 status_monitor.py                      # standalone
Refreshes ~2 Hz. Ctrl+C to quit.
"""
import math, time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sbg_driver.msg import SbgGpsPos, SbgGpsHdt, SbgGpsVel, SbgImuData, SbgStatus
try:
    from rtcm_msgs.msg import Message as RtcmMessage
    HAVE_RTCM = True
except Exception:
    HAVE_RTCM = False

R="\033[0m"; B="\033[1m"; D="\033[2m"
GRN="\033[32m"; YEL="\033[33m"; RED="\033[31m"; CYN="\033[36m"; GRY="\033[90m"; WHT="\033[97m"
def c(x,s): return f"{x}{s}{R}"
def grade(v,good,warn,fmt,invert=False):
    """color a number by thresholds (invert: smaller=better default)."""
    col=GRN if v<=good else (YEL if v<=warn else RED)
    if invert: col=GRN if v>=good else (YEL if v>=warn else RED)
    return c(col,fmt.format(v))
def yn(b): return c(GRN,"YES") if b else c(RED,"no")
def deg(r): return r*57.29578
def sv(n): return "--" if n==0xFF else str(n)   # num_sv_* : 0xFF = N/A

# ---- SbgGpsPosStatus / SbgGpsHdt enums (see sbg_ros2_driver/msg) ----
FIX={0:"NO_SOL",1:"UNKNOWN",2:"SINGLE",3:"PSRDIFF",4:"SBAS",5:"OMNISTAR",6:"RTK_FLOAT",7:"RTK_FIXED",8:"PPP_FLOAT",9:"PPP_FIXED",10:"FIXED"}
SOL={0:"SOL_COMPUTED",1:"INSUFFICIENT_OBS",2:"INTERNAL_ERROR",3:"HEIGHT_LIMIT"}
# interference monitoring: (label, colour)
IFM={0:("ERROR",RED),1:("UNKNOWN",GRY),2:("CLEAN",GRN),3:("MITIGATED",YEL),4:("CRITICAL",RED)}
SPOOF={0:("ERROR",RED),1:("UNKNOWN",GRY),2:("CLEAN",GRN),3:("SINGLE",YEL),4:("MULTIPLE",RED)}
OSNMA={0:("ERROR",RED),1:("DISABLED",GRY),2:("INIT",YEL),3:("WAIT_NTP",YEL),4:("VALID",GRN),5:("SPOOFED",RED)}
# (status field, constellation, signal) in display order
SIGNALS=[("gps_l1_used","GPS","L1"),("gps_l2_used","GPS","L2"),("gps_l5_used","GPS","L5"),
         ("glo_l1_used","GLO","L1"),("glo_l2_used","GLO","L2"),("glo_l3_used","GLO","L3"),
         ("gal_e1_used","GAL","E1"),("gal_e5a_used","GAL","E5a"),("gal_e5b_used","GAL","E5b"),
         ("gal_e5alt_used","GAL","E5alt"),("gal_e6_used","GAL","E6"),
         ("bds_b1_used","BDS","B1"),("bds_b2_used","BDS","B2"),("bds_b3_used","BDS","B3"),
         ("qzss_l1_used","QZSS","L1"),("qzss_l2_used","QZSS","L2"),("qzss_l5_used","QZSS","L5")]
def enum(tab,v):
    lab,col=tab.get(v,(f"?{v}",RED)); return c(col,lab)
def signals_used(st):
    """'GPS L1+L2 · GAL E1+E5b · ...' from the *_used flags of SbgGpsPosStatus."""
    by={}
    for f,con,sig in SIGNALS:
        if getattr(st,f,False): by.setdefault(con,[]).append(sig)
    return " · ".join(f"{k} {'+'.join(v)}" for k,v in by.items()) or "none"

# Antenna baseline the DEVICE is configured for: |leverArmSecondary - leverArmPrimary|
# from /api/v1/settings/aiding/gnss1. The measured baseline is compared against
# it, so a stale value here reads as a lever-arm error that is not there.
# 1.2567 = |[-1.07,0,0.13] - [0.18,0,0]|, written to flash 2026-08-01.
# Override without editing:  gnss --ros-args -p baseline_cfg:=<metres>
BASELINE_CFG=1.2567

class Track:
    def __init__(self): self.n=0; self.t0=time.time(); self.last=0.0; self.msg=None; self._hz=0.0
    def hit(self,m): self.n+=1; self.last=time.time(); self.msg=m
    def hz(self):
        dt=time.time()-self.t0
        if dt>=1.0: self._hz=self.n/dt; self.n=0; self.t0=time.time()
        return self._hz
    def age(self): return time.time()-self.last if self.last else 1e9
    def live(self,to=2.0): return self.age()<to

class Monitor(Node):
    def __init__(self):
        super().__init__("status_monitor")
        global BASELINE_CFG
        BASELINE_CFG = float(
            self.declare_parameter("baseline_cfg", BASELINE_CFG).value)
        q=QoSProfile(depth=10); q.reliability=ReliabilityPolicy.BEST_EFFORT
        self.tr={}
        def sub(t,ty): self.tr[t]=Track(); self.create_subscription(ty,t,lambda m,k=t:self.tr[k].hit(m),q)
        sub("/sbg/gps_pos",SbgGpsPos); sub("/sbg/gps_hdt",SbgGpsHdt); sub("/sbg/gps_vel",SbgGpsVel)
        sub("/sbg/imu_data",SbgImuData); sub("/sbg/status",SbgStatus)
        if HAVE_RTCM: sub("/ntrip_client/rtcm",RtcmMessage)
        self.create_timer(0.5,self.draw)

    def rate(self,t,want):
        tr=self.tr.get(t)
        if not tr or not tr.last: return c(GRY,"  -- ")
        if not tr.live(): return c(RED,"STALE")
        h=tr.hz(); col=GRN if h>=want*0.7 else YEL
        return c(col,f"{h:3.0f}Hz")

    def draw(self):
        L=[]
        A=L.append
        A(c(B+WHT,"  SBG ELLIPSE-D  ")+c(GRY,time.strftime("%H:%M:%S"))+c(GRY,"   raw GNSS (gps_pos / gps_hdt / gps_vel) -- EKF not used"))
        line="─"*60

        # ---- GNSS position (gps_pos) ----
        A(c(GRY,line))
        g=self.tr["/sbg/gps_pos"].msg
        if g and self.tr["/sbg/gps_pos"].live(4):
            st=g.status; ty=st.type; so=st.status
            tc=GRN if ty in(6,7,9) else (CYN if ty in(3,4) else (YEL if ty==2 else RED))
            sc=GRN if so==0 else RED
            A(c(B," POS  ")+c(tc,f"{FIX.get(ty,ty):9}")+" "+c(sc,SOL.get(so,so))+
              f"   sats {c(B,sv(g.num_sv_used))}/{sv(g.num_sv_tracked)}")
            hae=g.altitude+g.undulation
            A(f"   lat/lon {g.latitude:11.7f} {g.longitude:12.7f}  alt {g.altitude:6.1f}m MSL"
              +c(GRY,f"  (und {g.undulation:+.1f} → HAE {hae:.1f}m)"))
            pa=g.position_accuracy
            A("   pos σ N/E/D  "+grade(pa.x,0.5,2,"{:.2f}")+" / "+grade(pa.y,0.5,2,"{:.2f}")+
              " / "+grade(pa.z,0.8,3,"{:.2f}")+" m")
            A("   IFM "+enum(IFM,st.ifm)+"   spoof "+enum(SPOOF,st.spoofing)+
              "   osnma "+enum(OSNMA,st.osnma))
            A(c(GRY,"   sig ")+c(GRY,signals_used(st)))
        else:
            A(c(RED," POS  no gps_pos"))

        # ---- Dual-antenna heading (gps_hdt) ----
        h=self.tr["/sbg/gps_hdt"].msg
        if h and self.tr["/sbg/gps_hdt"].live(4):
            so=h.status & 0x3F
            resolved=so==0 and h.baseline>0.1
            rc=GRN if resolved else YEL
            A(c(B," HDT  ")+c(rc,"resolved" if resolved else "UNRESOLVED")+" "+c(GRN if so==0 else RED,SOL.get(so,so))+
              f"   sats {c(B,sv(h.num_sv_used))}/{sv(h.num_sv_tracked)}")
            A(f"   heading {h.true_heading:6.1f}° ±"+grade(h.true_heading_acc,1,3,"{:.2f}")+"°"
              f"   pitch {h.pitch:+5.1f}° ±"+grade(h.pitch_acc,1,3,"{:.2f}")+"°")
            dl=(h.baseline-BASELINE_CFG)*1000
            bc=GRN if abs(dl)<20 else (YEL if abs(dl)<50 else RED)
            A(f"   baseline "+c(bc,f"{h.baseline:5.3f}m")+c(GRY,f" [cfg {BASELINE_CFG}, Δ{dl:+.0f}mm]")+
              ("" if h.status & 0x40 else c(GRY,"  (baseline flag off)")))
        else:
            A(c(GRY," HDT  (gps_hdt off / unresolved)"))

        # ---- GNSS velocity (gps_vel) ----
        gv=self.tr["/sbg/gps_vel"].msg
        if gv and self.tr["/sbg/gps_vel"].live(4):
            vv=gv.velocity; spd=math.sqrt(vv.x*vv.x+vv.y*vv.y)
            A(c(B," VEL  ")+f"speed {spd:4.2f}m/s  course {gv.course:6.1f}° ±"+
              grade(gv.course_acc,1,3,"{:.2f}")+"°   "
              "σv "+grade(max(gv.velocity_accuracy.x,gv.velocity_accuracy.y),0.1,0.5,"{:.2f}")+"m/s")
        else:
            A(c(GRY," VEL  (gps_vel off)"))

        # ---- RTCM / RTK correction chain ----
        A(c(GRY,line))
        rt=self.tr.get("/ntrip_client/rtcm")
        if rt and rt.last and rt.live(3):
            nb=len(rt.msg.message) if rt.msg is not None else 0
            A(c(B," RTCM ")+c(GRN,"FLOWING")+f"  {self.rate('/ntrip_client/rtcm',1)}  last {nb}B "+c(GRY,"(NTRIP→device)"))
        elif HAVE_RTCM:
            A(c(B," RTCM ")+c(RED,"MISSING")+c(GRY,"  (NTRIP 미연결 → PSRDIFF 한계)"))
        else:
            A(c(B," RTCM ")+c(GRY,"rtcm_msgs 미설치"))
        if g and self.tr["/sbg/gps_pos"].live(4):
            ty=g.status.type
            rtkc=GRN if ty in(6,7,9) else (CYN if ty==3 else GRY)
            rtktxt={7:"RTK FIXED",6:"RTK FLOAT",3:"PSRDIFF",4:"SBAS",2:"SINGLE(no corr)"}.get(ty,FIX.get(ty,"?"))
            has_base=g.base_station_id not in (0,0xFFFF)
            if has_base:
                da=g.diff_age*0.01; dac=GRN if da<5 else (YEL if da<20 else RED)
                basetxt=f"base #{g.base_station_id}  diff_age "+c(dac,f"{da:.1f}s")
            else:
                basetxt=c(GRY,"base none (외부 보정 없음)")
            A("   "+basetxt+"   RTK: "+c(rtkc,rtktxt))

        # ---- raw IMU ----
        A(c(GRY,line))
        im=self.tr["/sbg/imu_data"].msg
        if im and self.tr["/sbg/imu_data"].live():
            am=math.sqrt(im.accel.x**2+im.accel.y**2+im.accel.z**2)
            ac=GRN if 9.6<am<10.0 else YEL
            A(c(B," IMU  ")+"|accel| "+c(ac,f"{am:.3f}")+"m/s²(g)  "
              f"gyro r/p/y {deg(im.gyro.x):+5.1f} {deg(im.gyro.y):+5.1f} {deg(im.gyro.z):+5.1f}°/s"
              f"  temp {im.temp:4.1f}°C")
        else:
            A(c(B," IMU  ")+c(RED,"MISSING")+c(GRY,"  (imu_data off -> no gyro heading between HDT epochs)"))

        # ---- rates + aiding ----
        A(c(GRY,line))
        A(c(B," RATE ")+f"gps_pos {self.rate('/sbg/gps_pos',5)} hdt {self.rate('/sbg/gps_hdt',5)} "
          f"vel {self.rate('/sbg/gps_vel',5)} imu {self.rate('/sbg/imu_data',25)} status {self.rate('/sbg/status',1)}")
        stt=self.tr["/sbg/status"].msg
        if stt and self.tr["/sbg/status"].live(3):
            a=stt.status_aiding; gen=stt.status_general; com=stt.status_com
            A(c(B," AID  ")+f"gps pos {yn(a.gps1_pos_recv)} vel {yn(a.gps1_vel_recv)} hdt {yn(a.gps1_hdt_recv)}   "
              f"gps_pwr {yn(gen.gps_power)} comA {yn(com.port_a)}")
        A("")
        print("\033[H"+"\n".join("\033[K"+x for x in L)+"\033[J",end="",flush=True)

def main():
    print("\033[2J",end="")
    rclpy.init(); n=Monitor()
    try: rclpy.spin(n)
    except KeyboardInterrupt: pass
    finally:
        print("\033[0m")
        n.destroy_node()
        try: rclpy.shutdown()
        except Exception: pass

if __name__=="__main__": main()
