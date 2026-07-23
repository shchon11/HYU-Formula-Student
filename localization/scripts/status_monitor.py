#!/usr/bin/env python3
"""Live SBG Ellipse-D status dashboard.

One-glance terminal panel of the INS/GNSS sensor itself: solution mode, fused
position/velocity/attitude with their 1-sigma accuracies, GNSS fix quality,
dual-antenna heading + measured baseline, raw IMU sanity, the bias-corrected
rot/accel, signal rates and aiding flags. Colour-coded green/yellow/red.

    ros2 run hyu_localization status_monitor      # after build
    python3 status_monitor.py                      # standalone
Refreshes ~2 Hz. Ctrl+C to quit.
"""
import math, time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sbg_driver.msg import (SbgEkfNav, SbgEkfEuler, SbgGpsPos, SbgGpsHdt,
                            SbgGpsVel, SbgImuData, SbgEkfRotAccel, SbgStatus)

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
MODE={0:"UNINIT",1:"VERT_GYRO",2:"AHRS",3:"NAV_VEL",4:"NAV_POS"}
FIX={0:"NO_SOL",1:"UNKNOWN",2:"SINGLE",3:"PSRDIFF",4:"SBAS",5:"OMNISTAR",6:"RTK_FLOAT",7:"RTK_FIXED",8:"PPP_FLOAT",9:"PPP_FIXED",10:"FIXED"}
BASELINE_CFG=1.219

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
        q=QoSProfile(depth=10); q.reliability=ReliabilityPolicy.BEST_EFFORT
        self.tr={}
        def sub(t,ty): self.tr[t]=Track(); self.create_subscription(ty,t,lambda m,k=t:self.tr[k].hit(m),q)
        sub("/sbg/ekf_nav",SbgEkfNav); sub("/sbg/ekf_euler",SbgEkfEuler)
        sub("/sbg/gps_pos",SbgGpsPos); sub("/sbg/gps_hdt",SbgGpsHdt); sub("/sbg/gps_vel",SbgGpsVel)
        sub("/sbg/imu_data",SbgImuData); sub("/sbg/ekf_rot_accel_body",SbgEkfRotAccel)
        sub("/sbg/status",SbgStatus)
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
        A(c(B+WHT,"  SBG ELLIPSE-D  ")+c(GRY,time.strftime("%H:%M:%S")))
        line="─"*60

        # ---- INS solution ----
        A(c(GRY,line))
        m=self.tr["/sbg/ekf_nav"].msg
        if m and self.tr["/sbg/ekf_nav"].live():
            s=m.status; mo=s.solution_mode
            mc=GRN if mo==4 else (YEL if mo>=2 else RED)
            A(c(B," INS  ")+c(mc,f"{MODE.get(mo,mo)}({mo})")+
              f"   pos {yn(s.position_valid)}  vel {yn(s.velocity_valid)}")
            pa=m.position_accuracy
            A(f"   lat/lon {m.latitude:11.7f} {m.longitude:12.7f}  alt {m.altitude:6.1f}m")
            A("   pos σ N/E/D  "+grade(pa.x,0.5,2,"{:.2f}")+" / "+grade(pa.y,0.5,2,"{:.2f}")+
              " / "+grade(pa.z,0.8,3,"{:.2f}")+" m")
            v=m.velocity; va=m.velocity_accuracy; sp=math.sqrt(v.x*v.x+v.y*v.y)
            A(f"   vel NED {v.x:+5.2f} {v.y:+5.2f} {v.z:+5.2f}  |v| {sp:4.2f}m/s   "
              "σ "+grade(max(va.x,va.y),0.1,0.5,"{:.2f}")+"m/s")
        else:
            A(c(RED," INS  no ekf_nav"))

        # ---- Attitude ----
        e=self.tr["/sbg/ekf_euler"].msg
        if e and self.tr["/sbg/ekf_euler"].live():
            st=e.status
            A(c(B," ATT  ")+f"roll {deg(e.angle.x):+6.1f}°  pitch {deg(e.angle.y):+6.1f}°  "
              f"yaw {deg(e.angle.z):6.1f}°")
            A("   σ r/p/y "+grade(deg(e.accuracy.x),0.5,2,"{:.2f}")+" / "+
              grade(deg(e.accuracy.y),0.5,2,"{:.2f}")+" / "+grade(deg(e.accuracy.z),1,3,"{:.2f}")+"°   "
              f"att {yn(st.attitude_valid)}  hdg {yn(st.heading_valid)}")

        # ---- GNSS position ----
        A(c(GRY,line))
        g=self.tr["/sbg/gps_pos"].msg
        if g and self.tr["/sbg/gps_pos"].live(4):
            ty=g.status.type
            tc=GRN if ty in(6,7,9) else (CYN if ty in(3,4) else (YEL if ty==2 else RED))
            pac=g.position_accuracy; la=pac.x; lo=pac.y; al=pac.z
            A(c(B," GNSS ")+c(tc,f"{FIX.get(ty,ty):9}")+
              f" sats {c(B,g.num_sv_used)}/{g.num_sv_tracked}   "
              "σ lat/lon "+grade(max(la,lo),0.5,2,"{:.2f}")+"m alt "+grade(al,1,3,"{:.2f}")+"m")
        else:
            A(c(GRY," GNSS (gps_pos off)"))

        # ---- Dual-antenna heading ----
        h=self.tr["/sbg/gps_hdt"].msg
        if h and self.tr["/sbg/gps_hdt"].live(4):
            resolved=(h.status & 0x3F)==0 and h.baseline>0.1
            rc=GRN if resolved else YEL
            A(c(B," HDT  ")+c(rc,"resolved" if resolved else "UNRESOLVED")+
              f"   heading {h.true_heading:6.1f}° ±"+grade(h.true_heading_acc,1,3,"{:.2f}")+"°"
              f"  pitch {h.pitch:+5.1f}°")
            dl=(h.baseline-BASELINE_CFG)*1000
            bc=GRN if abs(dl)<20 else (YEL if abs(dl)<50 else RED)
            A(f"   baseline "+c(bc,f"{h.baseline:5.3f}m")+c(GRY,f" [cfg {BASELINE_CFG}, Δ{dl:+.0f}mm]")+
              f"   sats {h.num_sv_used}")
        else:
            A(c(GRY," HDT  (gps_hdt off / unresolved)"))

        # ---- GNSS velocity ----
        gv=self.tr["/sbg/gps_vel"].msg
        if gv and self.tr["/sbg/gps_vel"].live(4):
            vv=gv.velocity; spd=math.sqrt(vv.x*vv.x+vv.y*vv.y)
            A(c(B," VEL  ")+f"speed {spd:4.2f}m/s  course {gv.course:6.1f}° ±"+
              grade(gv.course_acc,1,3,"{:.2f}")+"°   "
              "σv "+grade(max(gv.velocity_accuracy.x,gv.velocity_accuracy.y),0.1,0.5,"{:.2f}")+"m/s")

        # ---- IMU + rot/accel ----
        A(c(GRY,line))
        im=self.tr["/sbg/imu_data"].msg
        if im and self.tr["/sbg/imu_data"].live():
            am=math.sqrt(im.accel.x**2+im.accel.y**2+im.accel.z**2)
            gm=deg(math.sqrt(im.gyro.x**2+im.gyro.y**2+im.gyro.z**2))
            ac=GRN if 9.6<am<10.0 else YEL
            A(c(B," IMU  ")+"|accel| "+c(ac,f"{am:.3f}")+"m/s²(g)  "
              f"gyro {gm:5.1f}°/s  temp {im.temp:4.1f}°C")
        ra=self.tr["/sbg/ekf_rot_accel_body"].msg
        if ra and self.tr["/sbg/ekf_rot_accel_body"].live():
            A(c(B," ROT  ")+f"rate r/p/y {deg(ra.rate.x):+5.1f} {deg(ra.rate.y):+5.1f} {deg(ra.rate.z):+5.1f}°/s"
              +c(GRY,"  (bias-corrected yaw for DR)"))
        else:
            A(c(B," ROT  ")+c(RED,"MISSING")+c(GRY,"  (ekfRotAccelBody off -> no GNSS-free DR)"))

        # ---- rates + aiding ----
        A(c(GRY,line))
        A(c(B," RATE ")+f"nav {self.rate('/sbg/ekf_nav',25)} eul {self.rate('/sbg/ekf_euler',25)} "
          f"imu {self.rate('/sbg/imu_data',25)} rot {self.rate('/sbg/ekf_rot_accel_body',25)}")
        A("      "+f"gps_pos {self.rate('/sbg/gps_pos',5)} hdt {self.rate('/sbg/gps_hdt',5)} vel {self.rate('/sbg/gps_vel',5)}")
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
