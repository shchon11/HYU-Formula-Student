"""SBG Ellipse-D emulation: raw imu_data / gps_pos / gps_vel / gps_hdt.

Numbers are fitted to bag/0801_sensors/rosbag2_2026_08_01-17_22_34 (RTK
fixed throughout, 25 Hz IMU, 5 Hz receiver):

    imu_data     25.00 Hz, device dt exactly 40 000 us, frame imu_link_ned,
                 stationary accel std (0.035, 0.020, 0.012) m/s^2 with the unit
                 pitched so that accel.x reads -1.27 (z -9.72), gyro std
                 0.0005-0.0009 rad/s, gyro bias ~+0.0004/-0.0009/+0.0004, temp 41.7
    gps_pos      5 Hz, type 7, position_accuracy (0.0141, 0.0141, 0.0131),
                 stationary scatter 0.7/0.95 cm N/E, 1.6 cm alt, sv 40/32,
                 base 0xFFFF, diff_age ~88, undulation 18.586, altitude 31.49
    gps_vel      5 Hz, Doppler (type 2), accuracy 0.089-0.10 still / 0.067 moving,
                 scatter 0.018-0.020 m/s, course_acc 162 at rest / 1.8 moving
    gps_hdt      5 Hz, status 0x40 (0x41 = invalid, 3.9 % of epochs), baseline
                 1.509 m, true_heading_acc 0.38 deg, scatter 0.33 deg, pitch
                 2.64 +- 0.85 deg, sv 42/32
    receiver epochs arrive ~105 ms after the IMU sample of the same device
    time (header.stamp is the arrival time); the device clock is uint32 us.

Conventions: NED body (x fwd, y right, z down) -- what sbg_raw_ekf expects
with frame_convention ned. gps_vel is NED (N, E, D), course is degrees from
North; gps_hdt true_heading is degrees from North of the PRIMARY->SECONDARY
baseline, which on the car points BACKWARDS (secondary antenna behind), hence
the EKF's hdt_offset 180 and the +180 here.
"""
import math
import random
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

from sbg_driver.msg import SbgGpsHdt, SbgGpsPos, SbgGpsVel, SbgImuData

from .geo import LocalProjection, enu_yaw_to_ned_heading_deg

G = 9.80665

# gps_pos.status.type (SbgEComGpsPosType): 0 NO_SOLUTION 1 UNKNOWN 2 SINGLE
# 3 PSRDIFF 4 SBAS 5 OMNISTAR 6 RTK_FLOAT 7 RTK_INT 8 PPP_FLOAT 9 PPP_INT 10 FIXED
FIX_PRESETS = {
    # name: (status, type, sigma_xy_m, sigma_z_m, accuracy_xy, accuracy_z, sv_used, diff_age)
    'rtk_fixed': (0, 7, 0.009, 0.016, 0.0141, 0.0131, 32, 88),
    'rtk_float': (0, 6, 0.15, 0.30, 0.25, 0.40, 30, 120),
    'dgps':      (0, 3, 0.50, 0.90, 0.60, 1.00, 24, 300),
    'single':    (0, 2, 1.20, 2.00, 1.50, 2.50, 18, 0),
    'outage':    (1, 0, 0.0, 0.0, 0.0, 0.0, 0, 0),     # status INSUFFICIENT_OBS: invalid
}

# Per-mode slow (correlated) position-bias drive [m per epoch, OU process]. This
# is the DANGEROUS part of GNSS: a wandering bias the receiver does NOT report
# in its accuracy, so a filter that trusts position too much drifts with it.
# rtk_fixed ~ mm; float ~ cm; dgps ~ dm; single ~ m. Scale up with
# pos_bias_walk_scale for a conservative (harsher) test.
FIX_BIAS_SD = {'rtk_fixed': 0.0007, 'rtk_float': 0.02, 'dgps': 0.08,
               'single': 0.30, 'outage': 0.0}


@dataclass
class SbgParams:
    imu_rate_hz: float = 25.0
    gps_rate_hz: float = 5.0
    receiver_latency_s: float = 0.105
    device_ts0_us: int = 3_320_000_000      # as the 0801 device counter; wraps at 2^32 us
    frame_id: str = 'imu_link_ned'
    # IMU
    # Mount tilt is PER-MOUNT (changes on every remount); the device does not
    # level it, so a resting accel leaks gravity onto x/y. The EKF is built to
    # absorb this in its b_ax/b_ay states (ZUPT), so the sim's job is only to
    # present a realistic constant leak -- the exact value is an example mount
    # (0801: accel.x -1.27 -> pitch -7.45). Set imu_tilt_random_deg>0 to draw a
    # fresh pitch/roll each run and stress-test that the EKF absorbs ANY mount.
    imu_pitch_deg: float = -7.45            # example mount (0801); per-mount
    imu_roll_deg: float = 0.22              # per-mount
    imu_tilt_random_deg: float = 0.0        # >0: pitch,roll ~ U(-x,x) per run (mount robustness test)
    accel_noise: List[float] = field(default_factory=lambda: [0.035, 0.020, 0.012])
    gyro_noise: List[float] = field(default_factory=lambda: [0.0005, 0.0005, 0.0009])
    # Gyro bias is per-UNIT and per-POWER-CYCLE (turn-on repeatability) and
    # drifts in-run (bias instability). The EKF learns the z bias in b_gz via
    # ZARU and tracks its drift (sig_bg_rw). Default = the 0801 boot's value;
    # gyro_bias_random_dps>0 draws a fresh turn-on bias per run, and
    # gyro_bias_walk_dps_per_sqrt_s>0 adds in-run drift, to test b_gz tracking.
    gyro_bias: List[float] = field(default_factory=lambda: [0.00045, -0.00085, 0.00038])
    gyro_bias_random_dps: float = 0.0       # >0: each axis turn-on bias ~ U(-x,x) deg/s per run
    gyro_bias_walk_dps_per_sqrt_s: float = 0.0  # >0: in-run random-walk drift (bias instability)
    accel_bias: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    accel_vib_per_mps: float = 0.012        # extra accel noise per m/s (vibration)
    gyro_vib_per_mps: float = 0.0004
    temp_c: float = 41.7
    # receiver
    fix: str = 'rtk_fixed'
    fix_schedule: str = ''                  # "120:rtk_float,135:rtk_fixed,200:outage,203:rtk_fixed"
    altitude_m: float = 31.49
    undulation_m: float = 18.586
    sv_tracked: int = 40
    vel_noise: float = 0.019
    vel_accuracy_still: float = 0.10
    vel_accuracy_moving: float = 0.067
    course_acc_moving_deg: float = 1.8
    course_acc_still_deg: float = 162.0
    hdt_offset_deg: float = 180.0           # primary->secondary baseline points backwards
    hdt_noise_deg: float = 0.33
    hdt_accuracy_deg: float = 0.38
    hdt_dropout_prob: float = 0.039
    hdt_baseline_m: float = 1.509
    hdt_pitch_deg: float = 2.64
    hdt_pitch_noise_deg: float = 0.85
    hdt_sv_tracked: int = 42
    hdt_sv_used: int = 32
    # --- conservative GNSS-degradation knobs -------------------------------
    pos_bias_walk_scale: float = 1.0        # amplify the per-mode correlated bias (harsher test)
    pos_jump_prob: float = 0.01             # per-epoch multipath spike (~every 20 s @5Hz); ON by default (realism)
    pos_jump_m: float = 1.5                # multipath spike magnitude (1-sigma, m) -- NOT in reported accuracy
    hdt_dropout_degraded: float = 0.5      # extra HDT dropout prob when fix is not rtk_fixed


def _rot_rpy(roll: float, pitch: float, yaw: float):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return ((cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
            (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
            (-sp, cp * sr, cp * cr))


def _matT_vec(m, v):
    return (m[0][0] * v[0] + m[1][0] * v[1] + m[2][0] * v[2],
            m[0][1] * v[0] + m[1][1] * v[1] + m[2][1] * v[2],
            m[0][2] * v[0] + m[1][2] * v[1] + m[2][2] * v[2])


def parse_fix_schedule(text: str) -> List[Tuple[float, str]]:
    out = []
    for item in (text or '').split(','):
        item = item.strip()
        if not item:
            continue
        t, name = item.split(':', 1)
        name = name.strip()
        if name not in FIX_PRESETS:
            raise ValueError(f'unknown fix preset {name!r}; one of {sorted(FIX_PRESETS)}')
        out.append((float(t), name))
    out.sort()
    return out


class SbgEmulator:
    def __init__(self, params: SbgParams, proj: LocalProjection, rng: Optional[random.Random] = None):
        self.p = params
        self.proj = proj
        self.rng = rng or random.Random(7)
        self.imu_dt = 1.0 / params.imu_rate_hz
        self.gps_every = max(1, int(round(params.imu_rate_hz / params.gps_rate_hz)))
        pitch_deg, roll_deg = params.imu_pitch_deg, params.imu_roll_deg
        if params.imu_tilt_random_deg > 0.0:
            pitch_deg = self.rng.uniform(-params.imu_tilt_random_deg, params.imu_tilt_random_deg)
            roll_deg = self.rng.uniform(-params.imu_tilt_random_deg, params.imu_tilt_random_deg)
            self.p.imu_pitch_deg, self.p.imu_roll_deg = pitch_deg, roll_deg
        self._mount = _rot_rpy(math.radians(roll_deg), math.radians(pitch_deg), 0.0)
        if params.gyro_bias_random_dps > 0.0:
            self.p.gyro_bias = [math.radians(self.rng.uniform(-params.gyro_bias_random_dps,
                                                              params.gyro_bias_random_dps)) for _ in range(3)]
        self._gyro_walk = [0.0, 0.0, 0.0]   # accumulated in-run drift [rad/s]
        self._schedule = parse_fix_schedule(params.fix_schedule)
        self.fix_name = params.fix
        # slow RTK bias random walk (mm-level), keeps the scatter non-white like a real fix
        self._bias_n = 0.0
        self._bias_e = 0.0

    # --- clock ---------------------------------------------------------------
    def device_ts(self, tick: int) -> int:
        return (self.p.device_ts0_us + int(round(tick * self.imu_dt * 1e6))) & 0xFFFFFFFF

    def fix_at(self, t_sim: float) -> str:
        name = self.p.fix
        for t, n in self._schedule:
            if t_sim >= t:
                name = n
        self.fix_name = name
        return name

    # --- IMU -----------------------------------------------------------------
    def imu_msg(self, stamp, tick: int, a_body_enu: Tuple[float, float], yaw_rate: float,
                speed: float) -> SbgImuData:
        """a_body_enu = (ax fwd, ay left) of the IMU point in ROS body axes; yaw_rate CCW."""
        p = self.p
        # Specific force in a level NED body frame: f = a - g, g = (0,0,+G) down.
        f_level = (a_body_enu[0], -a_body_enu[1], -G)
        w_level = (0.0, 0.0, -yaw_rate)
        # Mount tilt: sensor axes = R^T * level axes.
        f = _matT_vec(self._mount, f_level)
        w = _matT_vec(self._mount, w_level)
        vib_a = p.accel_vib_per_mps * abs(speed)
        vib_w = p.gyro_vib_per_mps * abs(speed)
        m = SbgImuData()
        m.header.stamp = stamp
        m.header.frame_id = p.frame_id
        m.time_stamp = self.device_ts(tick)
        st = m.imu_status
        st.imu_com = st.imu_status = True
        st.imu_accel_x = st.imu_accel_y = st.imu_accel_z = True
        st.imu_gyro_x = st.imu_gyro_y = st.imu_gyro_z = True
        st.imu_accels_in_range = st.imu_gyros_in_range = True
        st.imu_gyros_use_high_scale = False
        acc = [f[i] + p.accel_bias[i] + self.rng.gauss(0.0, p.accel_noise[i] + vib_a) for i in range(3)]
        if p.gyro_bias_walk_dps_per_sqrt_s > 0.0:
            sd = math.radians(p.gyro_bias_walk_dps_per_sqrt_s) * math.sqrt(self.imu_dt)
            self._gyro_walk = [self._gyro_walk[i] + self.rng.gauss(0.0, sd) for i in range(3)]
        gyr = [w[i] + p.gyro_bias[i] + self._gyro_walk[i] + self.rng.gauss(0.0, p.gyro_noise[i] + vib_w) for i in range(3)]
        m.accel.x, m.accel.y, m.accel.z = acc
        m.gyro.x, m.gyro.y, m.gyro.z = gyr
        m.temp = p.temp_c
        # The driver's delta_vel/delta_angle carry the averaged acceleration / rate
        # (same units as accel/gyro in the 0801 logs), not dt-integrated deltas.
        m.delta_vel.x, m.delta_vel.y, m.delta_vel.z = acc
        m.delta_angle.x, m.delta_angle.y, m.delta_angle.z = gyr
        return m

    # --- receiver epoch ----------------------------------------------------------
    def gps_msgs(self, stamp, tick: int, t_sim: float, ant_xy_world: Tuple[float, float],
                 ant_v_world: Tuple[float, float], yaw_enu: float):
        """-> (SbgGpsPos, SbgGpsVel, SbgGpsHdt) for one epoch; stamps = arrival time (set by caller)."""
        p = self.p
        ts = self.device_ts(tick)
        tow_ms = int((t_sim * 1000.0) % (7 * 86400 * 1000))
        status, ftype, s_xy, s_z, acc_xy, acc_z, sv_used, diff_age = FIX_PRESETS[self.fix_at(t_sim)]

        # ---- position (antenna) ----
        # Slow correlated bias, driven per fix mode (mm on RTK-fixed, up to ~m on
        # single). Weaker mean-reversion (0.03) so it persists like real multipath
        # bias -- the receiver still reports the optimistic preset accuracy.
        drive = FIX_BIAS_SD.get(self.fix_name, 0.0007) * p.pos_bias_walk_scale
        self._bias_n += self.rng.gauss(0.0, drive) - 0.03 * self._bias_n
        self._bias_e += self.rng.gauss(0.0, drive) - 0.03 * self._bias_e
        # Multipath spike: an occasional large transient offset the filter must
        # reject by chi^2 gating (not reflected in position_accuracy).
        jn = je = 0.0
        if p.pos_jump_prob > 0.0 and status == 0 and self.rng.random() < p.pos_jump_prob:
            jn = self.rng.gauss(0.0, p.pos_jump_m)
            je = self.rng.gauss(0.0, p.pos_jump_m)
        e = ant_xy_world[0] + self._bias_e + je + self.rng.gauss(0.0, s_xy)
        n = ant_xy_world[1] + self._bias_n + jn + self.rng.gauss(0.0, s_xy)
        lat, lon = self.proj.to_latlon(n, e)
        pos = SbgGpsPos()
        pos.header.stamp = stamp
        pos.header.frame_id = p.frame_id
        pos.time_stamp = ts
        pos.gps_tow = tow_ms
        pos.status.status = status
        pos.status.type = ftype
        pos.status.ifm = 2
        pos.status.spoofing = 2
        pos.status.osnma = 1
        for name in ('gps_l1_used', 'gps_l2_used', 'gps_l5_used', 'glo_l1_used', 'glo_l2_used',
                     'gal_e1_used', 'gal_e5a_used', 'gal_e5b_used', 'bds_b1_used', 'bds_b2_used',
                     'qzss_l1_used', 'qzss_l2_used'):
            setattr(pos.status, name, status == 0)
        if status == 0:
            pos.latitude, pos.longitude = lat, lon
            pos.altitude = p.altitude_m + self.rng.gauss(0.0, s_z)
            pos.position_accuracy.x = pos.position_accuracy.y = acc_xy + abs(self.rng.gauss(0.0, 0.0004))
            pos.position_accuracy.z = acc_z + abs(self.rng.gauss(0.0, 0.002))
        else:
            pos.latitude = pos.longitude = pos.altitude = 0.0
        pos.undulation = p.undulation_m
        pos.num_sv_tracked = p.sv_tracked
        pos.num_sv_used = sv_used
        pos.base_station_id = 0xFFFF if ftype >= 6 else 0
        pos.diff_age = max(0, diff_age + self.rng.randint(-8, 8)) if ftype >= 3 else 0

        # ---- velocity (antenna), NED ----
        vn = ant_v_world[1] + self.rng.gauss(0.0, p.vel_noise)
        ve = ant_v_world[0] + self.rng.gauss(0.0, p.vel_noise)
        speed = math.hypot(vn, ve)
        vel = SbgGpsVel()
        vel.header.stamp = stamp
        vel.header.frame_id = p.frame_id
        vel.time_stamp = ts
        vel.gps_tow = tow_ms
        vel.status.vel_status = status
        vel.status.vel_type = 2 if status == 0 else 0   # DOPPLER
        if status == 0:
            vel.velocity.x, vel.velocity.y = vn, ve
            vel.velocity.z = self.rng.gauss(0.0, p.vel_noise)
            acc = p.vel_accuracy_moving if speed > 1.0 else p.vel_accuracy_still
            vel.velocity_accuracy.x = vel.velocity_accuracy.y = vel.velocity_accuracy.z = acc
            vel.course = math.degrees(math.atan2(ve, vn)) % 360.0
            vel.course_acc = p.course_acc_moving_deg if speed > 1.0 else p.course_acc_still_deg
        # ---- dual-antenna heading ----
        hdt = SbgGpsHdt()
        hdt.header.stamp = stamp
        hdt.header.frame_id = p.frame_id
        hdt.time_stamp = ts
        hdt.tow = tow_ms
        hdt.baseline = p.hdt_baseline_m
        hdt.num_sv_tracked = p.hdt_sv_tracked
        drop = p.hdt_dropout_prob + (0.0 if ftype >= 7 else p.hdt_dropout_degraded)
        valid = status == 0 and self.rng.random() >= drop
        if valid:
            hdt.status = 0x40      # baseline valid, SOL_COMPUTED
            heading = enu_yaw_to_ned_heading_deg(yaw_enu) + p.hdt_offset_deg
            hdt.true_heading = (heading + self.rng.gauss(0.0, p.hdt_noise_deg)) % 360.0
            hdt.true_heading_acc = p.hdt_accuracy_deg + abs(self.rng.gauss(0.0, 0.02))
            hdt.pitch = p.hdt_pitch_deg + self.rng.gauss(0.0, p.hdt_pitch_noise_deg)
            hdt.pitch_acc = 0.48
            hdt.num_sv_used = p.hdt_sv_used
        else:
            hdt.status = 0x41      # INSUFFICIENT_OBS
            hdt.true_heading = 0.0
            hdt.true_heading_acc = 0.0
            hdt.num_sv_used = 0
        return pos, vel, hdt
