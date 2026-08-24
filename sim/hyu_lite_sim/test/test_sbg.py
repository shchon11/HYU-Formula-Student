import math
import random

from builtin_interfaces.msg import Time

from hyu_lite_sim.geo import LocalProjection, enu_yaw_to_ned_heading_deg
from hyu_lite_sim.sbg_emu import SbgEmulator, SbgParams, parse_fix_schedule


def _emu(**kw):
    p = SbgParams(hdt_dropout_prob=0.0, hdt_noise_deg=0.0, vel_noise=0.0, **kw)
    return SbgEmulator(p, LocalProjection(37.5552263, 127.0454965), random.Random(1))


def test_device_clock_and_epoch_cadence():
    e = _emu()
    assert e.device_ts(1) - e.device_ts(0) == 40000
    assert e.gps_every == 5
    e2 = _emu(device_ts0_us=0xFFFFFFFF - 20000)
    assert e2.device_ts(1) == 19999                     # uint32 wrap, as the real counter


def test_heading_convention_matches_ekf():
    e = _emu()
    # car facing East (ENU yaw 0): NED heading 90; the baseline points backwards -> 270
    pos, vel, hdt = e.gps_msgs(Time(), 0, 0.0, (10.0, 20.0), (3.0, 0.0), 0.0)
    assert abs(hdt.true_heading - 270.0) < 1e-6
    assert (hdt.status & 0x3F) == 0
    # gps_vel is NED: vN = world y-velocity, vE = world x-velocity; course from North
    assert abs(vel.velocity.x - 0.0) < 1e-9 and abs(vel.velocity.y - 3.0) < 1e-9
    assert abs(vel.course - 90.0) < 1e-6
    assert pos.status.type == 7 and pos.status.status == 0
    # antenna position round-trips through the same projection the EKF uses
    n, east = e.proj.to_ne(pos.latitude, pos.longitude)
    assert abs(east - 10.0) < 0.05 and abs(n - 20.0) < 0.05


def test_fix_schedule_and_outage():
    e = _emu(fix_schedule='10:outage,20:rtk_fixed')
    assert parse_fix_schedule('20:rtk_fixed, 10:outage') == [(10.0, 'outage'), (20.0, 'rtk_fixed')]
    assert e.fix_at(5.0) == 'rtk_fixed'
    assert e.fix_at(15.0) == 'outage'
    pos, vel, hdt = e.gps_msgs(Time(), 0, 15.0, (0.0, 0.0), (0.0, 0.0), 0.0)
    assert pos.status.status != 0 and vel.status.vel_status != 0 and (hdt.status & 0x3F) != 0
    assert e.fix_at(25.0) == 'rtk_fixed'


def test_imu_at_rest_reads_gravity_with_mount_tilt():
    e = _emu(accel_noise=[0.0, 0.0, 0.0], gyro_noise=[0.0, 0.0, 0.0], gyro_bias=[0.0, 0.0, 0.0])
    m = e.imu_msg(Time(), 0, (0.0, 0.0), 0.0, 0.0)
    assert abs(m.accel.x + 1.27) < 0.02 and abs(m.accel.z + 9.72) < 0.02    # as the 0801 unit at rest
    # a left turn: yaw rate CCW 0.5 rad/s -> NED gyro z = -0.5 (roughly; tilt mixes a little)
    m = e.imu_msg(Time(), 1, (0.0, 0.0), 0.5, 3.0)
    assert m.gyro.z < -0.45
    assert enu_yaw_to_ned_heading_deg(math.pi / 2) == 0.0
