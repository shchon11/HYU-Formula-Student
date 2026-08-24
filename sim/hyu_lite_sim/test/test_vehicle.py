import math

from hyu_lite_sim.vehicle import BicycleVehicle, VehicleParams


def _run(car, seconds, dt=0.01):
    for _ in range(int(seconds / dt)):
        car.step(dt)


def test_straight_line_reaches_setpoint_and_wheels_agree():
    car = BicycleVehicle(VehicleParams())
    car.set_command(3.0, 0.0, True)
    _run(car, 6.0)
    assert abs(car.s.v - 3.0) < 0.15
    ws = car.wheel_speeds()
    assert all(abs(w - car.s.v) < 1e-6 for w in ws)
    # RPM -> m/s round trip with the bridge's constant (pi*D/60)
    rpm = car.wheel_rpm(None, 0.0, 0.0)
    k = math.pi * car.p.tire_diameter_m / 60.0
    for r, scale, w in zip(rpm, car.p.wheel_scale, ws):
        assert abs(r * k * scale - w) < 1e-6


def test_left_turn_geometry():
    car = BicycleVehicle(VehicleParams())
    car.set_command(3.0, 0.2, True)
    _run(car, 6.0)
    assert car.s.yaw_rate > 0.0
    fl, fr, rl, rr = car.wheel_speeds()
    assert rl < rr                       # inner rear wheel slower
    assert fl < fr
    assert fl > rl                       # front wheels run faster than rears in a turn
    # centripetal acceleration at the axle ~ v * yaw_rate
    ax, ay = car.body_point_acceleration(0.0, 0.0)
    assert abs(ay - car.s.v * car.s.yaw_rate) < 0.05


def test_disabled_brakes_to_rest_and_holds_steering():
    car = BicycleVehicle(VehicleParams())
    car.set_command(4.0, 0.3, True)
    _run(car, 6.0)
    delta = car.s.delta
    car.set_command(4.0, -0.3, False)    # ECU enable dropped: command ignored
    _run(car, 4.0)
    assert car.s.v == 0.0
    assert abs(car.s.delta - delta) < 1e-9
    assert all(r == 0.0 for r in car.wheel_rpm())


def test_base_pose_is_behind_rear_axle():
    p = VehicleParams(rear_axle_x_m=0.91)
    car = BicycleVehicle(p)
    car.teleport(10.0, 5.0, math.pi / 2)
    bx, by, byaw = car.base_pose()
    assert abs(bx - 10.0) < 1e-9 and abs(by - (5.0 - 0.91)) < 1e-9 and abs(byaw - math.pi / 2) < 1e-9
    wx, wy = car.world_point(1.25 - 0.91, 0.0)      # antenna 1.25 ahead of base
    assert abs(wx - 10.0) < 1e-9 and abs(wy - (5.0 - 0.91 + 1.25)) < 1e-9
