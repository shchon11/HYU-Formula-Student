"""Closed-loop tests against the TRUE plant: a kinematic bicycle whose
steering actuator is slew-limited exactly like the simulator's
(steeringLockTime -> ~1.04 rad/s). The MPC must converge onto straight and
circular paths and hold them without weaving."""

import math

import pytest

from mpc_controller.mpc_core import EgoState, MpcConfig, PathPoint, solve


class Plant:
    """Kinematic bicycle + slew-limited steering actuator (the sim's law)."""

    def __init__(self, ego, config):
        self.state = ego
        self.actual_steering = 0.0
        self.config = config

    def step(self, commanded_steering, acceleration, dt):
        limit = self.config.max_steering_rate_radps * dt
        difference = commanded_steering - self.actual_steering
        self.actual_steering += max(-limit, min(limit, difference))
        s = self.state
        self.state = EgoState(
            s.x + s.speed * math.cos(s.yaw) * dt,
            s.y + s.speed * math.sin(s.yaw) * dt,
            s.yaw + s.speed * math.tan(self.actual_steering) / self.config.wheelbase_m * dt,
            max(0.0, s.speed + acceleration * dt),
        )


def simulate(path, ego, config, steps, dt=0.05):
    """`path` is either a fixed list or a callable(state) -> windowed list,
    mirroring the real stack where the selector publishes a window AHEAD of
    the car (a fixed self-overlapping loop would alias the nearest search)."""
    plant = Plant(ego, config)
    previous = None
    estimated = 0.0
    trim = 0.0
    trace = []
    for _ in range(steps):
        current_path = path(plant.state) if callable(path) else path
        solution = solve(
            current_path, plant.state, config, previous, current_steering=estimated)
        assert solution is not None, f"solver failed at {plant.state}"
        assert abs(solution.steering_rad) <= config.max_steering_rad + 1e-9
        assert config.min_acceleration_mps2 - 1e-9 <= solution.acceleration_mps2
        assert solution.acceleration_mps2 <= config.max_acceleration_mps2 + 1e-9
        # Node-side LEAKY integral trim (same law as mpc_controller_node):
        # stale bias washes out when curvature flips; anti-windup on big errors.
        trim *= max(0.0, 1.0 - 0.3 * dt)
        if abs(solution.lateral_error_m) < 0.5:
            trim -= 0.12 * solution.lateral_error_m * dt
        trim = max(-0.08, min(0.08, trim))
        commanded = max(-config.max_steering_rad,
                        min(config.max_steering_rad, solution.steering_rad + trim))
        plant.step(commanded, solution.acceleration_mps2, dt)
        # Track the actuator estimate exactly as the node does.
        limit = config.max_steering_rate_radps * dt
        difference = commanded - estimated
        new_estimated = estimated + max(-limit, min(limit, difference))
        rate = (new_estimated - estimated) / dt
        estimated = new_estimated
        previous = (rate, solution.acceleration_mps2)
        trace.append(plant.state)
    return trace


def test_straight_line_offset_converges_without_weave():
    config = MpcConfig()
    path = [PathPoint(float(x), 0.0, 4.0) for x in range(0, 80, 1)]
    ego = EgoState(0.0, 1.0, 0.0, 3.0)  # 1 m left of the lane
    trace = simulate(path, ego, config, steps=160)
    settled = trace[100:]
    assert max(abs(s.y) for s in settled) < 0.15
    # No weave: once settled the lateral error must not re-grow.
    crossings = sum(
        1 for a, b in zip(settled, settled[1:]) if a.y * b.y < 0 and abs(a.y) > 0.05)
    assert crossings == 0


def test_circle_tracking_stays_centred():
    config = MpcConfig()
    radius = 9.125
    step = 2.0 * math.pi / 128

    def window(state):
        """50 waypoints ahead of the car, like the selector's rolling window."""
        angle = math.atan2(state.y, state.x)
        start = int(angle / step) - 2
        return [
            PathPoint(radius * math.cos((start + i) * step),
                      radius * math.sin((start + i) * step), 4.0)
            for i in range(50)
        ]

    ego = EgoState(radius, 0.0, math.pi / 2.0, 3.0)
    trace = simulate(window, ego, config, steps=400)
    # Allow the integral trim to settle (~10 s), then require centred tracking.
    settled = trace[220:]
    worst = max(abs(math.hypot(s.x, s.y) - radius) for s in settled)
    assert worst < 0.30, f"radial error {worst:.3f} m"


def test_speed_tracks_profile():
    config = MpcConfig()
    path = [PathPoint(float(x), 0.0, 5.0) for x in range(0, 80, 1)]
    ego = EgoState(0.0, 0.0, 0.0, 1.0)
    trace = simulate(path, ego, config, steps=120)
    assert abs(trace[-1].speed - 5.0) < 0.4


def test_path_end_brings_reference_to_rest():
    config = MpcConfig()
    path = [PathPoint(float(x), 0.0, 4.0) for x in range(0, 10, 1)]
    ego = EgoState(6.0, 0.0, 0.0, 4.0)
    solution = solve(path, ego, config, None, current_steering=0.0)
    assert solution is not None
    assert solution.acceleration_mps2 < 0.0


def test_degenerate_inputs_fail_closed():
    config = MpcConfig()
    assert solve([], EgoState(0, 0, 0, 0), config, None) is None
    assert solve([PathPoint(0, 0, 1)], EgoState(0, 0, 0, 0), config, None) is None
    bad = EgoState(float("nan"), 0.0, 0.0, 0.0)
    path = [PathPoint(float(x), 0.0, 2.0) for x in range(10)]
    assert solve(path, bad, config, None) is None
    assert solve(path, EgoState(0, 0, 0, 1), config, None,
                 current_steering=float("nan")) is None


def test_sharp_heading_error_recovers():
    config = MpcConfig()
    path = [PathPoint(float(x), 0.0, 3.0) for x in range(0, 40, 1)]
    ego = EgoState(0.0, 0.5, 0.9, 2.0)
    trace = simulate(path, ego, config, steps=200)
    settled = trace[140:]
    assert max(abs(s.y) for s in settled) < 0.2


def test_path_jitter_does_not_thrash_steering():
    """Real-mode local paths jitter laterally between replans; the commanded
    steering must stay calm (bounded activity), not slam the actuator."""
    import random
    config = MpcConfig()
    rng = random.Random(7)

    def jittered(state):
        noise = [(rng.uniform(-0.1, 0.1), rng.uniform(-0.1, 0.1)) for _ in range(60)]
        return [PathPoint(float(x) + noise[x][0], noise[x][1], 4.0)
                for x in range(60)]

    plant = Plant(EgoState(0.0, 0.0, 0.0, 3.0), config)
    previous = None
    estimated = 0.0
    trim = 0.0
    commands = []
    dt = 0.05
    for _ in range(200):
        solution = solve(jittered(plant.state), plant.state, config,
                         previous, current_steering=estimated)
        assert solution is not None
        trim *= max(0.0, 1.0 - 0.3 * dt)
        if abs(solution.lateral_error_m) < 0.5:
            trim -= 0.12 * solution.lateral_error_m * dt
        trim = max(-0.08, min(0.08, trim))
        commanded = max(-config.max_steering_rad,
                        min(config.max_steering_rad, solution.steering_rad + trim))
        commands.append(commanded)
        plant.step(commanded, solution.acceleration_mps2, dt)
        limit = config.max_steering_rate_radps * dt
        difference = commanded - estimated
        new_estimated = estimated + max(-limit, min(limit, difference))
        rate = (new_estimated - estimated) / dt
        estimated = new_estimated
        previous = (rate, solution.acceleration_mps2)
    activity = sum(abs(b - a) for a, b in zip(commands, commands[1:])) / len(commands)
    # Commanded steering-rate demand must stay within what the actuator can
    # actually do; thrash shows up as multiples of the slew limit.
    assert activity / dt < 0.8 * config.max_steering_rate_radps,         f"steering activity {activity / dt:.2f} rad/s vs slew 1.04"
    assert max(abs(s.y) for s in [plant.state]) < 1.0


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-v"]))
