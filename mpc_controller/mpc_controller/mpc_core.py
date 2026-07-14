"""Linear time-varying MPC for path tracking on a kinematic bicycle.

Pure numpy + quadprog; no ROS imports, so the closed-loop behaviour is unit
testable. The node feeds the selector path (map frame) and the ego state; the
solver returns the first control of the optimal sequence.

Formulation
-----------
The simulator's steering actuator is SLEW-LIMITED (steeringLockTime: full
lock-to-lock in ~1 s => ~1.04 rad/s). Commanding steering positions as if
they applied instantly turns that lag into a limit-cycle weave, so the
steering angle is part of the STATE and the control is its rate:

State x = [X, Y, psi, v, delta], control u = [ddelta, a], forward Euler at dt:

    X+     = X + v cos(psi) dt
    Y+     = Y + v sin(psi) dt
    psi+   = psi + v tan(delta) / L dt
    v+     = v + a dt
    delta+ = delta + ddelta dt

A reference trajectory is rolled along the path from the nearest waypoint at
the path's own speed profile; the model is linearised about it, states are
condensed out, and the dense QP over the control sequence solved by quadprog:

    min  sum_k |x_k - x_ref,k|_Q^2 + |u_k - u_ref,k|_R^2 + |u_k - u_{k-1}|_S^2
    s.t. |ddelta_k| <= max_steering_rate,  a_min <= a_k <= a_max,
         |delta_k| <= max_steering        (as condensed state constraints)

The command sent to the car is the PLANNED steering after one step; since the
actuator slews toward its target at the full rate, a reachable planned angle
is reproduced exactly.
"""

from dataclasses import dataclass, field
from math import sqrt as math_sqrt
from typing import List, Optional, Sequence, Tuple

import numpy as np
import quadprog

NX = 5
NU = 2


@dataclass
class MpcConfig:
    wheelbase_m: float = 1.58
    horizon_steps: int = 12
    horizon_dt_sec: float = 0.1
    max_steering_rad: float = 0.52
    # Simulator actuator: (0.52 - (-0.52)) / steeringLockTime(1 s) = 1.04.
    max_steering_rate_radps: float = 1.04
    max_speed_mps: float = 10.0
    min_acceleration_mps2: float = -8.0
    max_acceleration_mps2: float = 2.5
    # Position error is weighted in the PATH frame: cross-track error is what
    # tracking is about; along-track error is cheap (the reference index
    # slides), so weighting them equally lets the optimiser trade lateral
    # offset against longitudinal slack and park on a concentric circle.
    q_lateral: float = 8.0
    q_longitudinal: float = 0.5
    q_heading: float = 6.0
    q_speed: float = 1.0
    q_steering: float = 0.1
    r_steering_rate: float = 1.0
    r_acceleration: float = 0.5
    s_steering_rate: float = 2.0
    s_acceleration_rate: float = 1.0
    # Receding-horizon plans may keep DEFERRING a correction (each cycle only
    # the first control runs); weighting the terminal state makes the plan
    # commit early.
    terminal_weight: float = 8.0
    command_rate_hz: float = 20.0
    input_timeout_sec: float = 0.5
    # Compensate command latency: the ego is rolled forward by this much
    # before solving, so the plan starts where the car will be when the
    # command takes effect.
    actuation_delay_sec: float = 0.05


@dataclass
class PathPoint:
    x: float
    y: float
    speed: float


@dataclass
class EgoState:
    x: float
    y: float
    yaw: float
    speed: float


@dataclass
class MpcSolution:
    steering_rad: float
    acceleration_mps2: float
    speed_mps: float
    # Signed cross-track error of the (delay-compensated) ego in the path
    # frame, +left of path. Callers may integrate it into a steering trim.
    lateral_error_m: float = 0.0
    predicted_xy: List[Tuple[float, float]] = field(default_factory=list)


def wrap_angle(angle: float) -> float:
    return float(np.arctan2(np.sin(angle), np.cos(angle)))


def valid_config(config: MpcConfig) -> bool:
    return (
        np.isfinite(config.wheelbase_m) and config.wheelbase_m > 0.0 and
        config.horizon_steps >= 2 and
        np.isfinite(config.horizon_dt_sec) and config.horizon_dt_sec > 0.0 and
        np.isfinite(config.max_steering_rad) and config.max_steering_rad > 0.0 and
        np.isfinite(config.max_steering_rate_radps) and
        config.max_steering_rate_radps > 0.0 and
        np.isfinite(config.max_speed_mps) and config.max_speed_mps > 0.0 and
        config.min_acceleration_mps2 < 0.0 < config.max_acceleration_mps2 and
        config.command_rate_hz > 0.0
    )


def _nearest_index(path: Sequence[PathPoint], ego: EgoState) -> Optional[int]:
    best = None
    best_d2 = float("inf")
    for index, point in enumerate(path):
        if not (np.isfinite(point.x) and np.isfinite(point.y)):
            return None
        d2 = (point.x - ego.x) ** 2 + (point.y - ego.y) ** 2
        if d2 < best_d2:
            best_d2 = d2
            best = index
    return best


def _reference(
    path: Sequence[PathPoint], start: int, config: MpcConfig
) -> Optional[Tuple[np.ndarray, np.ndarray]]:
    """March along the path at its speed profile: N+1 states, N controls."""
    xy = np.array([[p.x, p.y] for p in path], dtype=float)
    speeds = np.array(
        [min(max(p.speed, 0.0), config.max_speed_mps) for p in path], dtype=float)
    segments = np.diff(xy, axis=0)
    lengths = np.hypot(segments[:, 0], segments[:, 1])
    if np.any(~np.isfinite(lengths)):
        return None

    cumulative = np.concatenate(([0.0], np.cumsum(lengths)))
    s0 = cumulative[start]
    total = cumulative[-1]

    def sample(s: float) -> Tuple[float, float, float, float]:
        s = min(max(s, 0.0), total)
        seg = int(np.searchsorted(cumulative, s, side="right")) - 1
        seg = min(max(seg, 0), len(lengths) - 1)
        t = 0.0 if lengths[seg] <= 1e-9 else (s - cumulative[seg]) / lengths[seg]
        px, py = xy[seg] + t * segments[seg]
        heading = float(np.arctan2(segments[seg, 1], segments[seg, 0]))
        speed = float(speeds[seg] + t * (speeds[min(seg + 1, len(speeds) - 1)] - speeds[seg]))
        return float(px), float(py), heading, speed

    states = np.zeros((config.horizon_steps + 1, NX))
    s = s0
    px, py, heading, speed = sample(s)
    prev_heading = heading
    states[0, 0:4] = (px, py, heading, max(speed, 0.3))
    for k in range(1, config.horizon_steps + 1):
        s += max(states[k - 1, 3], 0.3) * config.horizon_dt_sec
        px, py, heading, speed = sample(s)
        # Keep the reference heading continuous so heading errors never wrap.
        heading = prev_heading + wrap_angle(heading - prev_heading)
        prev_heading = heading
        states[k, 0:4] = (px, py, heading, max(speed, 0.0))
        if s >= total:
            states[k, 3] = 0.0  # end of known path: reference comes to rest

    # Back-propagate braking so the reference slows AHEAD of a stop.
    comfort_brake = 0.7 * abs(config.min_acceleration_mps2)
    for k in range(config.horizon_steps - 1, -1, -1):
        ds = float(np.hypot(
            states[k + 1, 0] - states[k, 0], states[k + 1, 1] - states[k, 1]))
        allowed = math_sqrt(states[k + 1, 3] ** 2 + 2.0 * comfort_brake * ds)
        states[k, 3] = min(states[k, 3], allowed)

    # Reference steering angle from the path yaw rate, then its rate.
    for k in range(config.horizon_steps):
        yaw_rate = (states[k + 1, 2] - states[k, 2]) / config.horizon_dt_sec
        v = max(states[k, 3], 0.3)
        states[k, 4] = float(np.clip(
            np.arctan(config.wheelbase_m * yaw_rate / v),
            -config.max_steering_rad, config.max_steering_rad))
    states[config.horizon_steps, 4] = states[config.horizon_steps - 1, 4]

    controls = np.zeros((config.horizon_steps, NU))
    for k in range(config.horizon_steps):
        controls[k, 0] = float(np.clip(
            (states[k + 1, 4] - states[k, 4]) / config.horizon_dt_sec,
            -config.max_steering_rate_radps, config.max_steering_rate_radps))
        controls[k, 1] = float(np.clip(
            (states[k + 1, 3] - states[k, 3]) / config.horizon_dt_sec,
            config.min_acceleration_mps2, config.max_acceleration_mps2))
    return states, controls


def solve(
    path: Sequence[PathPoint],
    ego: EgoState,
    config: MpcConfig,
    previous_control: Optional[Tuple[float, float]] = None,
    current_steering: float = 0.0,
) -> Optional[MpcSolution]:
    """previous_control is (steering_rate, acceleration) of the last cycle;
    current_steering is the estimated ACTUAL steering angle of the car."""
    if not valid_config(config) or len(path) < 2:
        return None
    if not all(np.isfinite(v) for v in (ego.x, ego.y, ego.yaw, ego.speed)):
        return None
    if not np.isfinite(current_steering):
        return None
    current_steering = float(np.clip(
        current_steering, -config.max_steering_rad, config.max_steering_rad))

    if config.actuation_delay_sec > 0.0:
        delay = config.actuation_delay_sec
        speed = max(ego.speed, 0.0)
        ego = EgoState(
            ego.x + speed * np.cos(ego.yaw) * delay,
            ego.y + speed * np.sin(ego.yaw) * delay,
            ego.yaw + speed * np.tan(current_steering) / config.wheelbase_m * delay,
            speed,
        )

    nearest = _nearest_index(path, ego)
    if nearest is None:
        return None
    reference = _reference(path, nearest, config)
    if reference is None:
        return None
    ref_states, ref_controls = reference

    horizon = config.horizon_steps
    dt = config.horizon_dt_sec
    wheelbase = config.wheelbase_m

    x_err0 = np.array([
        ego.x - ref_states[0, 0],
        ego.y - ref_states[0, 1],
        wrap_angle(ego.yaw - ref_states[0, 2]),
        max(ego.speed, 0.0) - ref_states[0, 3],
        current_steering - ref_states[0, 4],
    ])

    a_seq = np.zeros((horizon, NX, NX))
    b_seq = np.zeros((horizon, NX, NU))
    for k in range(horizon):
        psi = ref_states[k, 2]
        v = max(ref_states[k, 3], 0.3)
        delta = ref_states[k, 4]
        a_k = np.eye(NX)
        a_k[0, 2] = -v * np.sin(psi) * dt
        a_k[0, 3] = np.cos(psi) * dt
        a_k[1, 2] = v * np.cos(psi) * dt
        a_k[1, 3] = np.sin(psi) * dt
        a_k[2, 3] = np.tan(delta) / wheelbase * dt
        a_k[2, 4] = v / (wheelbase * np.cos(delta) ** 2) * dt
        b_k = np.zeros((NX, NU))
        b_k[4, 0] = dt
        b_k[3, 1] = dt
        a_seq[k] = a_k
        b_seq[k] = b_k

    n_u = NU * horizon
    phi = np.zeros((horizon, NX, NX))
    gamma = np.zeros((horizon, NX, n_u))
    running = np.eye(NX)
    for k in range(horizon):
        running = a_seq[k] @ running
        phi[k] = running
        for j in range(k + 1):
            block = b_seq[j]
            for m in range(j + 1, k + 1):
                block = a_seq[m] @ block
            gamma[k][:, NU * j:NU * j + NU] = block

    r_diag = np.array([config.r_steering_rate, config.r_acceleration])
    s_diag = np.array([config.s_steering_rate, config.s_acceleration_rate])

    hessian = np.zeros((n_u, n_u))
    gradient = np.zeros(n_u)
    for k in range(horizon):
        psi_ref = ref_states[k + 1, 2]
        cos_p, sin_p = np.cos(psi_ref), np.sin(psi_ref)
        rotation = np.array([[cos_p, -sin_p], [sin_p, cos_p]])
        q_k = np.zeros((NX, NX))
        q_k[0:2, 0:2] = rotation @ np.diag(
            [config.q_longitudinal, config.q_lateral]) @ rotation.T
        q_k[2, 2] = config.q_heading
        q_k[3, 3] = config.q_speed
        q_k[4, 4] = config.q_steering
        if k == horizon - 1:
            q_k *= config.terminal_weight
        gq = q_k @ gamma[k]
        hessian += gamma[k].T @ gq
        gradient += (phi[k] @ x_err0) @ gq

    for k in range(horizon):
        sl = slice(NU * k, NU * k + NU)
        hessian[sl, sl] += np.diag(r_diag)

    prev = np.zeros(NU)
    if previous_control is not None:
        prev = np.array(previous_control, dtype=float) - ref_controls[0]
    for k in range(horizon):
        sl = slice(NU * k, NU * k + NU)
        ref_step = ref_controls[k] - ref_controls[k - 1] if k > 0 else np.zeros(NU)
        if k == 0:
            hessian[sl, sl] += np.diag(s_diag)
            gradient[sl] += -s_diag * (prev - ref_step)
        else:
            sp = slice(NU * (k - 1), NU * (k - 1) + NU)
            hessian[sl, sl] += np.diag(s_diag)
            hessian[sp, sp] += np.diag(s_diag)
            hessian[sl, sp] -= np.diag(s_diag)
            hessian[sp, sl] -= np.diag(s_diag)
            gradient[sl] += s_diag * ref_step
            gradient[sp] -= s_diag * ref_step

    hessian += 1e-6 * np.eye(n_u)

    # Control box constraints (C^T u_err >= b) ...
    constraints = []
    bounds = []
    for k in range(horizon):
        for axis, low, high in (
            (0, -config.max_steering_rate_radps, config.max_steering_rate_radps),
            (1, config.min_acceleration_mps2, config.max_acceleration_mps2),
        ):
            row = np.zeros(n_u)
            row[NU * k + axis] = 1.0
            constraints.append(row)
            bounds.append(low - ref_controls[k, axis])
            row = np.zeros(n_u)
            row[NU * k + axis] = -1.0
            constraints.append(row)
            bounds.append(-(high - ref_controls[k, axis]))
    # ... plus steering ANGLE limits on the predicted states:
    # delta_k+1 = ref_delta_k+1 + (phi_k x0 + gamma_k u)[4] within +-max.
    for k in range(horizon):
        base = float(ref_states[k + 1, 4] + phi[k][4] @ x_err0)
        row = gamma[k][4]
        constraints.append(row.copy())
        bounds.append(-config.max_steering_rad - base)
        constraints.append(-row)
        bounds.append(-(config.max_steering_rad - base))
    c_matrix = np.array(constraints).T
    b_vector = np.array(bounds)

    try:
        u_err = quadprog.solve_qp(
            hessian, -gradient, c_matrix, b_vector, meq=0)[0]
    except ValueError:
        return None
    if not np.all(np.isfinite(u_err)):
        return None

    # Command the planned steering after one step; the slew-limited actuator
    # tracks a reachable target exactly.
    err1 = phi[0] @ x_err0 + gamma[0] @ u_err
    steering = float(np.clip(
        ref_states[1, 4] + err1[4],
        -config.max_steering_rad, config.max_steering_rad))
    u0 = ref_controls[0] + u_err[0:NU]
    acceleration = float(np.clip(
        u0[1], config.min_acceleration_mps2, config.max_acceleration_mps2))
    speed = float(np.clip(ref_states[1, 3], 0.0, config.max_speed_mps))

    psi0 = ref_states[0, 2]
    lateral_error = float(-np.sin(psi0) * x_err0[0] + np.cos(psi0) * x_err0[1])

    predicted = []
    for k in range(horizon):
        err = phi[k] @ x_err0 + gamma[k] @ u_err
        predicted.append((float(ref_states[k + 1, 0] + err[0]),
                          float(ref_states[k + 1, 1] + err[1])))
    return MpcSolution(steering, acceleration, speed, lateral_error, predicted)
