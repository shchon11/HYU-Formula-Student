# Pure Pursuit controller

The node publishes `/vehicle/cmd` at 20 Hz from the selected `/planning/path` and
`/localization/ego_odom`. It brakes exactly with speed `0`, acceleration `-5`,
and steering `0` unless every required input has been received and is valid.

Freshness is measured from steady-clock receive time. Path, selected validity,
odometry, and stop-request inputs all use `input_timeout_sec` (default `0.5 s`);
an age equal to the timeout is fresh, while a greater age is stale. A missing or
stale stop request fails safe to braking even when its last value was false.

## Stopping

There are two kinds of stop, and they steer differently.

The **fail-safe brake** is taken when the path cannot be trusted — missing,
stale, invalid, wrong frame, or no reachable target. It commands speed `0`,
`brake_acceleration_mps2`, and steering `0`: with nothing trustworthy to aim at,
straight is the only defensible answer. It exposes no lookahead target.

A **planned stop** (`/planning/stop_request` asserted over a path that is still
valid and fresh — mission complete, stop line) instead brakes *along the path*:
speed `0` and `brake_acceleration_mps2`, but steering still tracked from the
path. The car travels `v^2 / (2a)` regardless of what is commanded — about 10 m
from 10 m/s at 5 m/s² — so zeroing the steering sends those metres straight ahead
whatever the track does, which takes the car off a curving circuit at the end of
the lap. The path is valid and continues past the finish (it is a closed loop and
the publisher wraps around it), so there is always something to follow while the
speed comes off.

## Steering modes

`steering_mode` selects the lateral law. Longitudinal control (target speed from
the lookahead point, `longitudinal_kp` on the speed error, and every fail-safe
above) is identical in both modes.

### `geometric` (default)

Kinematic pure pursuit with a fixed `lookahead_m`:
`delta = atan2(2 * wheelbase * y_body, L1^2)`.

### `map` — Model- and Acceleration-based Pursuit (ETH-PBL)

A model-based steering law that accounts for the speed-dependent understeer the
kinematic law ignores ([ETH-PBL/MAP-Controller](https://github.com/ETH-PBL/MAP-Controller),
ICRA 2023). Each cycle:

1. **Adaptive lookahead** from the nearest waypoint's planned speed:
   `L_d = clamp(map_lookahead_intercept_m + map_lookahead_slope_s * v, min, max)`.
2. **L1 guidance** to the lookahead point gives the required lateral acceleration:
   `a_lat = 2 * v^2 * sin(eta) / L_d`, where `sin(eta) = y_body / L1`.
3. **Steering lookup** converts `(a_lat, v)` to a steering angle through a
   single-track + tyre model table, `delta = LUT(a_lat, v)`.

`map_speed_source` chooses the speed `v`: `planned` (default, feed-forward off
the path speed — commands sensibly near standstill) or `measured` (odometry
speed — exact when tracking lags). A null or invalid table fails safe to braking.

MAP is the **default trackdrive controller** (`hyu_pure_pursuit.yaml`, what
`race` loads). The previous kinematic tune is kept as
`hyu_pure_pursuit_geometric.yaml`; select it with
`controller_params_file:=<share>/config/hyu_pure_pursuit_geometric.yaml`.
The skidpad and acceleration missions still use their geometric configs.

#### The steering lookup table

The table is integrated once at node startup (no external files): the reduced
single-track lateral dynamics are stepped to steady state across a grid of
`(steer, velocity)`, storing `|a_lat| = |r_ss| * v`; a cell is marked saturated
once the yaw rate fails to settle. `lookup` snaps to the nearest velocity column
and interpolates the steering that produces the requested `a_lat`, clamping to
the reachable range at the tyre limit. `tire_model` is `pacejka`
(`F_y = mu * F_z * D * sin(C * atan(B*(1-E)*a + E*atan(B*a)))`, matching the EUFS
`DynamicBicycle` plant) or `linear` (`F_y = mu * F_z * C_S * a`).

Explicit RK4 is stiffness-limited, so `lut_sim_substeps` sub-steps each 0.01 s
interval (>= ~8 keeps full-scale tyres stable at low speed). The build is
validated against the reference F1TENTH `SIM_linear` table in
`test/test_steering_lookup.cpp`.

The `hyu_pure_pursuit.yaml` values describe the EUFS `eufs` car
(`eufs_racecar/robots/eufs/configDry.yaml`). Note `pacejka_c_*` takes the
**magnitude** of the plant's shape factor (the plant ships `C = -1.38` under an
inverted slip-sign convention; the table stores `|a_lat|`, so only the magnitude
matters).
