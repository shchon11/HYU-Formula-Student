# hyu_lite_sim — the FSK stack's simulator without Gazebo

`eufs_sim` needs Gazebo, and there is no arm64 Gazebo for the Jetson. This
package is the simulator that *does* run there: a bicycle-model car, an
emulated **ECU**, an emulated **SBG Ellipse-D** and an emulated **perception
output**, all on the wall clock, so that steps 2 and 3 of the run flow
(`stack`, `mission …`) run **unchanged** on top — the same `sbg_raw_ekf`,
`graph_slam`, planners, Pure Pursuit, `vehicle_state.py` and
`drive_udp_bridge` the car uses.

```
sim small_track lite            # step 1  (or just 'sim small_track' on the Jetson: auto-picks lite)
stack                           # step 2  sbg_raw_ekf + SLAM + planning + control (standby)
mission trackdrive 10           # step 3  arm -> (AS button is latched ON) -> the car drives
sim attach | mission status | mission reset | sim stop
```

## What is emulated, and how faithfully

| Real thing | Emulation | Fitted to |
|---|---|---|
| Car | kinematic bicycle at the rear axle, steering slew/lag, ECU speed loop (P, accel/brake limits, drivetrain lag), `base_footprint` 0.91 m behind the axle (camera under the hoop), steering lock 0.335 rad (wheel ±90°) | `vehicle_mount.yaml` `vehicle:` (read by the launch when hyu_sensor_bringup is built; else `lite_sim.yaml`) |
| ECU / Speedgoat | **UDP**: listens for the bridge's `<ffBB` command datagrams on `127.0.0.1:15000`, drives only while `enable` **and** `autonomous_enable` are 1 and packets are fresh; converts the steering-wheel angle back to the road-wheel angle through the bridge's own `steering_kinematics.csv`; sends four-wheel RPM (`<ffff`, FL FR RL RR) to `127.0.0.1:15001` at 100 Hz with per-wheel radius scale, noise and optional quantisation | `drive_udp_bridge` protocol / yaml |
| SBG Ellipse-D | `/sbg/imu_data` 25 Hz (device dt exactly 40 000 µs, uint32 µs clock that wraps, mount tilt → accel.x −1.27 at rest, bias + noise + speed-proportional vibration), `/sbg/gps_pos` `/gps_vel` `/gps_hdt` 5 Hz epochs delivered **105 ms late** (header = arrival time, `time_stamp` = epoch), RTK-fixed noise/accuracy/status fields, dual-antenna heading +180° (secondary behind), 3.9 % HDT drop-outs, NED conventions, antenna lever arm (1.25 m ahead of base) | `bag/0801_sensors/…17_22_34` statistics |
| Perception | `/perception/cones` (`base_footprint`) 10 Hz + 80 ms: range/FOV envelope, detection probability falling with range, colour only inside the camera FOV and ≤ 12 m with a range-dependent probability, blue/yellow confusion, lateral/longitudinal noise + deskew residual + rare outliers, covariance tiers (0.07 camera+LiDAR, 0.23 LiDAR-only, 0.26²/0.36² sparse), transient false positives | `perception.yaml`, 2026-08 evaluations |
| **Off-track clutter** | poles / bushes / fences / people / parked cars / stray cones scattered outside the corridor (never inside it), each with its own detection probability, jitter and, for fences/cars, several returns; published as **unknown_color_cones** so SLAM maps them exactly as the real course's fixed objects were mapped | user requirement (unknown landmarks on the 0801 maps) |
| AS / AMI state | **not emulated** — the car's own `vehicle_state.py` runs (button latched ON by default) | — |
| Camera / LiDAR | not emulated (no images, no clouds); `hyu_perception` is not run | — |

Ground truth, in the world frame: `/ground_truth/state` (CarState),
`/ground_truth/track`, `/ground_truth/clutter`, `/ground_truth/cones`
(visible, noise-free, `base_footprint`) and RViz markers `/sim/debug/world`,
`/sim/debug/car` (green = ECU enabled), `/sim/debug/live_cones`.

### Frames

The simulator's world frame is the ENU tangent plane at `datum_latitude/
longitude`. Step 2 starts `sbg_raw_ekf` with **the same datum** and the same
`antenna_offset_x/y`, so the EKF's output frame — the `odom` frame that
`graph_slam` publishes `odom -> base_footprint` in — *is* the simulator world
frame (to sensor noise). Ground truth is therefore published with
`frame_id: odom`, and the RViz config uses `odom` as fixed frame. `map` is
SLAM's; `map -> odom` starts at identity.

## Running it by hand

```
ros2 launch hyu_lite_sim lite_sim.launch.py track:=trackdrive_kase2026 clutter_count:=120 clutter_seed:=3
ros2 launch hyu_lite_sim lite_sim.launch.py ecu:=ros                 # no bridge: /vehicle/cmd direct
ros2 launch hyu_lite_sim lite_sim.launch.py ekf:=true rviz:=true bridge_require_map_reset:=false
                                                                   # standalone, without step 2
ros2 launch hyu_lite_sim lite_sim.launch.py fix_schedule:=90:rtk_float,100:rtk_fixed,150:outage,153:rtk_fixed
ros2 run hyu_lite_sim clutter_tool small_track --count 80 --seed 3 --out ~/fsk/data/clutter_small_3.yaml
ros2 launch hyu_lite_sim lite_sim.launch.py clutter_file:=~/fsk/data/clutter_small_3.yaml
```

Services: `/vehicle/reset_vehicle_pos` (teleport to the start line — what
`mission reset` calls; the INS must be restarted afterwards, which
`mission reset` also does) and `/sim/reset_clutter` (new population, next
seed). `/sim/status` prints one line per second (ECU packets, enable,
command, speed, pose, fix).

All model constants live in `config/lite_sim.yaml` (`params_file:=` to
swap). `config/drive_udp_bridge_sim.yaml` is the bridge on loopback — it is
what keeps a simulator run from ever reaching the real ECU even with the
car's Ethernet plugged in.

### The `sim` script's lite knobs

```
sim <track> lite [bg] [rviz] [clutter:=N] [seed:=N] [clutter_file:=f] [ecu:=udp|ros]
                 [button:=auto|manual] [fix:=SCHED] [datum_lat:= datum_lon:= antenna_x:= antenna_y:=]
                 [any other name:=value for lite_sim.launch.py]
```
`button:=manual` makes the flow identical to the vehicle: arm, then
`mission go` (the bridge resets the SLAM map and raises the enable byte),
`mission halt` to stop.

## Limits (on purpose)

* Kinematic car: no tyre slip, no load transfer. Good for the pipeline
  (timing, frames, gating, mapping, planning, control loop closure), not for
  tuning vehicle dynamics.
* Perception is a statistical model, not a sensor model: it will not
  reproduce a specific detector's failure modes. Tune
  `perception.p_colour_*` / `max_range_m` to match a given detector.
* Wall clock only (`use_sim_time` false everywhere), no real-time-factor
  games: on a loaded Jetson the node simply integrates with the measured dt.

## Tests

```
cd src/sim/hyu_lite_sim && python3 -m pytest test
```

## Regression runs (headless A/B)

`tools/lite_regression.sh LABEL LAPS [planning launch args…]` drives the real
3-step flow unattended (`sim small_track lite bg` → `stack` → `mission trackdrive
LAPS`), records a bag (`/ground_truth/state`, `ego_odom`, `/planning/cte`,
`/planning/path`, `/vehicle/cmd`, lap/state…), keeps the freeze-time map CSV and
the graph_slam / Pure Pursuit logs under `$FSK_REG_OUT/LABEL` (default
`~/fsk/runs/lite/LABEL`) and tears the session down. It kills any existing
`fsk_race` session first — do not run it next to a live session.
`tools/analyze_run.py DIR [track.csv]` prints per-lap corridor position of the
ground-truth rear axle (0 = centre, ±1 = on a cone), outside-track fraction,
minimum cone clearance, `/planning/cte` RMS/p95, steering saturation (PP lock
0.52 / bridge 90° clip 0.335) and freeze-map completeness vs the track CSV.

```bash
# fixed noise so A/B runs are comparable (noise_seed 0 = different every run)
sed 's/^    noise_seed: 0.*/    noise_seed: 1/' src/sim/hyu_lite_sim/config/lite_sim.yaml > /tmp/lite_seed1.yaml
FSK_REG_SIM_ARGS="params_file:=/tmp/lite_seed1.yaml clutter_seed:=1" \
  src/sim/hyu_lite_sim/tools/lite_regression.sh s1_base 3 controller_rear_axle_from_base_m:=0.0
FSK_REG_SIM_ARGS="params_file:=/tmp/lite_seed1.yaml clutter_seed:=1" \
  src/sim/hyu_lite_sim/tools/lite_regression.sh s1_axle 3
python3 src/sim/hyu_lite_sim/tools/analyze_run.py ~/fsk/runs/lite/s1_axle
```
