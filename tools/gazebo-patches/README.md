# Patched Gazebo Classic for the GPU LiDAR path

Why this exists: the CPU `ray` VLP-16 does 288k ray/collision tests per second
(1800×16 @ 10 Hz) **while holding the physics update mutex**, so the LiDAR
directly stalls the physics step and eats RTF. `gpu_ray` moves that work onto
the GPU — which idles at ~11 % while the OGRE render thread is CPU-bound (see
the measurement notes in `eufs_sensors/urdf/zed.urdf.xacro`) — but stock
`gpu_ray` degrades the scan in ways that hurt cone perception. This kit builds
a Gazebo 11.10.2 whose GPU laser is accurate enough to switch.

## How gpu_ray actually renders (gazebo11, our VLP-16 numbers)

`gpu_ray` is not raycasting. `GpuRaySensor` renders the scene into first-pass
depth textures and resamples them into beams:

- 360° hFOV → **3 internal cameras**, 120° each (`hfov > 5.6` → 3).
- First-pass texture per camera: width `max(2048, rangeCount/3)` = **2048**;
  height follows the camera aspect ratio ≈ `2048 / (tan60°/tan28.2°)` =
  **634** (the camera vFOV is keystone-padded to
  `2·atan(tan(15°)/cos(60°)) ≈ 56.4°` so the corners stay covered).
- Texel pitch at stock 2048: **0.059° azimuth / 0.089° elevation**. A second
  pass maps every beam's exact angles to a texel (nearest-neighbor,
  `TFO_NONE`) and reads the metric range the first pass wrote there
  (`laser_1st_pass.frag` stores `length(viewpos)` in R, float32 — no depth
  precision issue).
- The published cloud keeps the exact sample counts and beam angles — the
  texture is internal resolution only.

The error budget vs CPU ray is therefore texel-center snapping: the range is
sampled up to a half-texel away from the true beam direction. On surfaces at
grazing incidence (the ground at far rings) a 0.044° elevation snap is a
range error of `R·Δγ/tan(incidence)` — **~10 cm at the -5° ring, ~25 cm at
the -3° ring** — which shows up as radial banding/waviness in exactly the
points ground-removal consumes. Cone flanks (near-normal incidence) stay at
cm level.

## The patches (`patch_gazebo.py`, idempotent, tag gazebo11_11.10.2)

0. **`GpuLaser.cc` — hide SkyX during the laser render.** The headline fix.
   SkyX's dome and volumetric clouds are REAL scene geometry, so the depth
   pass returns them like a solid ceiling: measured on skidpad_kase2026,
   12,027 of 28,800 beams per scan came back as phantoms at 17-21 m, and the
   6 cones sitting at 17.9-21.6 m were occluded outright. A lidar gets no
   return from clouds. Upstream already hides the moon node for exactly this
   reason ("it clips gpu laser range values") but left dome and clouds
   visible; this completes it. The toggle is two cheap scene-node visibility
   flips per scan, and camera sensors still render the sky (single render
   thread, hide/restore brackets only the laser passes).
1. **`GpuRaySensor.cc` — `GAZEBO_GPU_LASER_TEX_MIN`** (default 2048 = stock,
   clamp [16, 16384]). Raising the floor shrinks BOTH texel pitches via the
   aspect-ratio branch; the GPU pays, the render thread's CPU cost (batch
   count) does not change. 4096 halves the snapping error for ~190 MB VRAM
   across the 3 faces; 8192 quarters it for ~750 MB. Each sensor logs its
   geometry so a patched server is recognizable:
   `[Msg] GpuRaySensor [...]: cameras=3 first-pass tex=4096x1268 (...)`.
2. **`gazebo.material` — no backface culling in the laser first pass.** A
   lidar return does not depend on triangle winding; with culling on, meshes
   with flipped normals vanish from the scan while looking fine on camera.
3. **`laser_2nd_pass.frag` — clamp UV instead of painting white.** Beams that
   land exactly on a 120° face boundary (indices 0/600/1200 at 1800 samples)
   can fall out of [0,1] by a float ulp; white decodes as a **phantom return
   at exactly 1.0 m** because the first pass stores metric range.

Not patched, by design: sample counts, angles, rates, noise — a real Puck
stays a real Puck (`VLP-16R.urdf.xacro` documents that contract).

## Build & use

```bash
bash tools/gazebo-patches/build-patched-gazebo.sh   # clone→patch→build→install
```

Defaults: source `~/gazebo-classic-fsk`, prefix `~/opt/gazebo11-fsk` (no
sudo), tag `gazebo11_11.10.2` — keep the tag equal to the installed gzserver
version so ros-humble-gazebo-* plugins stay ABI-compatible. DART physics is
disabled (jammy's libdart-dev is missing its ikfast headers; we only run ODE).

Activate for a sim run (prepend to the launch environment):

```bash
source ~/opt/gazebo11-fsk/share/gazebo/setup.sh
export PATH="$HOME/opt/gazebo11-fsk/bin:$PATH"
export LD_LIBRARY_PATH="$HOME/opt/gazebo11-fsk/lib:${LD_LIBRARY_PATH:-}"
export GAZEBO_GPU_LASER_TEX_MIN=4096   # accuracy knob; unset = stock geometry
```

Switch the sensor with the `gpu` param on the `VLP-16R` macro
(`eufs_sensors/urdf/VLP-16R.urdf.xacro`); the plugin
(`libgazebo_ros_ray_sensor.so`) is the same for both types.

## Verification

Measured on skidpad_kase2026, static car, sensors-only bringup (no
perception/planning), comparing per-beam median ranges over 30 frames against
the CPU `ray` reference — see "Measured results" below. Re-run with:

```bash
# capture & analysis scripts live in the session scratchpad; the method:
# 1. CPU ray run -> capture 30 frames of /velodyne_points + RTF from /clock
# 2. gpu_ray stock run, gpu_ray patched runs (TEX_MIN=2048/4096)
# 3. per-(ring,azimuth-bin) range deltas, cone cluster match, phantom counts
```

## Measured results (2026-07-16)

skidpad_kase2026, static car at spawn, sensors-only bringup (stereo ZED
2×1280×720@30 + VLP-16 1800×16@10, no perception/planning nodes), 30-frame
captures, RTF from /clock-vs-wall during the capture window. Single run per
config; quality metrics are per-beam medians over the 30 frames, so the 8 mm
gaussian noise floor is ~×(1/√30) per beam and reappears in beam-to-beam
comparisons as ~11 mm√-ish steps.

| config              |   RTF | pts/frame | notes                                    |
|---------------------|------:|----------:|------------------------------------------|
| CPU `ray` (baseline)| 0.350 |    16,772 | reference geometry                        |
| `gpu_ray` stock     | 0.896 |    28,800 | +12,027 sky-dome phantoms at 17-21 m; far cones occluded; ground \|dR\| p95 1.73 m (10-20 m band) |
| patched, 2048       | 0.955 |    16,761 | dome gone, far cones back                 |
| patched, 4096       | 0.961 |    16,759 | staircase halved                          |
| patched, 8192       | 0.973 |    16,756 | staircase at noise floor through -7°      |

Consecutive-beam range step p90 on ground rings — the "staircase" the eye
sees in RViz (CPU column = the sensor's own noise floor):

| ring | ground R | CPU  | 2048 | 4096 | 8192 |
|------|---------:|-----:|-----:|-----:|-----:|
| -15° |    2.1 m |  9.2 | 10.6 |  9.3 |  9.0 |
| -11° |    2.9 m | 10.0 | 15.2 | 11.8 | 10.1 |
| -7°  |    4.5 m | 11.4 | 27.5 | 20.3 | 14.1 |
| -5°  |    6.3 m | 13.8 | 40.5 | 29.4 | 21.1 |
| -3°  |   10.4 m | 20.3 | 54.3 | 55.1 | 38.3 |

(all mm; -3° barely responds until 8192 — grazing incidence is the method's
limit.)

Cones vs the CPU reference: matched clusters shift 6 mm median / 33 mm p95,
radial bias +4 mm, points-per-cone unchanged. The analysis clusterer reported
2 "missing" cones at 10.4 m; raw-point inspection shows the GPU cloud has
**more** returns there than CPU (14.8 vs 2.2 pts/frame) and the toy ground
gate ate them — no cone is actually lost.

Known residuals, measured and accepted:

- **Ground reads ~2 cm low** via gpu_ray (bias × tan(elev) ≈ 19-22 mm across
  all rings = a constant height offset; root cause not yet found — sensor
  origin vs frame lumping suspected). Radially that is 75-320 mm on grazing
  rings, but it is smooth, not staircase, and cone centroids sit at +4 mm.
- **Car-body self-returns differ**: the GPU sees visual meshes (~900 more
  points <2 m per frame than CPU's collision boxes). Crop them like on the
  real car.
- **-3° ring staircase ~2× noise floor at 8192** — the grazing-incidence
  limit of depth-texture resampling.
- RTF numbers are sensors-only and single-run; the full stack adds YOLO and
  planning load. The structural win stands regardless: the CPU lidar's
  physics-mutex contention is gone entirely.

## Rollback

Drop the three `export`/`source` lines from the launch environment (stock
`/usr/bin/gzserver` and stock media take over), and set `gpu:=false` on the
VLP-16R macro.
