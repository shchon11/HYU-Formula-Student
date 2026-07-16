# Introduction
This repository contains algorithms that allow us to determine an optimal racing line on a race track. You can chose
between several objectives:
* Shortest path
* Minimum curvature (with or without iterative call)

The minimum curvature line is quite near to a minimum time line in corners but will differ as soon as the car's
acceleration limits are not exploited. Please look into the `main_globaltraj.py` for all possible options.

# List of components
* `helper_funcs_glob`: This package contains some helper functions used in several other functions when 
calculating the global race trajectory.
* `inputs`: This folder contains the vehicle dynamics information (`veh_dyn_info/ggv.csv`,
`veh_dyn_info/ax_max_machines.csv`).
* `params`: This folder contains a parameter file with optimization and vehicle parameters.

# Trajectory Planning Helpers repository
Lots of the required functions for trajectory planning are cumulated in our trajectory planning helpers repository. It
can be found on https://github.com/TUMFTM/trajectory_planning_helpers. They can be quite useful for other projects as
well.

Note (HYU): version 0.79 of that package is **vendored** in
`trajectory_planning_helpers/` inside this directory, with a small patch for
modern scipy. Do not `pip install trajectory-planning-helpers` — see
`trajectory_planning_helpers/VENDORED.md`.

# Dependencies
Runs on the stock Ubuntu 22.04 / ROS 2 Humble `python3` (3.10). numpy, scipy,
matplotlib, pyyaml and OpenCV (incl. `cv2.ximgproc`) are already present
system-wide there; the only pip package usually missing is quadprog:\
`python3 -m pip install --user quadprog`

See `requirements.txt` for the details and the do-not-install warnings.

### Solutions for possible installation problems (Windows)
* `cvxpy`, `cython` or any other package requires a `Visual C++ compiler` -> Download the build tools for Visual Studio
2019 (https://visualstudio.microsoft.com/de/downloads/ -> tools for Visual Studio 2019 -> build tools), install them and
chose the `C++ build tools` option to install the required C++ compiler and its dependencies
* Problems with quadprog -> reason currently not clear, test using quadprog in version 0.1.6 instead 0.1.7

### Solutions for possible installation problems (Ubuntu)
* `matplotlib` requires `tkinter` -> can be solved by `sudo apt install python3-tk`
* `Python.h` required `quadprog` -> can be solved by `sudo apt install python3-dev`

# Running the code
* `Step 1:` (optional) Adjust the parameter file that can be found in the `params` folder (required file).
* `Step 2:` (optional) Adjust the ggv diagram and ax_max_machines file in `inputs/veh_dyn_info` (if used). This
acceleration should be calculated without drag resistance, i.e. simply by F_x_drivetrain / m_veh!
* `Step 3:` Generate `outputs/<map_name>/centerline.csv` (see the map CSV pipeline section below).
* `Step 4:` Adjust the parameters in the upper part of `main_globaltraj.py` and execute it to start the trajectory 
generation process. The calculated race trajectory is stored in `outputs/<map_name>/traj_race_cl.csv`.

# Wording and conventions
We tried to keep a consistant wording for the variable names:
* path -> [x, y] Describes any array containing x,y coordinates of points (i.e. point coordinates).\
* refline -> [x, y] A path that is used as reference line during our calculations.\
* reftrack -> [x, y, w_tr_right, w_tr_left] An array that contains not only the reference line information but also
right and left track widths. In our case it contains the race track that is used as a basis for the raceline
optimization.

Our normal vectors usually point to the right in the direction of driving. Therefore, we get the track boundaries by
multiplication as follows: norm_vector * w_tr_right, -norm_vector * w_tr_left.

# Trajectory definition
### Race Trajectory
The output csv contains the global race trajectory. The array is of size [no_points x 7] where no_points depends on
stepsize and track length. The seven columns are structured as follows:

* `s_m`: float32, meter. Curvi-linear distance along the raceline.
* `x_m`: float32, meter. X-coordinate of raceline point.
* `y_m`: float32, meter. Y-coordinate of raceline point.
* `psi_rad`: float32, rad. Heading of raceline in current point from -pi to +pi rad. Zero is north (along y-axis).
* `kappa_radpm`: float32, rad/meter. Curvature of raceline in current point.
* `vx_mps`: float32, meter/second. Target velocity in current point.
* `ax_mps2`: float32, meter/second². Target acceleration in current point. We assume this acceleration to be constant
  from current point until next point.

# References
* Minimum Curvature Trajectory Planning\
Heilmeier, Wischnewski, Hermansdorfer, Betz, Lienkamp, Lohmann\
Minimum Curvature Trajectory Planning and Control for an Autonomous Racecar\
DOI: 10.1080/00423114.2019.1631455\
Contact person: [Alexander Heilmeier](mailto:alexander.heilmeier@tum.de).

# Map CSV -> global plan pipeline (HYU addition)

Standalone scripts (no ROS 2) that turn a centerline map CSV into the inputs
this generator needs, then run it end to end. All paths are relative to this
directory; results land in `outputs/<map_name>/`.

Environment: the stock system `python3` (see the Dependencies section above;
only quadprog needs a pip install). Headless by default -- no GUI windows, no
key presses.

One-shot run:

    cd planning/trajectory_generator
    python3 map_csv_to_icra_global_plan.py \
        --input-csv /path/to/my_track.csv \
        --map-name my_track \
        --resolution 0.1

Stepwise equivalents:

    python3 map_csv_to_boundaries.py \
        --input-csv /path/to/my_track.csv --map-name my_track
    python3 csv_to_track_mask.py \
        --left  outputs/my_track/left_boundary.csv \
        --right outputs/my_track/right_boundary.csv \
        --map-name my_track --resolution 0.1
    # set map_name / map_img_ext in config/params.yaml, then:
    python3 lane_generator.py --headless
    python3 main_globaltraj.py --headless

RViz debug boundaries (periodic-spline resample of the cone map, matching the
track mask exactly) for the global_planner debug visualizer node:

    python3 export_boundary_splines.py --map-name my_track

Outputs: `maps/<map>.png|.yaml`, `outputs/<map>/left|right_boundary.csv`,
`centerline.csv`, `lane_*.csv`, `traj_race_cl.csv`,
`left|right_boundary_spline.csv`.
Pass `--show-plots` (pipeline) or drop `--headless` (single scripts) to get
the original interactive previews.

# CLCS Frenet conversion accuracy (HYU validation)

종합 판정: 위치 변환 `(s, d)`는 서브 센티미터 수준으로 정확하다. 단,
속도 성분, 폐루프 시임, waypoint `s_m` 도메인 차이는 알고 사용해야 한다.

검증 방법:

* 기존 gtest 9개 실행: 직선, 역방향, 중복점, 퇴화 경로, NaN, 폐루프,
  재구성 테스트 모두 통과
* 해석해가 있는 원형 트랙(`R=10 m`, 628점)으로 독립 수치 검증
* 실제 트랙 데이터(`global_waypoints.json`, 99점, 35.7 m)로 round-trip 검증

검증 결과:

| 항목 | 측정 결과 |
| --- | --- |
| `s` 정확도, 원형 1000점 sweep | 최대 오차 8.1 mm, polyline 이산화 오차 포함 |
| `d` 정확도, 안쪽 `d=+1.5 m` / 바깥 `d=-2 m` sweep | 최대 오차 0.13 mm |
| 실제 트랙 waypoint round-trip | `max abs(s - s_m) = 0.0000 m`, `max abs(d) = 0.00000 m` |
| `d` 부호 규약 | 진행 방향 왼쪽이 양수, race_stack 규약과 동일 |
| 트랙 길이 | 상대 오차 `4e-6` |
| Frenet -> Cartesian 재구성 | 일반 구간 `1e-4` ~ `1e-8 m` |
| 폐루프 `s` wrap | 시임 통과 시 `L - 0.01 -> 0.01`로 정상 wrap |
| 견고성 | NaN 입력, 퇴화 경로, 중복점 제거, 자가교차 감지, 경로 버전 관리, mutex thread safety 구현 |
| 성능 | 실제 트랙 99점 기준 평균 248 us, 최대 345 us. 628점 원형 기준 평균 1.5 ms |

주의할 특성:

* `v_s`는 실제 `s_dot`가 아니라 centerline 접선 방향 속도 성분이다.
  `clcs_frenet_converter.cpp`의 속도 변환은 곡률 보정 계수
  `1 / (1 - kappa * d)`를 적용하지 않는다. 원형 `R=10 m`에서
  `d=+1.5 m`이면 실제 `s_dot` 대비 약 15% 작게 나온다. 이는
  race_stack Python 구현과 같은 규약이므로 기존 소비자와 호환되지만,
  `v_s`를 lap time 예측용 실제 `s_dot`로 직접 쓰면 안 된다.
* 폐루프 시임 바깥쪽(`d < 0`)에는 millimeter 폭의 projection 실패 영역이
  있다. 시임 반경선 위 바깥 점에서만 반대편 후보로 튀며, 측정된 실패 폭은
  `d=-0.5 m`에서 약 5 mm, `d=-2 m`에서 약 20 mm이다.
  `max_projection_distance` guard가 이를 `valid=false`로 잡아내고,
  기본 `drop_message` 정책에서는 해당 odom 1건만 버린다. 5 m/s, 40 Hz
  기준 sample 간격 12.5 cm보다 훨씬 좁아 실주행 영향은 작다. 필요하면
  `projection_failure_policy: publish_last_valid`로 완화할 수 있다.
* `s` 도메인이 waypoint `s_m`과 약 0.361 m 다르다. 실제 트랙 데이터에서
  마지막 waypoint와 첫 waypoint 사이 닫는 segment 길이가 0.361 m이고,
  CLCS가 발행하는 `s`는 이 닫는 segment를 포함한 폐루프 길이
  35.667 m 기준이다. 반면 일부 state machine 로직은
  `wpnts.back().s_m`인 35.306 m를 track length로 사용할 수 있다.
  시임 근처 `s` 비교와 wrap 계산에 최대 0.36 m 오차가 생길 수 있으므로,
  perception 및 obstacle logic 연결 시 track length 기준을 통일해야 한다.

부수 발견: 첫 waypoint(`s=0`) 정확히 그 지점에서는
`tangent_epsilon=0.05 m` clamp 때문에 자가검증용 재구성 오차만 5 cm로
보일 수 있다. 출력되는 `s/d` 자체는 정확하며, heading error도 시임에서
최대 약 0.3 deg 수준이고 그 외 구간에서는 정상이다.
