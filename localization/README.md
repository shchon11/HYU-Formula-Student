# 🗺 Localization

**인지가 준 콘 관측과 차량 오도메트리로 트랙의 콘 지도(`cone_map`)와 내 위치(`ego_odom`)를 동시에 만듭니다.**
랩 1에서 지도를 만들고(mapping), 출발점으로 돌아오면 지도를 얼려 그 위에서 위치만 푸는(localization) 2단계 생애주기가 핵심입니다.

```mermaid
flowchart LR
    CONES["/perception/cones<br/>(base_footprint)"] --> GS
    ODOM["/localization/wheel_odom<br/>또는 /localization/ins_odom"] --> GS

    subgraph BRIDGE["INS/SBG 브리지 (실차 체인)"]
        SBG["🛰 /sbg/ekf_nav · ekf_euler"] --> BR["sbg_odometry_bridge"]
        WS["⚙️ /vehicle/wheel_speeds"] --> BR
        BR -->|"ins_odom (상대, 점프 없음)"| GS
    end

    GS["graph_slam<br/>g2o 포즈그래프 + CSM 보정"]
    GS ==>|"/localization/cone_map"| P1["planning"]
    GS ==>|"/localization/ego_odom"| P1
    GS ==>|"/localization/status"| P1
    GS -->|"map → odom → base_footprint"| TF["TF"]

    style GS fill:#1d4d33,stroke:#4fca7f,color:#e9fbef
```

## Input / Output

| 방향 | 토픽 | 타입 | 역할 |
|---|---|---|---|
| in | `/perception/cones` | `ConeArrayWithCovariance` | 콘 관측 (공분산 = 가중치) |
| in | `/localization/wheel_odom` (sim) / `ins_odom` (실차) | `CarState` | 키프레임 간 모션 |
| in | `/initialpose` | `PoseWithCovarianceStamped` | RViz 수동 재국지화 |
| **out** | **`/localization/cone_map`** | `ConeArrayWithCovariance` | **콘 지도** (map frame, latched) |
| **out** | **`/localization/ego_odom`** | `Odometry` | **보정된 내 위치** (map → base_footprint) |
| **out** | **`/localization/status`** | `String` | `mapping` → `mapping_converged` → `localization` |

`status`가 `localization`으로 바뀌는 순간이 전체 스택의 분기점입니다 — global planner가
이때부터 레이스라인을 만들기 시작합니다. TF는 graph_slam이 `map→odom→base_footprint`
전체를 소유합니다 (sim의 ground-truth TF는 꺼야 함).

## 어떻게 동작하나

**콘 = 랜드마크.** 키프레임(0.5 m 또는 0.2 rad마다) 포즈와 콘 랜드마크를 정점으로,
오도메트리와 관측을 간선으로 하는 2D 포즈그래프를 g2o(Levenberg)로 풉니다.

**1. 의심 많은 frontend — 콘은 바로 지도에 넣지 않습니다.**
새 관측이 기존 랜드마크와 매칭(χ² 게이트, 전역 greedy + 상호배제)에 실패하면 일단
그래프 **바깥의 tentative track**으로 둡니다. 여러 번 재관측되어 수렴한 트랙만
랜드마크로 승격하고, 그 시점에 과거 관측 이력을 원래 키프레임들에 소급 등록합니다.
유령 콘 하나가 지도에 박히면 association이 연쇄로 무너지기 때문입니다.

**2. 보정의 주역은 CSM(Correlative Scan Matching)입니다.**
최근 ~20 m의 콘 궤적을 하나의 서브맵으로 묶어 지도 격자 위에서 (x, y, θ) 전탐색 매칭:

- **tracking 모드** (localization 중) — 좁은 창(2 m)으로 상시 보정, 드리프트를 association 게이트 아래로 유지
- **loop/seam 모드** — 출발점 복귀 시 넓은 창(10 m)으로 랩 누적 드리프트를 한 번에 회수.
  **오렌지 게이트 콘 ≥2개가 보일 때만** 발화하고, 응답 표면이 애매하면(앨리어싱 능선) 스스로 기각

**mapping 중에는 게이트를 통과하지 못한 보정이 그래프를 건드릴 수 없습니다** — 지도가
만들어지는 중에 잘못된 보정이 들어가면 지도 자체가 휘기 때문입니다.

**3. 랩 완주 판정은 이중 증명을 요구합니다.**
출발점 근처 복귀(기하 조건) **그리고** 오렌지 게이트 seam CSM 등록 성공 — 둘 다
있어야 지도를 얼립니다(일반 loop edge는 증명서로 인정 안 함). 얼린 뒤에는 랜드마크가
전부 고정되고 위치 추정만 계속합니다. 고정 지도도 완전히 불변은 아니어서, 훨씬 엄격한
기준(5회 연속 히트/미스)으로만 콘 추가·삭제를 허용합니다 (오렌지 게이트 콘은 절대 삭제 불가).

**4. 절대 위치 복구는 게이트 별자리로.**
big-orange 출발 게이트 4개의 고유한 배치에서 드리프트와 무관하게 절대 SE2 포즈를
복원합니다(gate anchor). 좌우·180° 모호성은 시야 내 전체 콘의 지도 정합 수로 해소합니다.

## INS/SBG 브리지 (실차 측위 체인)

`ins_pipeline.launch.py`가 SBG Ellipse-D → `sbg_odometry_bridge` → graph_slam 체인을 띄웁니다.
브리지의 설계 원칙은 **"끊기보다 열화"** — 적분기는 하나, 모션 소스만 틱마다 사다리에서 고릅니다:

| 단 | 소스 | 조건 | σ |
|---|---|---|---|
| A | EKF nav 속도 + EKF 헤딩 | solution_mode ≥ 3, velocity_valid | 0.05 (mode 4) / 0.20 (mode 3) |
| C | **raw GNSS** — RTK 에폭 델타 + Doppler 속도, 헤딩은 dual-antenna HDT + 자이로 | `/sbg/gps_pos` RTK(≥float)·`/sbg/gps_vel`·`/sbg/gps_hdt` 살아 있음 (EKF 상태 무관) | 0.10 |
| B | 휠속 × 헤딩 (EKF 또는 HDT/자이로) | `/vehicle/wheel_speeds` 신선 | 0.20 |
| C' | raw GNSS Doppler만 (single-point) | gps_vel 살아 있음 | 0.20 |
| hold | 포즈 동결, σ=1e3 | 위 전부 불가, `held_max_sec`(0.5 s)까지만 | 1e3 |
| FAULT | 발행 중단 → SLAM은 스냅샷 관성 주행, PP는 0.5 s 뒤 제동 | 그 이후 | — |

**C단이 생긴 이유 (2026-08-01 실측)**: 세 번의 주행 bag(17:40/17:47/17:50)에서 Ellipse EKF가 주행 중
재초기화(mode 4→0/1, 각 ~30 s, 그동안 48–72 m 주행)됐는데, 그 사이 수신기 자체는 RTK_INT 위치(1 cm)·
Doppler 속도·듀얼안테나 헤딩(0.4°)을 5 Hz로 멀쩡히 내고 있었습니다. 기존 사다리는 solution_mode만 보므로
mode 1 = FAULT → `ins_odom` 30 s 침묵 → 재진입 시 SLAM 포즈가 실차보다 50–70 m 뒤(지도 오염).
C단은 `/sbg/gps_pos`·`gps_vel`·`gps_hdt`·`imu_data`를 직접 읽어 EKF 리셋을 SLAM에게 보이지 않게 합니다
(`scripts/bridge_bag_eval.py`로 실측: 세 outage 모두 발행 공백 ≤0.08 s, outage 종료 시 상대 오도 오차 0.06–0.25 m).
HDT는 안테나 순서에 따른 설치 오프셋(현재 차: 180°)이 있어 EKF가 정상일 때 온라인으로 학습하며(`hdt_yaw_offset_deg`
NaN), 고정하면 EKF 정렬 전 raw 시작도 가능합니다. 실차에는 아직 휠속 소스가 없어(CAN 브리지 미구현) 실제 사다리는
A → C → hold입니다. Hold는 시간 제한이 있고(동결 포즈가 계속 흐르면 하류 워치독이 못 봄), 블라인드 갭 뒤 첫 메시지는
σ=1e3로 나가 그래프가 갭을 가로지르는 델타를 믿지 않습니다.

출력은 두 갈래: `/localization/ins_odom`(점프 없는 상대 오도메트리 — SLAM 입력)과
`/localization/gnss_odom`(절대 ENU 앵커 — HUD·진단용). GNSS 재획득 직후 3초는
공분산 바닥을 깔아 "자신만만하지만 틀린" prior를 막습니다(refix holdoff).
`/sbg_bridge/status`의 `motion_source` 키(ekf/raw_gnss_rtk/raw_gnss_doppler/wheels/zupt/hold/fault)가 현재 단입니다.

> ⚠️ **EKF 리셋의 원인은 별개 문제**입니다. 세 번 모두 저속(≈2 m/s)·급선회(요레이트 ~35°/s) 중이었고 직전
> 수 초간 `gps1_hdt_used=0`(듀얼안테나 헤딩 거부)이었습니다. 08-01 bag의 실측 baseline은 1.50 m인데 flash된
> `leverArmPrimary/Secondary`(0.18/−1.07 m → 1.25 m)와 다르고, `leverArms/cog`=0이라 automotive 프로파일의
> 비홀로노믹 구속이 IMU 위치에 걸립니다(선회 중 course−heading 차 ~11°). 안테나 이설 후 lever arm 재설정을 권장합니다.

## 실행 · 서비스 · 맵

```bash
ros2 launch hyu_localization graph_slam.launch.py     # 휠오돔 + SLAM (sim 기본)
ros2 launch hyu_localization ins_pipeline.launch.py   # INS 체인 포함 (실차 경로)
# race/pbring에는 이미 포함 — 중복 실행 금지 (TF 충돌)
```

```bash
ros2 service call /graph_slam/save_map      std_srvs/srv/Trigger   # 지도 → map/map_<날짜>.csv
ros2 service call /graph_slam/load_map      std_srvs/srv/Trigger   # 저장맵을 고정 지도로 로드
ros2 service call /graph_slam/start_mapping std_srvs/srv/Trigger   # 매핑 모드로 리셋
ros2 service call /graph_slam/reset         std_srvs/srv/Trigger
```

저장맵으로 랩 1을 생략하려면 launch 인자 `localization_mode:=true load_map_path:=<csv>`.

## 튜닝 포인트

전부 [config/graph_slam.yaml](config/graph_slam.yaml) — 주석에 근거 있습니다.

| 파라미터 | 기본 | 뜻 |
|---|---|---|
| `keyframe_distance` | 0.5 m | 그래프 밀도 (촘촘할수록 정확·무거움) |
| `association_max_distance` / `_gate_chi2` | 1.5 / 5.991 | 관측↔랜드마크 매칭 게이트 |
| `csm_track_window_m` / `csm_loop_window_m` | 2.0 / 10.0 | CSM 탐색 창 — loop 창은 랩 드리프트를 덮어야 함 |
| `lap_returns_to_freeze` | 1 | 몇 번째 복귀에 지도를 얼릴지 |
| `odom_sigma_mode3/2` (브리지) | 0.20 | 열화 모드 오도메트리 불신 정도 — **0.5로 올리면 지도가 휨** |
| `odom_sigma_raw_rtk/doppler` (브리지) | 0.10 / 0.20 | raw GNSS 단(C/C')의 오도메트리 σ |
| `held_max_sec` (브리지) | 0.5 | hold(동결 포즈) 허용 시간 — 넘으면 FAULT(발행 중단) |
| `hdt_yaw_offset_deg` (브리지) | NaN | 듀얼안테나 HDT→차량 헤딩 오프셋. NaN=EKF 정상 시 온라인 학습(현재 차 ≈180°) |
| `odom_invalid_sigma` (graph_slam) | 10 m | 이 이상 σ의 모션 입력은 "무효 선언": 동결 입력에 키프레임 안 찍고 그동안 콘 프레임 폐기 |

## 평가

검증은 반드시 **real perception**으로 (`race perception` 또는 풀스택) — GT 콘 입력은
association 회귀를 숨깁니다. ATE·지도 품질 하네스는 `scripts/evaluate_slam.py`,
라이브 모니터는 `ate_monitor`(HUD `GNSS` 줄)입니다.

브리지의 폴백 사다리는 실측 bag으로 오프라인 채점합니다 — 엔코더가 없으므로 휠속은 raw Doppler에서 합성:

```bash
# 기록된 /sbg/*를 브리지 콜백에 그대로 먹여 RTK 대비 상대 오도 오차를 outage별로 출력
python3 src/localization/scripts/bridge_bag_eval.py bag/0801_outdoor/rosbag2_2026_08_01-17_47_09           # 실차 상태(A→C→hold)
python3 src/localization/scripts/bridge_bag_eval.py <bag> --raw heading --wheels doppler                  # 휠 단(B) 검증
python3 src/localization/scripts/bridge_bag_eval.py <bag> --bridge <다른 버전의 bridge.py>                 # A/B 비교
```

단위 테스트: `test/test_sbg_bridge_fallback.py`(휠 DR·hold·refix) + `test/test_sbg_bridge_raw_gnss.py`(raw GNSS 단·
HDT 오프셋 학습·자이로 헤딩·hold 타임아웃·블라인드 갭).
