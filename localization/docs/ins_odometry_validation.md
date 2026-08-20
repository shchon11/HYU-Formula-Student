# INS 오도메트리 파이프라인 — 검증 현황

> **2026-08-21 갱신**: 아래의 모션 A(`ekf_nav`+`ekf_euler` → `sbg_odometry_bridge.py`)는 폐기되었고,
> raw `imu_data`·`gps_pos`·`gps_vel`·`gps_hdt`만 쓰는 `sbg_raw_ekf`(C++)로 교체되었다. 이 문서는 그 이전 기록이며,
> 현행 구조·검증 수치는 `localization/README.md`의 "INS 체인" 절 참조.

SBG Ellipse-D 기반 localization 파이프라인의 구조·검증 결과·개선 여지 기록.
브링업 절차는 [`sbg_ellipse_d_bringup.md`](./sbg_ellipse_d_bringup.md) 참조.

> 기준 데이터: `rosbag2_2026_07_23-21_23_30` (15분·5.1km·강체마운트·PSRDIFF).
> 이 런에서 **헤딩 소스(자이로 DR)가 처음으로 검증됨.**

---

## 0. 한 줄 요약

그동안 병목이던 **헤딩**이 (강체마운트 + 안테나 baseline + `ekfRotAccelBody`)로 해결됐고,
**bias-보정 자이로(rot_accel)만으로 15분 주행 헤딩 드리프트가 −0.5°** 로 측정됨 →
**GNSS-free(휠+자이로+콘) 방향이 실증적으로 타당.** 남은 입력은 휠속도(CAN)와 콘.

---

## 1. 파이프라인 구조

```
 SBG Ellipse-D ──sbg_driver──► /sbg/ekf_nav (pos·vel·mode)
                               /sbg/ekf_euler (heading)
                               /sbg/ekf_rot_accel_body (bias-corr yaw rate) ★
                               /sbg/gps_pos, /sbg/gps_hdt
                                    │
   ┌── 모션 A (GNSS융합) ──────────┤
   │   ekf_nav + ekf_euler → sbg_odometry_bridge.py
   │     → /localization/ins_odom  (상대 오도, mode별 sigma)
   │     → /localization/gnss_odom (전역 앵커)  ← graph_slam 미구독 (개선①)
   │
   └── 모션 B (GNSS-free) ─────────┤
       /vehicle/wheel_speeds (CAN) + ekf_rot_accel_body → wheel_odometry.py
         → /localization/wheel_odom (상대 오도)
                                    │
   /perception/cones (blue+yellow+orange) ──┐
                                    ▼        ▼
              graph_slam_node (g2o 포즈그래프 2D)
                = 모션(EdgeSE2 델타) + 콘(EdgeSE2PointXY) + orange seam 루프클로저
                → /localization/ego_odom, /localization/cone_map
```

- **모션은 A/B 중 하나**를 graph_slam의 `car_state_topic`으로 (config 기본 = `/localization/wheel_odom`).
- graph_slam은 CarState pose를 **키프레임 간 델타(상대)** 로만 사용 → 모션의 절대드리프트 무관, 헤딩·증분 품질이 관건.
- **loop closure = 출발지 orange 콘 constellation 매칭** (blue/yellow는 반복돼 단독으로 seam 못 닫음).

---

## 2. 검증 결과 (기준 bag)

### 데이터 품질
| 항목 | 값 | 이전 런 대비 |
|---|---|---|
| solution_mode | **NAV_POS 100%** | 82% → 100% |
| position_valid / heading_valid | **100% / 100%** | ~10% / 30~75% |
| 헤딩 정확도 (median) | **0.34°** | 1~2° |
| GNSS 등급 | PSRDIFF 93% (SINGLE 7%) | — |
| 위성 수 (median) | 32 | — |
| 듀얼안테나 baseline (실측) | 1.221~1.223m (설정 1.219, Δ~2mm) | 미해결 → 해결 |

### ★ 오도메트리(GNSS-free 헤딩) 품질 — 핵심
`ekf_rot_accel_body`의 yaw rate를 적분한 헤딩 vs EKF 실제헤딩(GNSS-aided truth):

| 지표 | 값 |
|---|---|
| \|오차\| median | **1.29°** |
| \|오차\| p90 / max | 3.2° / 7.9° |
| **15분 후 종점 드리프트** | **−0.5°** |

→ 순수 자이로로 15분/5km 주행해도 헤딩이 ~1° 안에서 유지. 콘 SLAM 교정범위(~1-3°)를 한참 하회.
코너에서 순간 ±5-8° excursion(자이로 스케일오차)이 있으나 빠르게 0으로 회귀.

> 참고: raw `imu_data` 자이로는 bias 미보정이라 이 값이 안 나옴 → **반드시 `ekf_rot_accel_body` 사용.**

---

## 3. 현재 품질 → 효과

| 측정 품질 | 파이프라인 효과 |
|---|---|
| 자이로 DR ~1°/15min (우수) | 모션 B 헤딩원 충분 → 콘 association gate(1.5m) 안 → 매핑 가능 |
| position_valid 100% (우수) | 모션 A가 **연속 GNSS 앵커** 생성 가능 (이전 순간플래그와 대비) |
| PSRDIFF σ 0.3~1m (보통) | 앵커 서브미터 — RTK(cm) 아님. 랩 반복정밀도 ~1m에 갇힘 |
| 코너 헤딩σ 상승 | 급기동 구간 순간 매핑품질 저하 가능(회귀 빠름) |

---

## 4. 개선 여지 (우선순위)

**P1 — gnss_odom 앵커를 graph_slam에 배선.** 브리지는 전역 앵커를 내보내는데 graph_slam이
`EdgeSE2XYPrior`로 구독·추가하지 않음(브리지 docstring엔 명세, `graph_slam_node.cpp`엔 없음).
배선하면 GPS가 드리프트를 직접 pin → 최대 견고성. pos_valid 100%인 지금이 효과 최대.

**P1 — `/vehicle/wheel_speeds` (CAN) 확보.** 모션 B의 속도원이 아직 없음(sim은 Gazebo 플러그인,
실차는 CAN). rot_accel(헤딩) 검증됐으니 휠속도만 있으면 wheel_odometry 완성.

**P2 — RTK 보정(NTRIP).** PSRDIFF→RTK_FIXED면 앵커 σ 0.5m→~2cm. 어번 멀티패스면 어려움 →
GNSS-free+콘으로 우회(현 방향).

**P2 — perception `/cones` + 출발지 orange 게이트.** 드리프트 교정 실주체. orange 없으면 seam 미폐합.

**P3 — 자이로 스케일 보정 + lever arm.** 코너 excursion은 스케일오차 신호(SBG 자체보정 여지).
`leverArms/cog`(선회), `aiding/odometer/leverArm`(휠오돔, 뒷차축)은 소폭 개선.

---

## 5. 이 결과를 만든 설정 (재현용)

- **장치 flash**: `output/comA/messages/ekfRotAccelBody = "40ms"` (SAVE 필요).
- **드라이버 yaml**: `log_ekf_rot_accel_body: 8` ([config](../../sbg_ros2_driver/config/sbg_device_uart_default.yaml), `patch-sbg-humble.sh`가 재적용).
- **물리**: 장치 강체 고정(회전 금지), 안테나 primary=앞/secondary=뒤·센터라인, baseline 1.219m.
- 진단: `ros2 run hyu_localization status_monitor` (실시간 mode·GNSS·헤딩·baseline·레이트).

---

## 6. 참조 데이터셋

| bag | 내용 | 용도 |
|---|---|---|
| `datasets/rosbag2_2026_07_23-21_23_30/` | 15분·5.1km·강체·PSRDIFF, ekf_rot_accel_body 포함 (17MB) | **헤딩 DR 검증 기준** |

> ros2 bag play datasets/rosbag2_2026_07_23-21_23_30 --topics /sbg/ekf_nav /sbg/ekf_euler /sbg/ekf_rot_accel_body /sbg/gps_pos
> ```

분석 도구(scratchpad, 참고): `odom_rotaccel.py`(rot_accel DR), `gyro_pure.py`(자이로 vs EKF), `bigviz.py`(6면 시각화).
