# SBG Ellipse-D INS — 브링업 & 검증 런북

실차 GNSS/INS(**SBG Ellipse-D**) 연결·설정·검증 절차. SLAM 스택은 이 장치를
`localization/scripts/sbg_odometry_bridge.py`를 통해 `hyu_msgs/CarState`로 받는다.

> 기록: 2026-07-22 실내 브링업. 장치 SN 68336.

---

## 0. 파이프라인이 필요로 하는 것 (sim ↔ 실차 공통 계약)

자율주행 파이프라인은 INS를 **직접** 받지 않는다. `sbg_odometry_bridge`가
SBG 원시 메시지를 융합·좌표변환해 `CarState`로 만들고, 그 뒤가 전부 동일하다.
**sim과 실차의 유일한 차이는 `/sbg/ekf_nav`+`/sbg/ekf_euler`를 누가 내보내는가** 뿐:

```
 SIM :  gazebo ──► sim_ellipse_d(eufs_sensors) ┐
                                               ├─► /sbg/ekf_nav + /sbg/ekf_euler
 실차:  SBG 장치 ──► sbg_driver ───────────────┘        │  ← ★심(seam)★
                                                        ▼
                       sbg_odometry_bridge  ──►  /localization/ins_odom (CarState)  ─┐
                                             ──►  /localization/gnss_odom (전역 앵커) │
                                             ──►  /sbg_bridge/status                 │
                                                                                     ▼
                                     graph_slam ──► /localization/ego_odom ──► 컨트롤러/frenet
```

즉 **sim→실차 전환 = `sim_ellipse_d` 노드를 `sbg_driver`로 교체**하는 것뿐이고,
브리지 이하(graph_slam·컨트롤러)는 손댈 게 없다. `sim_ellipse_d.py`는 실드라이버를
`/sbg/ekf_nav`+`/sbg/ekf_euler` 레벨에서 흉내 내는 드롭인 대체물이다.

### 심에서 파이프라인이 요구하는 계약 (실드라이버가 이걸 그대로 내야 함)

`/sbg/ekf_nav` (**SbgEkfNav**) — 브리지가 읽는 필드:
- `status.solution_mode` (0~4), `status.position_valid`, `status.velocity_valid`
- `latitude`, `longitude`, `altitude`
- `velocity.{x,y,z}` (**NED** m/s), `velocity_accuracy`, `position_accuracy`

`/sbg/ekf_euler` (**SbgEkfEuler**):
- `angle.z` (헤딩, **NED** 0=북 시계방향), `accuracy.z`, `status.heading_valid`

공통: **단조 증가 타임스탬프, ~25 Hz, NED 규약**(브리지 `frame_convention:ned` ↔ 드라이버 `use_enu:false`).
브리지 산출 `CarState`(`/localization/ins_odom`)는 ENU pose + body twist + mode별 공분산 (참고용, 다운스트림 계약).

### 실측에서 "바로 물리기" 위한 세팅 (요약, 상세는 아래 절)

1. **장치 출력**(comA, flash 저장 §4): ekfNav 40ms · ekfEuler 40ms · imuData 40ms · status 1000ms · utcTime 1000ms.
2. **드라이버**: `confWithRos:false`, uart 115200, `use_sim_time:false` → `/sbg/ekf_nav`+`/sbg/ekf_euler` 발행.
3. **브리지**: `frame_convention:ned`, `/vehicle/wheel_speeds` 연결(DR 폴백용).
4. **런치**: 현재 `ins_pipeline.launch.py`는 **`sim_ellipse_d`를 하드코딩**한다 →
   실차 브링업은 이 노드를 빼고 대신 `sbg_driver`를 띄우는 변형이 필요하다. **(미구현 통합 항목)**
   그 외 브리지·graph_slam 인자는 동일하게 재사용.

---

## 1. 장치 / 연결

| 항목 | 값 |
|---|---|
| 모델 | ELLIPSE-D-G4A3-B2 |
| 시리얼 | SN 68336 |
| 펌웨어 | 3.0.3949-stable (IMU calib 2025-10-13) |
| 인터페이스 | FTDI USB-RS232 → `/dev/ttyUSB0`, **115200 bps**, comA |
| 드라이버 | `sbg_ros2_driver` (메인 리포에서 **git-ignored**, 자체 .git), uart config |
| `confWithRos` | **false** — 장치가 설정의 진실원, yaml `log_*`는 퍼블리셔 생성 게이팅용 |

실행: `ros2 launch sbg_driver sbg_device_launch.py` (uart config 기본).

ROS 환경: `ROS_LOCALHOST_ONLY=1`(도메인 0). **새 셸에서 토픽 확인 시**
`--no-daemon` + `--qos-reliability best_effort` 필요(안 그러면 stale daemon으로
데이터가 있어도 `NO DATA`로 오판).

```bash
ros2 topic hz /sbg/imu_data --no-daemon --qos-reliability best_effort
```

---

## 2. ⚠️ `confWithRos: true`를 지금 yaml로 켜지 말 것

`confWithRos:true`면 드라이버가 yaml 전체를 장치에 적용하고 **flash에 저장
(SAVE_SETTINGS)** 한다. 그런데 배포 yaml의 안테나 lever arm은 전부 `0.0`
플레이스홀더다 → **실측 커미셔닝(아래)을 0으로 덮어써 저장**한다.

- 현재 방침: `confWithRos:false` 유지, 장치 설정은 장치에 둔다.
- 나중에 ROS를 설정 진실원으로 쓰고 싶으면 → **먼저 yaml을 아래 실측값과
  일치**시킨 뒤에만 `confWithRos:true`를 켤 것.

---

## 3. 장치 커미셔닝 값 (백업 — 날아가면 이 값으로 복원)

전체 스냅샷: [`sbg_ellipse_d_settings_backup.json`](./sbg_ellipse_d_settings_backup.json)
(`sbgEComApi /api/v1/settings -g` 덤프).

| 항목 | 값 |
|---|---|
| headingMode | `dualAntennaKnownLeverArm` (입력 lever arm을 신뢰, 온라인 재추정 안 함) |
| leverArmPrimary (front) | `[0.692, 0, 0.476]` m — X 전방, Y 우측, Z 하방 |
| leverArmSecondary (rear) | `[-0.527, 0, 0.471]` m |
| 베이스라인 | **1.219 m, 전후(세로), 센터라인**(Y=0), 높이차 5 mm |
| motionProfile | `automotive` |
| alignment | rough `[forward, right]`, fine `[0,0,0]` |
| gnss1 | model/source/sync `internal`, antenna `GENERIC` |
| comA | rs232, 115200 |

**안테나 위치 검증 (실차 빌드와 일치하는지 확인 필요):**
- `dualAntennaKnownLeverArm`라 값이 틀리면 헤딩이 계통오차로 남고 **자가보정 안 됨**.
- **Y(횡) 오차가 헤딩에 제일 치명적**: 1.219 m 베이스라인에서 2 cm 횡오차 ≈ **0.94° 헤딩오차**. 안테나가 실제로 센터라인인지 cm 단위 확인.
- X(전후) 길이는 헤딩 각엔 둔감, 위치엔 직접 영향.
- Z 부호는 헤딩에 거의 무관(두 안테나 높이차 5 mm). 수직 위치에만.
- **primary가 front가 맞는지** 확인(뒤바뀌면 헤딩 180°).

---

## 4. "launch해도 데이터 0" — 출력꺼짐 함정 + 복구

**증상**: `ros2 launch`가 연결(productCode 읽음)까지는 되는데 모든 토픽 0 Hz.
와이어엔 airData(ECOM id 36)만 흐름.

**원인**: 장치 comA 출력 메시지가 `off`로 리셋됨. 그리고 REST로 켜는 것만으론
안 됨 — **REST POST는 스테이징만**, 그냥 리부트(`REBOOT_ONLY`)하면 버려짐.
`/api/v1/settings/save`는 404(별도 save 없음). **반드시 `SAVE_SETTINGS`
(save+reboot)로 flash 저장**해야 유지됨.

**복구 절차:**

```bash
# 0) 드라이버 정지해 /dev/ttyUSB0 해제

# 1) sbgEComApi 툴 빌드 (벤더 라이브러리)
cmake -S sbg_ros2_driver/external/sbgECom -B /tmp/sbgecom -DBUILD_TOOLS=ON
cmake --build /tmp/sbgecom --target sbgEComApi
API=/tmp/sbgecom/sbgEComApi

# 2) comA 출력 메시지 켜기 (트리거 = 주기문자열 "<N>ms"; off/onChange도 유효)
$API -s /dev/ttyUSB0 -r 115200 /api/v1/settings/output/comA/messages/ekfNav   -p -b '"40ms"'   # 25 Hz
$API -s /dev/ttyUSB0 -r 115200 /api/v1/settings/output/comA/messages/ekfEuler -p -b '"40ms"'   # 25 Hz
$API -s /dev/ttyUSB0 -r 115200 /api/v1/settings/output/comA/messages/imuData  -p -b '"40ms"'   # 25 Hz
$API -s /dev/ttyUSB0 -r 115200 /api/v1/settings/output/comA/messages/status   -p -b '"1000ms"' # 1 Hz
$API -s /dev/ttyUSB0 -r 115200 /api/v1/settings/output/comA/messages/utcTime  -p -b '"1000ms"' # 1 Hz

# 3) flash에 저장(+reboot). 방법 중 택1:
#    (a) sbgCenter GUI에서 Save settings
#    (b) 레거시 SBG_ECOM_SAVE_SETTINGS 액션 전송 (libsbgECom.a 링크한 소형 툴)
#    (c) yaml을 §3 실측값과 맞춘 뒤 confWithRos:true로 1회 launch (설정+save를 드라이버가 수행)
```

> 참고: 2026-07-22에 위 5개 메시지를 flash에 저장 완료. 재리셋되지 않는 한
> 이후 `ros2 launch`만으로 스트리밍됨.

---

## 5. 실내 최소 검증 (RTK 장소 나가기 전) — 정상 기준선

솔루션은 GNSS를 못 쓰므로 `solution_mode=1`이 정상. IMU·자세·배선을 검증한다.

| 항목 | 2026-07-22 실측 | 판정 |
|---|---|---|
| IMU \|accel\| (정지) | 9.806 m/s² | ✅ 중력 정확 |
| 자이로 바이어스 (정지) | 0.05 °/s | ✅ ~0 |
| IMU 온도 | 36.7 °C | ✅ |
| EKF roll/pitch | att_valid=true (roll −0.4°, pitch 2.9°) | ✅ 수렴 |
| EKF heading | heading_valid=**false** | ✅ 실내 당연 |
| solution_mode | **1 (VERTICAL_GYRO)** | ✅ 실내 예상값 |
| pos/vel valid | false | ✅ GNSS 픽스 없음 |
| 레이트 | imu/euler/nav 25 Hz, status/utc 1 Hz | ✅ 설정대로 |
| 타임스탬프 | 단조 증가 | ✅ |
| `use_sim_time` | false | ✅ 실차 필수 |
| comA / CAN | port_a_rx/tx, can_rx = true | ✅ |

검증 스니펫(핵심 필드; `.status`는 중첩 메시지):
```python
# SbgEkfNav.status.solution_mode / .position_valid / .velocity_valid
# SbgEkfEuler.status.attitude_valid / .heading_valid
# SbgImuData.accel/.gyro/.temp
```

---

## 6. RTK 장소에서 확인할 것 (실내선 불가)

1. `solution_mode`가 **4 (NAV_POSITION)** 까지 상승 — 브리지 georef 시작 조건.
2. `heading_valid=true` + 듀얼안테나 헤딩이 실제 차 방향과 일치(계통 오프셋 → §3 lever arm 재확인).
3. 브리지가 RTK FIXED에서 `/localization/gnss_odom` sigma ≤ 0.05 로 앵커링.
4. **휠오돔** `/vehicle/wheel_speeds` (hyu_msgs/WheelSpeedsStamped) — 브리지 DR 폴백에 필요. 현재 `odo_recv:false`라 미연결(CAN 휠오돔 드라이버 별도).

---

## 7. SLAM 브리지 요약

`localization/scripts/sbg_odometry_bridge.py`:
- **구독**: `/sbg/ekf_nav`, `/sbg/ekf_euler`, `/vehicle/wheel_speeds`
- **발행**: `/localization/ins_odom` (CarState, 상대 오도메트리), `/localization/gnss_odom` (전역 앵커), `/sbg_bridge/status`
- `frame_convention` 기본 `ned`가 장치 `use_enu:false`와 일치(한쪽만 바꾸면 좌표계 붕괴).
- 실내 mode 1에선 정상적으로 `WAITING FOR FIX` 대기.
