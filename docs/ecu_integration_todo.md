# ECU 연동 TODO — 휠 엔코더 입력 / 제어 명령 출력

작성 2026-07-29. 실차 퍼셉션·SLAM·플래닝·제어는 백 리플레이로 끝까지 도는 것을
확인했다. 남은 것은 **차량 ECU와의 양방향 연결** 두 가닥뿐이다.

```
   ECU ──(휠 속도·조향)──►  /vehicle/wheel_speeds  ──► wheel_odometry ──► SLAM
   ECU ◄──(조향·가속)────  /vehicle/cmd           ◄── hyu_cmd_selector
```

현재 위쪽은 **가짜로 채워져 있고**, 아래쪽은 **아무도 받지 않는다**.

---

## 1. 입력: 휠 엔코더 → `/vehicle/wheel_speeds`

| 항목 | 값 |
|---|---|
| 토픽 | `/vehicle/wheel_speeds` |
| 타입 | `hyu_msgs/WheelSpeedsStamped` |
| 필드 | `speeds.steering` (rad), `speeds.{lf,rf,lb,rb}_speed` (**RPM**) |
| 소비자 | `hyu_localization/wheel_odometry` → `/localization/wheel_odom` |
| 현재 | `hyu_sensor_bringup/stationary_wheels.py` 가 0으로 채움 |

`wheel_odometry` 는 연속 두 헤더 스탬프에서 dt 를 적분한다. **스탬프가 멈춰
있으면 dt=0 이 되어 정지와는 전혀 다른 동작을 한다** — 실제 시각을 넣을 것.

### 할 일
- [ ] ECU → ROS 브리지 노드 작성 (아래 "미결정" 참고: CAN 인지 시리얼인지 먼저)
- [ ] 단위 확인: 드라이버가 rad/s 나 m/s 로 준다면 **RPM 으로 변환**해서 발행
- [ ] 조향은 **rad**, 부호 규약(좌+/우+)을 `wheel_odometry` 와 맞출 것
- [ ] 발행 주기 결정. 현재 가짜는 50 Hz. SLAM 모션 입력이 아니라 폴백 티어라
      50 Hz 가 하한은 아니지만, 20 Hz 밑으로 내려가면 dt 적분 오차가 커진다
- [ ] **`stationary_wheels.py` 를 끌 것.** 실차에서 같이 돌면 0 속도를 실제
      주행에 덮어써서 dead reckoning 을 조용히 망친다. `bagplay` 전용이다

---

## 2. 출력: `/vehicle/cmd` → ECU

| 항목 | 값 |
|---|---|
| 토픽 | `/vehicle/cmd` |
| 타입 | `ackermann_msgs/AckermannDriveStamped` |
| 발행자 | `hyu_cmd_selector` (`output_topic` 파라미터로 변경 가능) |
| 필드 | `drive.steering_angle` (rad), `drive.speed` (m/s), `drive.acceleration` (m/s²), `drive.jerk` |
| 실측 주기 | **20 Hz** (2026-07-29 백 리플레이) |
| 현재 | 실차에서 이 토픽을 **구독하는 노드가 없다** |

미션 arm 전(standby)에는 `speed: 0.0, acceleration: -5.0` 즉 제동 명령이
나온다. 시뮬레이터는 `AS_DRIVING` 상태에서만 명령을 먹도록 게이트를 두는데,
**실차에는 그 게이트가 아직 없다.**

### 할 일
- [ ] `/vehicle/cmd` 구독 → ECU 프레임 변환·송신 노드 작성
- [ ] **arm 게이트 구현.** 미션이 arm 되기 전 명령은 절대 액추에이터로 나가면
      안 된다. 규약은 이미 있다 — `mission.sh` 헤더가 *"on the vehicle, the
      actuation bridge honoring the same `/vehicle/set_mission` contract"* 라고
      적고 있다. 시뮬레이터 플러그인이 `AS_DRIVING` 밖에서 명령을 무시하듯,
      ECU 브리지도 같은 규약을 지켜야 한다. **지금 실차엔 그게 없다**
- [ ] **워치독.** `/vehicle/cmd` 가 N 주기 끊기면 ECU 가 스스로 제동으로
      떨어지게 할 것. ROS 쪽이 죽었을 때 마지막 명령이 유지되면 안 된다
- [ ] `speed` 와 `acceleration` 중 무엇을 따를지 ECU 와 합의 (지금은 둘 다 실림)
- [ ] 조향 각도 한계·레이트 리밋을 ECU 와 ROS 중 어디서 걸지 결정
- [ ] EBS / `mission stop` 경로가 이 토픽을 거치는지, 별도 하드와이어인지 확인

---

## 3. `fsk` 가 ECU 노드도 같이 띄우게 할 것

노드만 만들고 끝내면 안 된다. **실차 step 1 (`fsk`) 이 센서·퍼셉션과 함께 ECU
브리지도 기동해야 한다.** 지금 `fsk.sh` 는 패널 ① 센서 브링업 + 패널 ② 퍼셉션
뿐이고, 사람이 따로 띄우는 구조면 "띄우는 걸 잊어서 차가 안 움직인다" 가
반드시 한 번은 일어난다.

- [ ] `scripts/fsk.sh` 에 ECU 브리지 패널 추가 (센서 ① / 퍼셉션 ② 와 나란히)
- [ ] 또는 `hyu_sensor_bringup/launch/sensors.launch.py` 안에 넣기.
      **어느 쪽이든 인자 하나로 끌 수 있게 할 것** — ECU 없이 백만 돌릴 때
      (`bagplay`) 는 꺼져 있어야 한다. `gnss:=auto` 가 시리얼 포트 존재로
      판단하는 것과 같은 패턴이 쓸 만하다
- [ ] `fsk.sh` 인자 라우팅에 ECU 관련 토큰 추가. 지금 `lidar:=` `camera:=`
      `gnss:=` `tf:=` `mount:=` `extrinsic:=` `camera_frame:=` 는 센서 패널로,
      나머지는 퍼셉션으로 간다 (`fsk.sh` 의 `SENSOR_EXTRA` 분기)
- [ ] `fskcheck` 에 ECU 토픽 추가해서 한눈에 살았는지 보이게 할 것
- [ ] 기동 실패 시 **조용히 넘어가지 말 것.** 센서 브링업이 그렇듯 뭐가 없는지
      찍고 죽어야 한다 — step 3 에서 "명령이 안 나간다" 로 발견되면 늦다

참고로 step 3 에서 arm 할 수 있는 미션은 다음과 같다 (`mission.sh`):

| 명령 | 기본값 | 비고 |
|---|---|---|
| `mission trackdrive` (`td`) | 10 랩 | KASE 트랙드라이브 |
| `mission autocross` (`ax`) | 1 랩 | 목표 랩에서 자동 정지 |
| `mission skidpad` (`sp`) | 2 | **원당** 랩 수 (우 N, 좌 N) |
| `mission acceleration` (`accel`) | — | 직선 스프린트, 콘 끝나면 제동 |
| `mission dlc` | — | 개활 코리도 데모 (dlc_track) |
| `mission inspection` (`insp`) | — | 제22조 검차: 잭 위 축 회전 + 조향 사인 |
| `mission stop` (`ebs`) | — | EBS 비상제동 |
| `mission reset` | — | standby 복귀, 포즈 리셋, 플래닝 재기동 |
| `mission status` (`st`) | — | 상태/랩/AS/DSSI 일회 조회 |

ECU 브리지는 이 전부에서 동작해야 하고, 특히 `stop`/`ebs` 는 **다른 경로보다
먼저** 도달해야 한다.

## 4. 미결정 (먼저 정해야 나머지가 진행됨)

- [ ] **물리 인터페이스**: CAN(SocketCAN) / RS-232 / 이더넷 중 무엇인가.
      레포에 CAN 코드가 **전혀 없다** (`grep -r socketcan` → 0건). 다만
      `hyu_msgs` 에 `CanState.msg`, `ChassisCommand.msg`, `ChassisState.msg`,
      `VehicleCommands(Stamped).msg` 가 이미 정의돼 있다 — 예전 설계의 흔적으로
      보이며, 새 메시지를 만들기 전에 이것들이 쓸 만한지 먼저 볼 것
- [ ] DBC / 프레임 정의서 확보
- [ ] ECU 펌웨어 쪽 담당자와 토픽·단위·주기·안전 동작 합의

---

## 5. 테스트 방법 (하드웨어 없이)

ECU 없이도 양쪽 다 검증할 수 있다.

```bash
race stop
bagplay "bag/0726_cone detection test" once bg   # 센서 리플레이 + 퍼셉션
stack                                            # INS + SLAM + 플래닝 + 제어
mission trackdrive 3                             # arm
rth /vehicle/cmd                                 # 명령이 나오는지
rte /vehicle/cmd                                 # 내용 확인
```

`bagplay` 는 정지 상태 휠 엔코더와 INS 를 합성해서 넣는다. ECU 브리지가
생기면 **입력 쪽 가짜(`stationary_wheels.py`)를 끄고 실제 ECU 를 물린 뒤 같은
흐름으로 비교**하는 것이 가장 빠른 검증이다.

주의: 이 백은 정차 중 취득이라 SBG 가 3643 메시지 전부 `VERTICAL_GYRO` 로
절대 측위가 없다. 그래서 `bagplay` 는 SBG 도 합성한다(`stationary_ins.py`).
주행 중 취득한 백이 생기면 그 백의 SBG 를 그대로 쓰도록 바꿀 것.

---

## 6. 참고

- 토픽·프레임 규약: [`topic_contract.md`](topic_contract.md)
- 실차 센서/TF 브링업: `src/sensors/hyu_sensor_bringup/`
- 가짜 센서 노드: `hyu_sensor_bringup/scripts/stationary_{wheels,ins}.py`
  — 둘 다 기동 시 경고를 찍는다. **실차에서 그 경고가 보이면 끄지 않은 것이다.**
