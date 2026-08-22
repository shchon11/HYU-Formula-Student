# drive_udp_bridge

AGX와 Speedgoat ECU 사이의 양방향 UDP 통신을 담당하는 독립 ROS 2 Humble
패키지입니다. `/vehicle/cmd`의 최신 Ackermann 주행 명령을 ECU에 전달하고, ECU의
누적 wheel encoder count를 바퀴별 선속도(m/s)로 변환해
`/vehicle/wheel_speeds`로 발행합니다.

## ROS 인터페이스

| 구분 | 이름 | 타입 | 기본값 |
|---|---|---|---|
| 입력 토픽 | `command_topic` | `ackermann_msgs/msg/AckermannDriveStamped` | `/vehicle/cmd` |
| 입력 토픽 | `auto_state_topic` | `hyu_msgs/msg/CanState` | `/vehicle/as_state` |
| 출력 토픽 | `wheel_speeds_topic` | `hyu_msgs/msg/WheelSpeedsStamped` | `/vehicle/wheel_speeds` |
| 노드 | — | — | `drive_udp_bridge` |
| 실행 파일 | — | — | `drive_udp_bridge` |

다음 두 필드만 전송합니다. 별도의 gain, clipping 또는 단위 변환은 없습니다.

| ROS 필드 | UDP 필드 | 단위와 부호 |
|---|---|---|
| `drive.speed` | `speed` | 목표 종방향 속도, m/s. 양수는 전진, 음수는 후진 |
| `drive.steering_angle` | `steering_angle` | 앞차축 중앙 가상 휠 조향각, rad. 양수는 좌회전 |

## UDP 패킷 규격

Python `struct` 형식은 `<ffBB`이고 항상 Little Endian, 총 10 bytes입니다.

| Byte offset | 크기 | 타입 | 필드 |
|---:|---:|---|---|
| 0–3 | 4 bytes | IEEE-754 float32 | `speed` |
| 4–7 | 4 bytes | IEEE-754 float32 | `steering_angle` |
| 8 (9번째 byte) | 1 byte | uint8 | `enable` (항상 `1`) |
| 9 (10번째 byte) | 1 byte | uint8 | `autonomous_enable` |

동일한 패킷을 만드는 Python 예시는 다음과 같습니다.

```python
packet = struct.pack('<ffBB', speed, steering_angle, 1, autonomous_enable)
```

최신 `/vehicle/as_state`가 `AS_DRIVING`이면 `autonomous_enable=1`, 그 외에는
`autonomous_enable=0`으로 전송합니다. 이 마지막 바이트는 자율주행 스위치 상태만
나타내며 주행 명령 watchdog과 독립적입니다.

스위치 **OFF→ON 전환**에서는 먼저 SLAM 지도를 초기화하고(`map_reset_service`,
기본 `/graph_slam/reset`, `std_srvs/Trigger`) 그 응답이 `success`일 때에만
`autonomous_enable=1`로 올립니다 — 버튼을 누르는 순간 깨끗한 지도에서 주행이
시작되도록. 응답이 없거나 실패하면 0을 유지하며 `map_reset_timeout_sec`마다
재시도합니다(로그 ERROR). **ON→OFF 전환은 즉시** 0입니다. SLAM 없이 벤치에서
쓸 때는 `require_map_reset: false`로 즉시 1이 되게 할 수 있습니다.

다음 경우에는 `speed=0.0`, `steering_angle=0.0`을 전송합니다.

- 자율주행 스위치가 OFF인 경우
- 노드 시작 후 명령을 한 번도 받지 못한 경우
- 마지막 명령 수신 후 `command_timeout_sec`를 초과한 경우
- NaN, Inf 또는 float32 범위를 벗어난 명령을 받은 경우

`enable`은 ECU 요청에 따라 항상 `1`입니다. 스위치가 ON이지만 명령 watchdog이
만료된 경우 패킷은 `(0.0, 0.0, 1, 1)`입니다.
AS 상태를 받지 못했거나 `auto_state_timeout_sec`를 초과하면 스위치도 fail-closed되어
패킷은 `(0.0, 0.0, 1, 0)`이 됩니다.

두 timeout은 ROS 메시지 시각이 아니라 이 컴퓨터에서 각 토픽을 받은 monotonic
시각으로 계산합니다. `/clock`이 멈춰도 Ethernet watchdog 송신은 계속됩니다.

## ECU → AGX encoder feedback

feedback UDP datagram을 Little Endian `<IIII>`, 총 16 bytes로 해석합니다. 실제
Speedgoat 모델에 연결하기 전에 ECU 팀과 형식 및 필드 순서를 동일하게 확정해야 합니다.

| Byte offset | 크기 | 타입 | 필드 |
|---:|---:|---|---|
| 0–3 | 4 bytes | uint32 | front-left 누적 encoder count |
| 4–7 | 4 bytes | uint32 | front-right 누적 encoder count |
| 8–11 | 4 bytes | uint32 | rear-left 누적 encoder count |
| 12–15 | 4 bytes | uint32 | rear-right 누적 encoder count |

각 count는 motor encoder 원시값이 아니라 정방향에서 증가하고 후진에서 감소하는
wheel-side 누적 counter로 가정합니다. `encoder_counts_per_revolution`에는 encoder
resolution, quadrature, 기어비를 모두 반영한 **타이어 1회전당 실제 count**를
입력해야 합니다. uint32 rollover와 역회전은 bridge가 signed delta로 보정합니다.

연속된 두 feedback packet 사이에서 각 바퀴를 독립적으로 계산합니다.

```text
delta_count = unwrap(current_count - previous_count)
wheel_distance_m = delta_count / counts_per_revolution * (pi * tire_diameter_m)
wheel_speed_mps = wheel_distance_m / delta_time
```

계산 결과는 다음 네 필드만 채워 발행합니다.

```text
/vehicle/wheel_speeds  hyu_msgs/msg/WheelSpeedsStamped
  speeds.lf_speed  front-left m/s
  speeds.rf_speed  front-right m/s
  speeds.lb_speed  rear-left m/s
  speeds.rb_speed  rear-right m/s
```

bridge는 wheel odometry, `x`, `y`, `yaw`, `CarState`를 만들거나 발행하지 않습니다.
첫 packet은 count와 시간 기준점으로만 저장하고 두 번째 packet부터 발행합니다. 잘못된
크기, 허용 속도 초과, 다른 source IP의 packet은 폐기합니다(`feedback_source_ip`,
기본은 `ecu_ip`; `0.0.0.0`이면 소스 검사 없음). feedback timeout 이후에는 마지막
속도를 반복하거나 0을 임의 발행하지 않고 출력을 멈춘 뒤 다음 정상 packet을 새
기준점으로 사용합니다.

`delta_time`은 폴링 주기가 아니라 **datagram 도착 시각** 차이입니다. Linux에서는
커널 수신 타임스탬프(`SO_TIMESTAMPNS`)를 monotonic 시계로 환산해 쓰므로 200 Hz 폴
타이머의 양자화·지터가 속도에 섞이지 않습니다(옵션을 못 켜면 drain 시각으로 대체,
시작 로그에 표시). 발행 메시지의 `header.stamp`도 같은 도착 시각을 노드 시계로 옮긴
값입니다(now − 샘플 나이). 속도 분해능은 count 하나당
`pi * tire_diameter_m / encoder_counts_per_revolution / delta_time`이므로 ECU 송신
주기와 CPR이 정해지면 이 값을 확인하고, 너무 거칠면 ECU 송신 주기를 낮추거나 ECU측
타임스탬프 필드 추가를 검토합니다. 한 틱에 여러 datagram이 쌓여 있으면 가장 최신
것만 사용합니다(누적 count라 손실 없음).

**누가 뭘 정하나.** `feedback_bind_ip`/`feedback_port`는 우리 쪽 값이다(기본 `0.0.0.0:5001`
→ ECU 모델의 feedback 목적지를 "AGX의 192.168.9.x 주소:5001"로 잡아 달라고 전달).
`encoder_counts_per_revolution`만 ECU/드라이브트레인 쪽 값이며, 확정 전(0)에는 노드가
feedback만 끈 채 정상 기동한다. 데이터시트로 계산하기 애매하면(모터축/휠축, 엣지 ×4,
기어비 반영 여부) 직접 재는 게 확실하다 — 브리지를 끄고 count만 찍어 보면서 차를 타이어
정확히 한 바퀴 밀면 된다:

```bash
python3 - <<'EOF'
import socket, struct
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM); s.bind(("0.0.0.0", 5001))
print("FL FR RL RR  (Ctrl+C to stop)")
while True:
    print(*struct.unpack("<IIII", s.recv(64)), end="\r")
EOF
```

한 바퀴 전후의 차이가 `encoder_counts_per_revolution`이다(네 바퀴가 같아야 정상; 후진 시
감소해야 signed delta 가정이 맞는다).

## 벤치 구동 테스트 패턴 (`cmd_pattern`)

스택 없이 ECU 구동만 확인할 때 `/vehicle/cmd`에 고정 패턴을 뿌리는 도구입니다.
조향은 사인파, 속도는 0에서 선형으로 올라가다 최고속에 닿는 순간 0으로 급정지하고
잠시 멈춘 뒤 반복합니다.

```text
steer = steer_amp * sin(2*pi*t / steer_period)              rad, +좌회전
speed = accel * (t mod cycle)   (t mod cycle) < max_speed/accel 동안 (램프)
      = 0                       그 뒤 stop_hold 초 (급정지)
```

```bash
# 터미널 1: 브리지 + AS 버튼 + vehicle_state (벤치 리그, 지도 리셋 없음)
ros2 launch drive_udp_bridge drive_udp_bridge.launch.py bench:=true
# 터미널 2: 패턴 송신 (기본 ±0.3 rad / 4 s, 0→3 m/s @ 1 m/s² 후 2 s 정지, 무한 반복)
ros2 run drive_udp_bridge cmd_pattern
ros2 run drive_udp_bridge cmd_pattern --max-speed 0              # 조향만
ros2 run drive_udp_bridge cmd_pattern --steer-amp 0 --accel 2    # 속도만, 1.5 s 램프
ros2 run drive_udp_bridge cmd_pattern --cycles 3                 # 3회 반복 후 종료
ros2 run drive_udp_bridge cmd_pattern 1                          # 속도 1 m/s 고정(램프/정지 없음), 조향은 사인파
ros2 run drive_udp_bridge cmd_pattern 1 0.2                      # 속도 1 m/s·조향 0.2 rad 둘 다 고정
ros2 run drive_udp_bridge cmd_pattern 1 --steer-amp 0            # 속도 1 m/s 고정, 직진
ros2 run drive_udp_bridge cmd_pattern -h                         # 전체 옵션
```

위치 인자 `SPEED [STEER]`를 주면 그 값을 Ctrl+C까지 그대로 유지합니다(`--cycles` 무시).

| 옵션 | 기본값 | 의미 |
|---|---:|---|
| `--steer-amp` | `0.3` | 조향 사인파 진폭, rad (차량 락 0.52) |
| `--steer-period` | `4.0` | 조향 사인파 주기, s |
| `--max-speed` | `3.0` | 급정지하는 속도, m/s (`0`이면 속도 항상 0) |
| `--accel` | `1.0` | 램프 기울기, m/s² (램프 시간 = max_speed/accel) |
| `--stop-hold` | `2.0` | 급정지 후 0 유지 시간, s |
| `--cycles` | `0` | 반복 횟수, `0`=무한 |
| `--rate` | `50` | 송신 주기, Hz (브리지 watchdog 0.2 s보다 충분히 빠르게) |
| `--topic` | `/vehicle/cmd` | 송신 토픽 |

PlotJuggler로 명령 대 엔코더 속도를 같이 보려면(`sudo apt install ros-humble-plotjuggler-ros`):

```bash
ros2 run plotjuggler plotjuggler -l $(ros2 pkg prefix drive_udp_bridge)/share/drive_udp_bridge/config/plotjuggler_drive.xml
```

위 창: `/vehicle/cmd` speed(검정) + 4륜 `/vehicle/wheel_speeds`, 아래 창: steering_angle.
레이아웃을 열면 ROS2 Topic Subscriber가 두 토픽을 구독한 상태로 시작합니다.

ECU는 AS 버튼이 ON(`AS_DRIVING`)일 때만 이 명령을 받습니다 — 패턴 자체는 자율주행
바이트를 건드리지 않습니다. Ctrl+C 시 0.5 s 동안 `speed=0, steer=0`을 먼저 보내고
종료하므로 watchdog 만료가 아닌 명시적 정지로 끝납니다.

## 파라미터

설정 파일은 `config/drive_udp_bridge.yaml`입니다.

| 파라미터 | 기본값 | 의미 |
|---|---:|---|
| `command_topic` | `/vehicle/cmd` | 최종 제어 명령 토픽 |
| `auto_state_topic` | `/vehicle/as_state` | 자율주행 상태 토픽 (`AS_DRIVING`이면 ON) |
| `ecu_ip` | `""` | Speedgoat IPv4 주소. 필수 |
| `ecu_port` | `0` | Speedgoat 수신 UDP 포트. 1–65535 필수 |
| `local_bind_ip` | `0.0.0.0` | 송신에 사용할 로컬 IPv4 인터페이스 |
| `local_bind_port` | `0` | 송신 source port. 0이면 OS가 임시 포트 선택 |
| `send_rate_hz` | `100.0` | Timer UDP 송신 주기, Hz |
| `command_timeout_sec` | `0.2` | 명령 수신 watchdog, s |
| `auto_state_timeout_sec` | `0.5` | 자율주행 상태 수신 watchdog, s |
| `feedback_bind_ip` | `0.0.0.0` | feedback을 받을 로컬 IPv4 (**우리가 정하는 값**). `0.0.0.0`=모든 인터페이스. port와 함께 설정 |
| `feedback_port` | `5001` | ECU가 feedback을 보낼 AGX UDP port (**우리가 정하는 값** → ECU팀에 "AGX IP:5001"로 전달). IP와 함께 설정 |
| `feedback_poll_rate_hz` | `200.0` | non-blocking UDP receive queue 확인 주기 |
| `feedback_timeout_sec` | `0.2` | feedback 단절 판단 및 속도 계산 최대 dt, s |
| `feedback_source_ip` | `""` | feedback을 받아들일 source IPv4. 빈 값이면 `ecu_ip`, `0.0.0.0`이면 아무 소스나 허용(벤치) |
| `encoder_counts_per_revolution` | `0` | 타이어 1회전당 ECU count 증가량 (**ECU팀/데이터시트 값**: 엔코더 PPR × 쿼드러처 ×4 여부 × 기어비 — 최종값은 Speedgoat 모델이 뭘 누적하느냐에 따름). `0`=미확정 → feedback만 꺼지고 노드는 정상 기동 |
| `tire_diameter_m` | `0.4572` | 타이어 지름, m (Hoosier PAC02 UNLOADED_RADIUS 0.2286 × 2) |
| `max_wheel_speed_mps` | `50.0` | 비정상 count jump 거부 한계. 0이면 검사 비활성화 |
| `wheel_speeds_topic` | `/vehicle/wheel_speeds` | 네 바퀴 m/s 출력 토픽 |
| `wheel_speeds_frame_id` | `base_footprint` | 출력 Header frame |
| `map_reset_service` | `/graph_slam/reset` | OFF→ON 전환 시 먼저 호출하는 지도 초기화 서비스 (`std_srvs/Trigger`) |
| `map_reset_timeout_sec` | `5.0` | 초기화 응답 대기/재시도 주기, s |
| `require_map_reset` | `true` | false면 초기화 없이 즉시 1 (벤치용) |

기본 `ecu_ip`와 `ecu_port`는 의도적으로 사용할 수 없는 값입니다. 실제 ECU 주소를
입력하지 않으면 노드는 오류를 출력하고 종료하므로 잘못된 장비로 송신하지 않습니다.

## Ethernet 설정

PC와 Speedgoat의 Ethernet 인터페이스를 같은 subnet의 고정 IPv4 주소로 설정합니다.
`local_bind_ip`에는 PC의 해당 인터페이스 주소를, `ecu_ip`에는 Speedgoat 주소를
입력합니다. Speedgoat가 정해진 source port를 요구할 때만 `local_bind_port`를 0이 아닌
값으로 설정합니다. `feedback_bind_ip`에는 AGX의 Ethernet 주소를 넣고 ECU의 feedback
목적지 IP/port를 동일하게 설정합니다. 양쪽 firewall에서도 설정한 UDP 포트를 허용해야
합니다.

Speedgoat 수신 모델에서는 10-byte `uint8` datagram을 받아 byte offset 0과 4를
Little-Endian single로, offset 8의 `enable`과 offset 9의 `autonomous_enable`을
각각 uint8로 해석해야 합니다.

## 빌드 및 실행

```bash
cd /path/to/HYU-Formula-Student
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select drive_udp_bridge
source install/setup.bash
```

YAML에 실제 주소를 입력한 뒤 standalone launch를 실행합니다.

```bash
ros2 launch drive_udp_bridge drive_udp_bridge.launch.py
```

원본 설정을 수정하지 않고 별도 YAML을 사용할 수도 있습니다.

```bash
ros2 launch drive_udp_bridge drive_udp_bridge.launch.py \
  params_file:=/absolute/path/to/drive_udp_bridge.yaml
```

### 벤치 ECU 통신 테스트 — `bench:=true`

스택 없이 ECU와의 송수신만 확인할 때는 같은 launch에 `bench:=true`를 줍니다
(`.bashrc`의 `bridge` alias가 이것). 브리지 외에 다음이 같이 뜹니다.

- `as_button.py`(hyu_sensor_bringup): 젯슨 헤더 31번 핀의 AS 버튼 → `/vehicle/as_button`
  (Jetson.GPIO가 없으면 건너뛰고 서비스 대용법을 안내).
- `vehicle_state.py`(hyu_planning_bringup): `/vehicle/as_button` → `/vehicle/as_state`.
  `bench_mission`(기본 `TRACK_DRIVE`)이 미리 선택돼 있어 **버튼만으로 AS_DRIVING**이 됨
  (실차에선 `mission <이름>`이 하는 일; 벤치 전용).
- 브리지는 `require_map_reset:=false`로 기동(SLAM 없음) → 버튼 ON 즉시 `autonomous_enable=1`.

```bash
bridge                                   # = ros2 launch drive_udp_bridge drive_udp_bridge.launch.py bench:=true
bridge as_button:=off                    # 버튼 미배선 벤치: GPIO 드라이버 없이, 아래 서비스가 스위치
# 스위치 대용 서비스 (버튼 드라이버가 돌고 있으면 실제 버튼 상태(토픽)가 20 Hz로 우선해 덮어씀):
ros2 service call /vehicle/set_as_button std_srvs/srv/SetBool "{data: true}"    # ON  -> (0,0,1,1)
ros2 service call /vehicle/set_as_button std_srvs/srv/SetBool "{data: false}"   # OFF -> (0,0,1,0)
ros2 topic echo /vehicle/as_state_str          # OFF / READY / DRIVING
ros2 topic echo /vehicle/wheel_speeds          # ECU count 피드백 (encoder_counts_per_revolution 기입 후)
```

기본값(`bench:=false`)은 브리지 노드 하나만 띄우며 `race.sh`가 쓰는 형태입니다
(버튼 드라이버는 센서 bringup이, vehicle_state는 race.sh가 따로 띄움). 실차에서는
`bench:=true`를 쓰지 마세요 — 미션 사전 선택 때문에 버튼만으로 차가 arm 됩니다.

입력과 노드 상태를 확인합니다.

```bash
ros2 topic info /vehicle/cmd --verbose
ros2 topic echo /vehicle/cmd
ros2 topic info /vehicle/wheel_speeds --verbose
ros2 topic echo /vehicle/wheel_speeds
ros2 node info /drive_udp_bridge
```

이 패키지는 기존 planning/control bringup에 자동으로 포함되지 않습니다. 실제 ECU에
연결할 때 별도 프로세스로 실행해야 합니다.

## 테스트

```bash
colcon test --packages-select drive_udp_bridge
colcon test-result --verbose
```

테스트는 command/feedback 패킷 endian과 크기, watchdog, parameter 검증, uint32
rollover, 역회전, encoder-to-m/s 변환, UDP loopback, ROS topic 발행 및 실제 Timer의
반복 송신과 timeout 전환을 확인합니다.
