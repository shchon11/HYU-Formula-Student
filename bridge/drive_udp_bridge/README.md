# drive_udp_bridge

`/vehicle/cmd`의 최신 Ackermann 주행 명령을 고정 주기의 UDP datagram으로
Speedgoat ECU에 전달하는 독립 ROS 2 Humble 패키지입니다. 제어기 콜백에서는 명령을
저장하기만 하며, 실제 UDP 송신은 steady-clock ROS 2 Timer에서만 수행합니다.

## ROS 인터페이스

| 구분 | 이름 | 타입 | 기본값 |
|---|---|---|---|
| 입력 토픽 | `command_topic` | `ackermann_msgs/msg/AckermannDriveStamped` | `/vehicle/cmd` |
| 입력 토픽 | `auto_state_topic` | `hyu_msgs/msg/CanState` | `/vehicle/as_state` |
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
| `map_reset_service` | `/graph_slam/reset` | OFF→ON 전환 시 먼저 호출하는 지도 초기화 서비스 (`std_srvs/Trigger`) |
| `map_reset_timeout_sec` | `5.0` | 초기화 응답 대기/재시도 주기, s |
| `require_map_reset` | `true` | false면 초기화 없이 즉시 1 (벤치용) |

기본 `ecu_ip`와 `ecu_port`는 의도적으로 사용할 수 없는 값입니다. 실제 ECU 주소를
입력하지 않으면 노드는 오류를 출력하고 종료하므로 잘못된 장비로 송신하지 않습니다.

## Ethernet 설정

PC와 Speedgoat의 Ethernet 인터페이스를 같은 subnet의 고정 IPv4 주소로 설정합니다.
`local_bind_ip`에는 PC의 해당 인터페이스 주소를, `ecu_ip`에는 Speedgoat 주소를
입력합니다. Speedgoat가 정해진 source port를 요구할 때만 `local_bind_port`를 0이 아닌
값으로 설정합니다. 양쪽 firewall에서도 설정한 UDP 포트를 허용해야 합니다.

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

입력과 노드 상태를 확인합니다.

```bash
ros2 topic info /vehicle/cmd --verbose
ros2 topic echo /vehicle/cmd
ros2 node info /drive_udp_bridge
```

이 패키지는 기존 planning/control bringup에 자동으로 포함되지 않습니다. 실제 ECU에
연결할 때 별도 프로세스로 실행해야 합니다.

## 테스트

```bash
colcon test --packages-select drive_udp_bridge
colcon test-result --verbose
```

테스트는 패킷 endian/크기, watchdog, 파라미터 검증, UDP loopback 및 실제 ROS Timer의
반복 송신과 timeout 전환을 확인합니다.
