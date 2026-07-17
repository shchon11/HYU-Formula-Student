# TUM MPC Output Bridge

`hyu_tmpc_output_bridge`는 Formula TMPC 래퍼의 `/output`을 EUFS 시뮬레이터가
사용하는 `ackermann_msgs/msg/AckermannDriveStamped` 명령으로 변환합니다.
기본 출력은 실제 차량 명령 토픽이 아닌 `/tmpc/cmd_shadow`이므로 기존
Pure Pursuit와 나란히 실행하면서 값을 먼저 비교할 수 있습니다.

## 데이터 흐름

```text
/output (hyu_tmpc_msgs/msg/TumMpcOutput)
  request_steering_angle_rad ───────────────> drive.steering_angle
  request_long_force_n ── clamp, divide ───> drive.acceleration
  tube_mpc_status ──────────────────────────> validity gate

/tmpc/cmd_shadow (ackermann_msgs/msg/AckermannDriveStamped)
/tmpc/cmd_valid  (std_msgs/msg/Bool)
```

`long_acc_target_mps2`는 MPC의 목표/진단 값이므로 제어 명령 변환에는 사용하지
않습니다. 실제 종방향 제어 출력인 `request_long_force_n`만 사용합니다.

## 힘-가속도 변환과 질량 주의사항

현재 변환식은 다음과 같습니다.

```text
force = clamp(request_long_force_n, -6000, 6000)
acceleration = clamp(force / conversion_mass_kg, -10.0, 3.0)
```

`conversion_mass_kg`는 자동 생성된 MPC C++ 코드에 구워진 질량
(`mvdc_mpc_data.cpp`의 `vehiclemass_kg`, 현재 실측 `225 kg`)과 반드시
같아야 합니다. MPC는 그 모델 질량 기준으로 힘을 계획하므로, 같은 질량으로
나눠야 MPC가 의도한 가속도가 복원됩니다. 시뮬레이션 차량 질량이 아니라
**생성 코드의 질량**을 따라야 한다는 점에 주의하세요.

추후 MPC 모델을 다른 질량으로 재생성해서 코드를 교체하면 이 bridge의
`conversion_mass_kg`도 반드시 함께 변경해야 합니다. Bridge는 생성 코드
내부 질량을 자동으로 읽거나 동기화하지 않습니다.

가속도 클램프는 EUFS `eufs/configDry.yaml` 플랜트 한계(`[-10, +3] m/s²`)와
일치시켰습니다. 실차의 구동 한계는 실측 GGV(ax ≈ 11.8 m/s²) 기준으로
별도 조정이 필요합니다.

## 안전 동작

다음 조건을 모두 만족할 때만 TMPC 명령을 전달합니다.

- `tube_mpc_status == 2` (`OK`)
- steering과 force가 유한한 값
- 마지막 `/output` 수신 후 경과 시간이 `output_timeout_sec` 이하

조건을 만족하지 않거나 아직 `/output`을 받지 못했으면 bridge는 기본 100 Hz로
다음 안전 명령과 `cmd_valid=false`를 계속 발행합니다. 정상 명령일 때는 같은
타이머 주기에서 `cmd_valid=true`를 발행합니다.

```text
drive.speed = 0.0
drive.steering_angle = 0.0
drive.acceleration = -5.0
```

질량, 제한값, timeout 또는 publish rate 파라미터 자체가 유효하지 않으면 잘못된
안전 명령을 발행하지 않도록 노드가 시작을 거부합니다.

`TumMpcOutput`에는 header/stamp가 없으므로 freshness는 메시지를 받은 로컬
steady clock 기준으로 판단합니다.

## 실행

Shadow mode:

```zsh
ros2 launch hyu_tmpc_output_bridge hyu_tmpc_output_bridge.launch.xml
```

실제 EUFS `/cmd`로 내보내려면 Pure Pursuit 등 다른 `/cmd` publisher를 먼저 끈
뒤 명시적으로 출력 토픽을 바꿉니다.

```zsh
ros2 topic info /cmd --verbose
ros2 launch hyu_tmpc_output_bridge hyu_tmpc_output_bridge.launch.xml output_topic:=/cmd
```

EUFS는 `commandMode:=acceleration`으로 실행해야 `drive.acceleration`을 실제
입력으로 사용합니다. `drive.speed`는 안전을 위해 항상 `0.0`으로 채웁니다.

## 파라미터

| 이름 | 기본값 | 설명 |
| --- | ---: | --- |
| `input_topic` | `/output` | TMPC 출력 토픽 |
| `output_topic` | `/tmpc/cmd_shadow` | Ackermann 명령 출력 토픽 |
| `valid_topic` | `/tmpc/cmd_valid` | 변환 결과 validity 토픽 |
| `conversion_mass_kg` | `225.0` | force를 acceleration으로 바꾸는 질량 (생성 코드 질량과 동일해야 함) |
| `steering_min_rad` | `-0.52` | 최소 조향각 |
| `steering_max_rad` | `0.52` | 최대 조향각 |
| `acceleration_min_mps2` | `-10.0` | 최소 가속도 |
| `acceleration_max_mps2` | `3.0` | 최대 가속도 |
| `safe_brake_mps2` | `-5.0` | invalid/timeout 안전 감속 |
| `output_timeout_sec` | `0.1` | `/output` freshness 한계 |
| `publish_rate_hz` | `100.0` | Ackermann 명령 발행 주기 |

## 빌드 및 테스트

```zsh
cd /home/shchon11/fsk
source /opt/ros/humble/setup.zsh

colcon build --base-paths src --packages-select \
  hyu_tmpc_msgs \
  hyu_tmpc_output_bridge

source install/setup.zsh
colcon test --packages-select hyu_tmpc_output_bridge
colcon test-result --verbose
```
