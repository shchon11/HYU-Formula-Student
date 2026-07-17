# hyu_tmpc_state_bridge

ROS2 Humble package that combines HYU-Formula-Student simulation/localization
topics into `hyu_tmpc_msgs/msg/TumVehicleState` for the TUM MPC
wrapper.

## Build and run

The HYU-Formula-Student overlay is required for `eufs_msgs`.

```zsh
cd /home/shchon11/fsk/src/Hyu_formular_control
source /opt/ros/humble/setup.zsh
source /home/shchon11/fsk/install/setup.zsh
colcon build --packages-select hyu_tmpc_msgs hyu_tmpc_state_bridge
source install/setup.zsh
```

Run the bridge as a standalone executable or with its launch file:

```zsh
ros2 run hyu_tmpc_state_bridge hyu_tmpc_state_bridge
```

```zsh
ros2 launch hyu_tmpc_state_bridge hyu_tmpc_state_bridge.launch.xml
```

Run `hyu_formular_control` separately after its trajectory inputs are ready.

## 역할과 전체 흐름

`hyu_tmpc_state_bridge`는 HYU-Formula-Student 시뮬레이터와 localization에서
발행하는 여러 메시지의 최신값을 합쳐 MPC 입력 메시지
`hyu_tmpc_msgs/msg/TumVehicleState`를 만든다.

```text
/localization/ego_odom --------------------┐
  pose: x, y, quaternion                   │
  twist: vx, vy, yaw rate                  │
                                           │
/odometry_integration/car_state -----------+--> latest input cache
  linear_acceleration: ax, ay              │         │
                                           │         v
/ros_can/wheel_speeds ---------------------┘    100 Hz timer
  speeds.steering: applied steering angle            │
                                                      ├─ freshness 검사
                                                      ├─ finite/quaternion 검사
                                                      ├─ v, beta, ax_vel 계산
                                                      v
                                            /tmpc/vehicle_state
                                                      │
                                                      v
                                            hyu_formular_control
```

각 subscriber callback은 메시지와 수신 시각만 저장한다. 100 Hz timer는 세
입력의 최신 스냅샷을 가져와 모두 fresh하고 유한한 경우에만 변환 결과를
발행한다. 입력 메시지들을 exact-time synchronize하지는 않으므로 실제 입력
주기와 지연 차이를 시뮬레이션에서 확인해야 한다.

## 필드 매핑

| `TumVehicleState` 필드 | 입력 또는 계산 | 현재 시뮬레이션 의미 |
|---|---|---|
| `se_status` | 파라미터, 기본 `2` | `OK` |
| `se_state` | 파라미터, 기본 `1` | `SE_NORMAL` |
| `x_m`, `y_m` | `/localization/ego_odom.pose.pose.position` | map 좌표계 위치 |
| `psi_rad` | `wrap(yaw_ros - pi/2)` | Formula 규약 heading: 북쪽 0, 서쪽 방향 양수 |
| `dpsi_radps` | `ego_odom.twist.twist.angular.z` | yaw rate |
| `vx_mps`, `vy_mps` | `ego_odom.twist.twist.linear.x/y` | `base_footprint` 기준 body velocity라는 전제 |
| `v_mps` | `hypot(vx, vy)` | 속도 크기 |
| `beta_rad` | `atan2(vy, vx)` | 차량 body sideslip 추정값 |
| `ax_mps2`, `ay_mps2` | `CarState.linear_acceleration.x/y` | 시뮬레이터 vehicle model acceleration |
| `ax_vel_mps2` | `v_mps` 시간 미분 후 저역통과 필터 | 속도 변화로 계산한 종방향 가속도 추정값 |
| `delta_wheel_rad` | `WheelSpeedsStamped.speeds.steering` | 시뮬레이터에 실제 적용된 등가 전륜 조향각 |
| `valid_imu` | 파라미터와 acceleration freshness | 실제 IMU health가 아닌 임시 시뮬레이션 validity |

기본 출력 토픽은 `/tmpc/vehicle_state`이며 MPC wrapper의 기본 구독 토픽과
동일하다. (과거의 `/tmcp` 오타는 양쪽에서 함께 수정되었다.)

## delta_wheel_rad

현재 시뮬레이션에서는 값이 나온다. 브리지는
`/ros_can/wheel_speeds.speeds.steering`을 그대로 사용한다. 시뮬레이터는
`/cmd.drive.steering_angle`을 목표값으로 받은 뒤 steering-rate limit을 적용하고,
현재 적용된 조향각을 wheel-speeds 메시지에 넣는다. 따라서 `/cmd`는 요청값이고
`delta_wheel_rad`는 적용값이다. 조향 노이즈 설정이 활성화되어 있으면 이 토픽에도
노이즈가 포함될 수 있다.

실제 차량으로 전환할 때는 같은 필드가 명령값이 아니라 조향 엔코더 또는 CAN에서
측정한 실제 road-wheel angle인지 반드시 확인해야 한다.

## valid_imu

`valid_imu`는 MPC가 IMU 기반 acceleration/steering 보정 값을 신뢰해도 되는지를
알리는 flag다. 현재 브리지는 실제 IMU status 토픽을 구독하지 않는다. 현재 구현은
다음 조건에서만 `true`를 보낸다.

```text
valid_imu_default == true
AND /odometry_integration/car_state가 timeout 안쪽으로 fresh함
AND ax, ay가 finite함
```

따라서 현재의 `true`는 “실제 IMU self-test 통과”가 아니라 “시뮬레이터 가속도
입력을 사용할 수 있음”이라는 뜻이다. IMU 기반 보정을 끄고 비교하려면 다음처럼
실행한다.

```zsh
ros2 launch hyu_tmpc_state_bridge hyu_tmpc_state_bridge.launch.xml \
  valid_imu_default:=false
```

실제 차량 단계에서는 SBG의 communication, self-test, axis range, message freshness를
사용해 이 값을 만들어야 한다.

## beta_rad와 ax_vel_mps2

`beta_rad`는 다음 식으로 계산한다.

```text
v = hypot(vx, vy)
beta = 0                         if v < min_speed_for_beta_mps
beta = atan2(vy, vx)             otherwise
```

기본 저속 경계는 `0.1 m/s`다. 이 계산이 의미 있으려면 `vx`, `vy`가 반드시 차량
body frame 속도여야 한다. 시뮬레이터의 `CarState.slip_angle`은 현재 앞 타이어 slip
angle로 계산되므로 `beta_rad`의 정답값으로 직접 비교하면 안 된다.

`ax_vel_mps2`는 localization의 새 velocity sample이 들어왔을 때만 갱신한다.

```text
raw_ax_vel = (current_v - previous_v) / dt
alpha = dt / (ax_vel_filter_tau_sec + dt)
filtered_ax_vel += alpha * (raw_ax_vel - filtered_ax_vel)
```

메시지 header stamp가 있으면 `dt` 계산에 사용하고, 없으면 callback 수신 시각을
사용한다. 최초 샘플, `dt <= 0`, `dt > localization_timeout_sec`, stale 복구
직후에는 `ax_vel_mps2=0`으로 필터를 재초기화한다. 기본 필터 시정수는 `0.1 s`다.

`ax_vel_mps2`는 `hypot(vx, vy)`의 미분이고 `ax_mps2`는 시뮬레이터 모델이 직접
내보낸 body-x acceleration이다. 직선 가감속에서는 비슷해야 하지만 선회 중에는
정의 차이와 노이즈 때문에 완전히 같지 않을 수 있다.

## psi_rad 좌표계 변환

`/localization/ego_odom`의 quaternion은 ROS ENU yaw 규약을 사용한다. ROS yaw는
동쪽을 `0`, 북쪽을 `pi/2`로 표현하지만 Formula TMPC의 `psi_rad`는 북쪽을 `0`으로
사용한다. 브리지는 다음 변환을 적용하고 결과를 `[-pi, pi)`로 정규화한다.

```text
psi_formula = wrap_to_pi(yaw_ros - pi/2)
```

따라서 ROS yaw가 동쪽 `0`이면 `psi_rad=-pi/2`, 북쪽 `pi/2`이면 `0`, 서쪽
`pi`이면 `pi/2`다. 이 변환은 heading offset만 바꾸며 x/y 위치, yaw rate,
body-frame vx/vy의 축과 부호는 변경하지 않는다.

## Timeout이 의미하는 것

Timeout은 마지막 callback 수신 이후 해당 토픽을 최신값으로 인정하는 최대
시간이다. 기본값은 세 입력 모두 `0.2 s`다.

```text
age = current ROS time - callback receipt time
fresh = 0 <= age <= timeout
```

localization, acceleration, wheel-speeds 중 하나라도 아직 수신되지 않았거나 stale이면
브리지는 `/tmpc/vehicle_state` 발행을 중단한다. 오래된 조향이나 가속도를 정상값처럼
계속 보내지 않기 위한 동작이다. 입력이 다시 들어오면 자동으로 발행을 재개하고
`ax_vel` 필터는 0부터 다시 시작한다.

브리지 timeout과 MPC timeout은 서로 다른 두 단계의 보호장치다.

- 브리지 입력 timeout: 기본 `0.2 s`; 여러 원본 센서 중 오래된 값이 있으면 조합을 중단
- MPC `state_timeout_sec`: 기본 `0.5 s`; 브리지 출력 자체가 끊기면 제어 입력을 invalid 처리

## 파라미터

실제 ROS 파라미터 이름에는 `hyu_tmpc_state_bridge/` prefix가 붙는다. Launch에서는
아래 suffix를 argument 이름으로 그대로 사용할 수 있다.

| Launch argument / parameter suffix | 기본값 | 설명 |
|---|---:|---|
| `localization_topic` | `/localization/ego_odom` | pose, velocity, yaw-rate 입력 |
| `car_state_topic` | `/odometry_integration/car_state` | acceleration 입력 |
| `wheel_speeds_topic` | `/ros_can/wheel_speeds` | steering 입력 |
| `output_topic` | `/tmpc/vehicle_state` | MPC vehicle-state 출력 |
| `publish_rate_hz` | `100.0` | timer 발행 주기 |
| `localization_timeout_sec` | `0.2` | localization freshness 한계 |
| `car_state_timeout_sec` | `0.2` | acceleration freshness 한계 |
| `wheel_speeds_timeout_sec` | `0.2` | steering freshness 한계 |
| `se_status` | `2` | `0=ERROR`, `1=WARNING`, `2=OK` |
| `se_state` | `1` | `0=OFF`, `1=NORMAL`, `2=BYPASS`, `3=ODOM`, `4=FAIL` |
| `valid_imu_default` | `true` | 시뮬레이션 acceleration 사용 허용 |
| `min_speed_for_beta_mps` | `0.1` | 이 속도 아래에서는 beta를 0으로 설정 |
| `ax_vel_filter_tau_sec` | `0.1` | 속도 미분 저역통과 필터 시정수 |

## 시뮬레이션에서 직접 확인할 항목

1. 세 원본 토픽이 실제로 존재하고 주기가 충분한지 확인한다.

   ```zsh
   ros2 topic hz /localization/ego_odom
   ros2 topic hz /odometry_integration/car_state
   ros2 topic hz /ros_can/wheel_speeds
   ```

2. `/localization/ego_odom.twist`의 `linear.x/y`가 0으로 고정되지 않는지 확인한다.
   Graph SLAM은 입력 `CarState.twist`를 그대로 전달하므로 원본 twist가 비어 있으면
   브리지에서도 `vx`, `vy`, `dpsi`가 0이 된다.

3. 직진, 좌회전, 우회전 주행으로 축과 부호를 확인한다.

   - 직진: `vx > 0`, `vy`와 `beta`는 0 근처
   - 좌/우 선회: `dpsi`, `vy`, `beta`, `delta_wheel_rad`의 부호가 서로 일관적인지 확인
   - localization yaw와 출력은 `psi_rad=wrap(yaw_ros-pi/2)` 관계인지 확인

4. 요청 조향과 적용 조향의 차이를 확인한다.

   ```zsh
   ros2 topic echo /cmd
   ros2 topic echo /ros_can/wheel_speeds
   ros2 topic echo /tmpc/vehicle_state
   ```

   조향 rate limit 때문에 빠르게 steering을 바꿀 때 `/cmd.drive.steering_angle`보다
   `delta_wheel_rad`가 늦게 따라오는 것이 정상이다.

5. 직선 가감속에서 `ax_mps2`와 `ax_vel_mps2`를 비교한다. 속도 step에서 미분값이
   순간적으로 커질 수 있으므로 `ax_vel_filter_tau_sec`를 조절하면서 확인한다.

6. 저속과 정지 상태에서 `beta_rad=0`인지, 정상 속도에서는
   `atan2(vy_mps, vx_mps)`와 일치하는지 확인한다.

7. `valid_imu_default:=false`로 실행했을 때 출력의 `valid_imu`가 false가 되고 MPC의
   IMU 기반 보정 유무에 따른 결과 차이가 있는지 확인한다.

8. 세 입력 중 하나를 끊었을 때 0.2초 후 `/tmpc/vehicle_state`가 멈추고, 다시
   연결하면 자동 복구되는지 확인한다.

   ```zsh
   ros2 topic hz /tmpc/vehicle_state
   ```

9. 최종 메시지의 15개 필드가 모두 finite하고, 정상 입력 중 약 100 Hz로 나오는지
   확인한다.

   ```zsh
   ros2 topic echo /tmpc/vehicle_state
   ros2 topic hz /tmpc/vehicle_state
   ```
