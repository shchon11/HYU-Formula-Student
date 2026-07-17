# Hyu_formular_control

HYU Formula TMPC 래퍼와 관련 메시지 및 어댑터 패키지를 위한 독립형 ROS2 Humble
미니 워크스페이스입니다.

## 패키지 구성

- `hyu_formular_control_msgs`: `TumVehicleState`, `TumTrajectory`, `TumMpcOutput` 메시지
- `hyu_formular_control`: 자동 생성된 경로 매칭/TMPC 래퍼와 저장소에 포함된 OSQP 런타임
- [`hyu_tmpc_state_bridge`](src/hyu_tmpc_state_bridge/README.md): HYU-Formula-Student 시뮬레이션 상태 어댑터
- [`tum_mpc_output_bridge`](src/tum_mpc_output_bridge/README.md): TMPC force/steering 출력을 EUFS Ackermann acceleration 명령으로 변환하는 어댑터

컨트롤러를 빌드할 때 전체 `mod_vehicle_dynamics_control` 저장소는 필요하지 않습니다.
차량 상태 브리지를 빌드하려면 HYU-Formula-Student가 설치한 `eufs_msgs`가 추가로
필요합니다. 출력 브리지는 ROS2의 `ackermann_msgs`와 이 워크스페이스의 메시지만
사용합니다.

## 빌드

컨트롤러와 메시지 패키지만 빌드하는 경우:

```zsh
cd Hyu_formular_control
source /opt/ros/humble/setup.zsh
colcon build --packages-select hyu_formular_control_msgs hyu_formular_control
source install/setup.zsh
```

시뮬레이션 입출력 브리지를 포함한 네 패키지를 모두 빌드하는 경우:

```zsh
cd Hyu_formular_control
source /opt/ros/humble/setup.zsh
source /home/shchon11/fsk/install/setup.zsh
colcon build --packages-select \
  hyu_formular_control_msgs hyu_tmpc_state_bridge \
  tum_mpc_output_bridge hyu_formular_control
source install/setup.zsh
```

zsh 대신 bash를 사용한다면 `setup.zsh`를 `setup.bash`로 바꾸세요.

## 컨트롤러 실행

```zsh
ros2 launch hyu_formular_control hyu_formular_control.launch.xml
```

기본 구독 토픽:

- `/tmpc/vehicle_state` (`hyu_formular_control_msgs/msg/TumVehicleState`)
- `/tmpc/trajectory_performance` (`hyu_formular_control_msgs/msg/TumTrajectory`)
- `/tmpc/trajectory_emergency` (`hyu_formular_control_msgs/msg/TumTrajectory`)

기본 발행 토픽:

- `/output` (`hyu_formular_control_msgs/msg/TumMpcOutput`)

## TMPC 출력 브리지

[`tum_mpc_output_bridge`](src/tum_mpc_output_bridge/README.md)는 `/output`의
steering과 longitudinal force를 EUFS용 `AckermannDriveStamped`로 변환합니다.
기본 출력은 실제 `/cmd`가 아닌 `/tmpc/cmd_shadow`이며, 현재 1300 kg MPC 모델과
300 kg EUFS 차량 사이의 질량 변환 정책은 패키지 README에 정리되어 있습니다.

차량 상태 토픽 이름은 현재 래퍼의 기본값을 의도적으로 따릅니다. 사용하는 스택의
토픽이 `/tmpc/vehicle_state`라면 `hyu_formular_control/vehicle_state_topic`
파라미터를 설정하세요.

## 컨트롤러 파라미터

- `hyu_formular_control/loop_rate_hz`: 기본값 `100.0`
- `hyu_formular_control/state_timeout_sec`: 기본값 `0.5`
- `hyu_formular_control/performance_trajectory_timeout_sec`: 기본값 `0.5`
- `hyu_formular_control/emergency_trajectory_timeout_sec`: 기본값 `0.5`
- `hyu_formular_control/enable_emergency`: 기본값 `false`
- `hyu_formular_control/publish_on_timeout`: 기본값 `false`
- `hyu_formular_control/steering_angle_min_rad`: 기본값 `-0.52`
- `hyu_formular_control/steering_angle_max_rad`: 기본값 `0.52`
- `hyu_formular_control/drive_force_min_n`: 기본값 `-6000.0`
- `hyu_formular_control/drive_force_max_n`: 기본값 `6000.0`

## 참고 사항

- 자동 생성된 예제 `ert_main.cpp` 파일은 이 배포본에 포함되어 있지 않습니다.
- 심볼 중복을 피하기 위해 `rtGetInf.cpp`, `rtGetNaN.cpp`, `rt_nonfinite.cpp`는 각각 한 세트만 컴파일합니다.
- OSQP는 저장소에 포함된 기존 0.6.3 호환 API를 그대로 사용해야 하며, OSQP 1.x로 교체하면 안 됩니다.
