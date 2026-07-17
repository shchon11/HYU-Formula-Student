# TMPC Command Selector

`hyu_cmd_selector` is the only writer of the final EUFS `/vehicle/cmd` topic in the
hybrid Trackdrive setup. It forwards Pure Pursuit in `LOCAL`, waits for a
continuously valid TMPC command after entering `GLOBAL` (a configurable ready
dwell), and then hands `/vehicle/cmd` to TMPC. A TMPC dropout after takeover falls
back to the fresh Pure Pursuit command instead of parking the car; TMPC must
stay valid for a full dwell again before it retakes. On `STOP` or a stop
request the selector keeps forwarding Pure Pursuit — which already brakes
along the path — and only issues its own straight-line safe brake when no
usable Pure Pursuit command exists.

## Topics

| Direction | Topic | Type |
| --- | --- | --- |
| input | `/planning/state` | `std_msgs/msg/String` (`LOCAL`, `GLOBAL`, `STOP`) |
| input | `/planning/stop_request` | `std_msgs/msg/Bool` |
| input | `/control/pp/cmd` | `ackermann_msgs/msg/AckermannDriveStamped` |
| input | `/control/tmpc/cmd` | `ackermann_msgs/msg/AckermannDriveStamped` |
| input | `/control/tmpc/valid` | `std_msgs/msg/Bool` |
| output | `/vehicle/cmd` | `ackermann_msgs/msg/AckermannDriveStamped` |
| output | `/control/selector/status` | `std_msgs/msg/String` |

The status values are `LOCAL_PP`, `GLOBAL_WAITING_TMPC`, `GLOBAL_TMPC`,
`GLOBAL_PP_FALLBACK`, `STOP_PP`, `FAULT_BRAKE`, `STOP_BRAKE`, and
`INPUT_BRAKE`.

Run the complete control/planning composition with:

```zsh
ros2 launch hyu_planning_bringup tmpc_trackdrive.launch.py
```

기존 standard planning/Pure Pursuit launch는 먼저 종료해야 합니다. 실행 후
`ros2 topic info /vehicle/cmd --verbose`에서 `hyu_cmd_selector` 하나만 publisher로
표시되는지 확인하십시오.
