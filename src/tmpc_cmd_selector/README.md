# TMPC Command Selector

`tmpc_cmd_selector` is the only writer of the final EUFS `/cmd` topic in the
hybrid Trackdrive setup. It forwards Pure Pursuit in `LOCAL`, waits for a
continuously valid TMPC command after entering `GLOBAL`, and then latches TMPC
for that GLOBAL episode. A TMPC failure after takeover latches safe braking
until planning returns to `LOCAL` or the selector is restarted.

## Topics

| Direction | Topic | Type |
| --- | --- | --- |
| input | `/planning/state` | `std_msgs/msg/String` (`LOCAL`, `GLOBAL`, `STOP`) |
| input | `/planning/stop_request` | `std_msgs/msg/Bool` |
| input | `/cmd/pure_pursuit` | `ackermann_msgs/msg/AckermannDriveStamped` |
| input | `/cmd/tmpc` | `ackermann_msgs/msg/AckermannDriveStamped` |
| input | `/tmpc/cmd_valid` | `std_msgs/msg/Bool` |
| output | `/cmd` | `ackermann_msgs/msg/AckermannDriveStamped` |
| output | `/tmpc/cmd_selector/status` | `std_msgs/msg/String` |

The status values are `LOCAL_PP`, `GLOBAL_WAITING_TMPC`, `GLOBAL_TMPC`,
`FAULT_BRAKE`, `STOP_BRAKE`, and `INPUT_BRAKE`.

Run the complete control/planning composition with:

```zsh
ros2 launch planning_bringup tmpc_trackdrive.launch.py
```

기존 standard planning/Pure Pursuit launch는 먼저 종료해야 합니다. 실행 후
`ros2 topic info /cmd --verbose`에서 `tmpc_cmd_selector` 하나만 publisher로
표시되는지 확인하십시오.
