# Pure Pursuit controller

The node publishes `/cmd` at 20 Hz from the selected `/path_waypoints` and
`/localization/ego_odom`. It brakes exactly with speed `0`, acceleration `-5`,
and steering `0` unless every required input has been received and is valid.

Freshness is measured from steady-clock receive time. Path, selected validity,
odometry, and stop-request inputs all use `input_timeout_sec` (default `0.5 s`);
an age equal to the timeout is fresh, while a greater age is stale. A missing or
stale stop request fails safe to braking even when its last value was false.
