# EUFS Teleop

Keyboard teleoperation for the EUFS simulator, shaped like a driving game so
it is actually comfortable to drive with a keyboard.

## Run

Start the simulator, then in a terminal:

```bash
source install/setup.zsh
ros2 run hyu_teleop teleop
```

The node sets the `MANUAL` mission automatically (`auto_mission` parameter),
which lets `/cmd` through immediately — no mission GUI needed.

## Keys

| Key | Action |
|-----|--------|
| `W` / `↑` | raise target speed (cruise style — no need to hold) |
| `S` / `↓` | lower target speed; at standstill engages brake hold |
| `A` / `←`, `D` / `→` | steer left / right |
| `SPACE` | full brake |
| `C` | center steering instantly |
| `R` | toggle forward/reverse (only near standstill) |
| `M` | re-send MANUAL mission |
| `+` / `-` | speed cap up / down |
| `Q` / `Ctrl-C` | quit (leaves the car braking straight) |

## Why it feels right (driving-sim techniques)

- **Cruise-style speed target** — W/S move a target speed; an internal
  P controller with a jerk-limited acceleration command does the pedal
  work. Keyboard taps never jerk the car.
- **Speed-sensitive steering** — available lock and per-tap step shrink
  with speed (`speed_steer_falloff`), so the car is precise at speed and
  agile in tight maneuvers.
- **Auto-centering** — steering relaxes to center at a rate proportional
  to speed (Live-for-Speed keyboard-stabilized style). Stop pressing and
  the car straightens; at standstill full lock holds for tight turns.
- **Counter-steer boost** — input against the current steering angle acts
  `counter_steer_gain`× faster, so corrections are immediate.
- **Expo curve** — `steer_expo` gives fine control near center without
  losing full lock.
- **Deadman-safe** — commands publish at 50 Hz continuously; quitting
  always leaves a braking command.

## Parameters

`max_speed` (8 m/s cap, adjustable live with +/-), `max_accel` (3),
`max_brake` (8), `max_jerk` (12), `speed_step` (0.4 m/s per tap),
`steer_step` (0.09 per tap), `steer_expo` (1.6), `counter_steer_gain` (2),
`auto_center_rate` (0.16 per m/s), `speed_steer_falloff` (0.07 per m/s),
`speed_kp` (1.6), `cmd_topic` (`/cmd`),
`car_state_topic` (`/odometry_integration/car_state`), `auto_mission` (true).
