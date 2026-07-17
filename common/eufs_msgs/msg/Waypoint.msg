# Waypoint position
geometry_msgs/Point position

# Suggested forward velocity (x direction in car frame)
float64 speed # m/s

# Suggested steering angle
float64 suggested_steering # rad

# --- HYU Formula Student global trajectory fields ---
# Echo of the offline global-trajectory CSV columns produced by
# trajectory_generator (trajectory_planning_helpers global raceline):
#   s_m, x_m, y_m, psi_rad, kappa_radpm, vx_mps, ax_mps2
# global_planner fills these and hyu_frenet_conversion/wpnt_publisher consume
# them as the primary source (falling back to `position` when x_m/y_m are 0).
float64 x_m           # global x position [m]
float64 y_m           # global y position [m]
float64 s_m           # arc length along the raceline [m]
float64 psi_rad       # heading [rad]
float64 kappa_radpm   # curvature [rad/m]
float64 vx_mps        # target longitudinal velocity [m/s]
float64 ax_mps2       # target longitudinal acceleration [m/s^2]