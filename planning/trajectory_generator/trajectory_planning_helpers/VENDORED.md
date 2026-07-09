# Vendored: trajectory_planning_helpers 0.79

Source: https://pypi.org/project/trajectory-planning-helpers/0.79/
(TUMFTM, LGPL-3.0 — see LICENSE in this directory). Upstream has not been
updated since 2021.

Vendored here (2026-07-05) so the pipeline runs on the stock system
`python3` without a virtualenv. Do NOT replace this copy with a plain
`pip install trajectory-planning-helpers`:

1. Its package metadata pins `quadprog==0.1.7`, which conflicts with any
   modern quadprog install.
2. The pristine 0.79 release crashes on scipy >= ~1.9 with
   `ValueError: Input vector should be 1-D`.

Local patch vs pristine 0.79 (the fix for point 2):

- `spline_approximation.py`, `dist_to_p()`: `scipy.interpolate.splev`
  returns a list of arrays, which newer
  `scipy.spatial.distance.euclidean` rejects. Both vectors are now
  flattened to strictly 1-D before the distance call.
