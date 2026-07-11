# Copyright 2026 shchon11
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

from pathlib import Path
import unittest

import yaml


ROOT = Path(__file__).resolve().parents[1]


def _parameters():
    config = yaml.safe_load((ROOT / "config" / "graph_slam.yaml").read_text())
    return config["graph_slam"]["ros__parameters"]


class GraphSlamConfigTest(unittest.TestCase):

    def test_optimizer_has_finite_default_active_pose_window(self):
        parameters = _parameters()
        interval = int(parameters["optimize_every_n_keyframes"])
        pose_limit = int(parameters["max_optimization_poses"])

        self.assertGreater(interval, 0)
        self.assertGreater(pose_limit, interval)

    def test_timestamp_guards_are_positive_and_frames_are_explicit(self):
        parameters = _parameters()

        self.assertGreater(float(parameters["clock_rollback_threshold"]), 0.0)
        future_lead = float(parameters["max_future_stamp_lead"])
        rollback_threshold = float(parameters["clock_rollback_threshold"])
        self.assertGreater(future_lead, 0.0)
        self.assertLess(future_lead, rollback_threshold)
        self.assertTrue(parameters["car_state_frame"])
        self.assertTrue(parameters["car_state_child_frame"])
        self.assertEqual(parameters["car_state_frame"], parameters["map_frame"])
        self.assertEqual(
            parameters["car_state_child_frame"], parameters["slam_base_frame"]
        )
        tf_frames = {
            parameters["map_frame"],
            parameters["odom_frame"],
            parameters["slam_base_frame"],
        }
        self.assertNotIn("", tf_frames)
        self.assertEqual(len(tf_frames), 3)


if __name__ == "__main__":
    unittest.main()
