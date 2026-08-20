// Copyright 2026 shchon11
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "hyu_localization/sbg_raw_ekf_node.hpp"

#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  // Single thread on purpose: every callback touches the same filter and
  // event order matters; the work per message is microseconds.
  rclcpp::spin(std::make_shared<hyu_localization::SbgRawEkfNode>());
  rclcpp::shutdown();
  return 0;
}
