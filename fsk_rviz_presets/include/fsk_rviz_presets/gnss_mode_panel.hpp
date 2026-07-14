// Copyright 2026 HYU Formula Student.
//
// RViz panel to set the simulated SBG Ellipse-D GNSS/RTK correction and INS
// solution mode at runtime. It simply publishes to the sim_ellipse_d control
// topics (/sim_ins/set_correction_type, /sim_ins/set_solution_mode), which
// override that node's internal schedules.

#pragma once

#include <QWidget>

#include "rclcpp/rclcpp.hpp"
#include "rviz_common/panel.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int8.hpp"

class QLabel;

namespace fsk_rviz_presets
{

class GnssModePanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit GnssModePanel(QWidget * parent = nullptr);

  /// Grab the shared RViz ROS node and create the control publishers.
  void onInitialize() override;

private:
  void publishCorrection(const QString & correction);
  void publishSolutionMode(int mode);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr correction_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr solution_pub_;
  QLabel * status_label_{nullptr};
};

}  // namespace fsk_rviz_presets
