// Copyright 2026 HYU Formula Student.

#include "fsk_rviz_presets/gnss_mode_panel.hpp"

#include <utility>
#include <vector>

#include <QButtonGroup>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QString>
#include <QVBoxLayout>

#include "pluginlib/class_list_macros.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/ros_integration/ros_node_abstraction_iface.hpp"

namespace fsk_rviz_presets
{

namespace
{
constexpr const char * kCorrectionTopic = "/sim_ins/set_correction_type";
constexpr const char * kSolutionTopic = "/sim_ins/set_solution_mode";

// (button label, published correction_type string)
const std::vector<std::pair<QString, QString>> kCorrections = {
  {"RTK Fixed", "rtk_fixed"},
  {"RTK Float", "rtk_float"},
  {"Single (GNSS)", "single"},
};

// (button label, published solution_mode value)
const std::vector<std::pair<QString, int>> kSolutions = {
  {"NAV_POSITION (4)", 4},
  {"NAV_VELOCITY (3)", 3},
  {"AHRS (2)", 2},
  {"VERTICAL_GYRO (1)", 1},
  {"UNINIT (0)", 0},
};
}  // namespace

GnssModePanel::GnssModePanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  auto * layout = new QVBoxLayout(this);

  // --- GNSS/RTK correction row ---
  layout->addWidget(new QLabel("<b>GNSS / RTK correction</b>"));
  auto * correction_group = new QButtonGroup(this);
  correction_group->setExclusive(true);
  auto * correction_row = new QHBoxLayout();
  for (const auto & entry : kCorrections) {
    auto * button = new QPushButton(entry.first, this);
    button->setCheckable(true);
    button->setChecked(entry.second == "rtk_fixed");  // sim_ellipse_d default
    correction_group->addButton(button);
    correction_row->addWidget(button);
    const QString correction = entry.second;
    connect(
      button, &QPushButton::clicked, this,
      [this, correction]() {publishCorrection(correction);});
  }
  layout->addLayout(correction_row);

  // --- INS solution-mode row ---
  layout->addWidget(new QLabel("<b>INS solution mode</b>"));
  auto * solution_group = new QButtonGroup(this);
  solution_group->setExclusive(true);
  auto * solution_row = new QHBoxLayout();
  for (const auto & entry : kSolutions) {
    auto * button = new QPushButton(entry.first, this);
    button->setCheckable(true);
    button->setChecked(entry.second == 4);  // sim_ellipse_d default (NAV_POSITION)
    solution_group->addButton(button);
    solution_row->addWidget(button);
    const int mode = entry.second;
    connect(button, &QPushButton::clicked, this, [this, mode]() {publishSolutionMode(mode);});
  }
  layout->addLayout(solution_row);

  status_label_ = new QLabel("Initializing…", this);
  status_label_->setWordWrap(true);
  layout->addWidget(status_label_);
  layout->addStretch();
}

void GnssModePanel::onInitialize()
{
  auto abstraction = getDisplayContext()->getRosNodeAbstraction().lock();
  if (!abstraction) {
    status_label_->setText("ERROR: no RViz ROS node available.");
    return;
  }
  node_ = abstraction->get_raw_node();
  correction_pub_ = node_->create_publisher<std_msgs::msg::String>(kCorrectionTopic, 10);
  solution_pub_ = node_->create_publisher<std_msgs::msg::UInt8>(kSolutionTopic, 10);
  status_label_->setText(
    QString("Ready. Publishing to %1 and %2.").arg(kCorrectionTopic).arg(kSolutionTopic));
}

void GnssModePanel::publishCorrection(const QString & correction)
{
  if (!correction_pub_) {
    return;
  }
  std_msgs::msg::String msg;
  msg.data = correction.toStdString();
  correction_pub_->publish(msg);
  status_label_->setText(QString("Set correction → %1").arg(correction));
}

void GnssModePanel::publishSolutionMode(int mode)
{
  if (!solution_pub_) {
    return;
  }
  std_msgs::msg::UInt8 msg;
  msg.data = static_cast<uint8_t>(mode);
  solution_pub_->publish(msg);
  status_label_->setText(QString("Set solution_mode → %1").arg(mode));
}

}  // namespace fsk_rviz_presets

PLUGINLIB_EXPORT_CLASS(fsk_rviz_presets::GnssModePanel, rviz_common::Panel)
