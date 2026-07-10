// Copyright 2026 HYU Formula Student.

#include "fsk_rviz_presets/stack_displays.hpp"

#include <QTimer>

#include "rviz_common/display_context.hpp"
#include "rviz_common/properties/property.hpp"

namespace fsk_rviz_presets
{

void PresetGroup::onInitialize()
{
  DisplayGroup::onInitialize();
  // Populate one event-loop tick later: by then this group is fully attached
  // and enabled (its own Enabled flag is applied after onInitialize), so the
  // children go through the exact same code path as an interactive
  // "Add Display" — which is the only path that reliably wires subscriptions.
  QTimer::singleShot(
    0, this, [this]() {
      if (!populated_) {
        populated_ = true;
        populate();
      }
    });
}

void PresetGroup::load(const rviz_common::Config & config)
{
  // A saved session brings its own children — suppress the deferred default
  // population so it does not duplicate what the config restores.
  if (config.mapGetChild("Displays").isValid()) {
    populated_ = true;
    removeAllDisplays();
  }
  DisplayGroup::load(config);
}

rviz_common::Display * PresetGroup::addChildDisplay(
  const QString & class_id, const QString & name)
{
  rviz_common::Display * display = createDisplay(class_id);
  addDisplay(display);
  display->initialize(context_);
  display->setName(name);
  display->setEnabled(true);
  return display;
}

rviz_common::Display * PresetGroup::addTopicDisplay(
  const QString & class_id, const QString & name, const QString & topic,
  const QString & reliability, const QString & durability)
{
  rviz_common::Display * display = addChildDisplay(class_id, name);
  display->subProp("Topic")->setValue(topic);
  if (!reliability.isEmpty()) {
    display->subProp("Topic")->subProp("Reliability Policy")->setValue(reliability);
  }
  if (!durability.isEmpty()) {
    display->subProp("Topic")->subProp("Durability Policy")->setValue(durability);
  }
  return display;
}

void PerceptionStack::populate()
{
  setName("FSK Perception");
  addTopicDisplay(
    "rviz_default_plugins/MarkerArray", "Live Cones (fusion)", "/cones/viz");
}

void SlamStack::populate()
{
  setName("FSK SLAM");
  addTopicDisplay(
    "rviz_default_plugins/MarkerArray", "SLAM Map (cones + graph)",
    "/graph_slam/markers");

  rviz_common::Display * path = addTopicDisplay(
    "rviz_default_plugins/Path", "SLAM Path", "/graph_slam/path", "Best Effort");
  path->subProp("Color")->setValue(QColor(25, 255, 240));

  rviz_common::Display * odom = addTopicDisplay(
    "rviz_default_plugins/Odometry", "Ego Odometry", "/localization/ego_odom",
    "Best Effort");
  odom->subProp("Keep")->setValue(60);

}

void PlanningStack::populate()
{
  setName("FSK Planning");

  rviz_common::Display * raceline = addTopicDisplay(
    "rviz_default_plugins/Path", "Global Raceline", "/global_waypoints/path",
    "Reliable", "Transient Local");
  raceline->subProp("Color")->setValue(QColor(0, 220, 60));
  raceline->subProp("Line Style")->setValue("Billboards");
  raceline->subProp("Line Width")->setValue(0.12f);
  raceline->subProp("Offset")->subProp("Z")->setValue(0.05f);

  rviz_common::Display * window = addTopicDisplay(
    "rviz_default_plugins/Path", "Local Window", "/path_waypoints/path");
  window->subProp("Color")->setValue(QColor(255, 255, 255));
  window->subProp("Line Style")->setValue("Billboards");
  window->subProp("Line Width")->setValue(0.08f);
  window->subProp("Offset")->subProp("Z")->setValue(0.10f);
}

void HudStack::populate()
{
  setName("FSK HUD");
  addTopicDisplay(
    "rviz_2d_overlay_plugins/TextOverlay", "Path CTE HUD", "/planning/cte_overlay",
    "Reliable", "Transient Local");
  addTopicDisplay(
    "rviz_2d_overlay_plugins/TextOverlay", "SLAM Status HUD",
    "/graph_slam/status_overlay", "Reliable", "Transient Local");
  addTopicDisplay(
    "rviz_2d_overlay_plugins/TextOverlay", "GNSS HUD", "/gnss/overlay",
    "Reliable", "Transient Local");
}

void FullStack::populate()
{
  setName("FSK Full Stack");
  addChildDisplay("fsk_rviz_presets/PerceptionStack", "FSK Perception");
  addChildDisplay("fsk_rviz_presets/SlamStack", "FSK SLAM");
  addChildDisplay("fsk_rviz_presets/PlanningStack", "FSK Planning");
  addChildDisplay("fsk_rviz_presets/HudStack", "FSK HUD");
}

}  // namespace fsk_rviz_presets

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(fsk_rviz_presets::PerceptionStack, rviz_common::Display)
PLUGINLIB_EXPORT_CLASS(fsk_rviz_presets::SlamStack, rviz_common::Display)
PLUGINLIB_EXPORT_CLASS(fsk_rviz_presets::PlanningStack, rviz_common::Display)
PLUGINLIB_EXPORT_CLASS(fsk_rviz_presets::HudStack, rviz_common::Display)
PLUGINLIB_EXPORT_CLASS(fsk_rviz_presets::FullStack, rviz_common::Display)
