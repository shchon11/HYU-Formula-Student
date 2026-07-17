// Copyright 2026 HYU Formula Student.
//
// One-click RViz display groups for the perception → SLAM → planning stack.
// Adding "FSK … Stack" from the Add-Display dialog creates a DisplayGroup
// pre-populated with the right child displays, topics, QoS and styling —
// the same set default.rviz ships with, but insertable into any session.

#pragma once

#include <QColor>
#include <QString>
#include <QVariant>

#include "rviz_common/display_group.hpp"

namespace hyu_rviz_presets
{

/// DisplayGroup that fills itself with a preset list of child displays.
///
/// When restored from a saved .rviz config the saved children are used
/// instead (the defaults are cleared first), so configs stay authoritative.
class PresetGroup : public rviz_common::DisplayGroup
{
  Q_OBJECT

public:
  void onInitialize() override;
  void load(const rviz_common::Config & config) override;

protected:
  /// Create the preset children. Runs once from onInitialize().
  virtual void populate() = 0;

  /// Create + initialize a child display of `class_id` named `name`.
  rviz_common::Display * addChildDisplay(const QString & class_id, const QString & name);

  /// addChildDisplay + set the child's "Topic" value (and optional QoS).
  rviz_common::Display * addTopicDisplay(
    const QString & class_id, const QString & name, const QString & topic,
    const QString & reliability = QString(), const QString & durability = QString());

private:
  bool populated_{false};
};

class PerceptionStack : public PresetGroup
{
  Q_OBJECT

protected:
  void populate() override;
};

class SlamStack : public PresetGroup
{
  Q_OBJECT

protected:
  void populate() override;
};

class PlanningStack : public PresetGroup
{
  Q_OBJECT

protected:
  void populate() override;
};

class HudStack : public PresetGroup
{
  Q_OBJECT

protected:
  void populate() override;
};

/// Perception + SLAM + Planning + HUD as nested groups.
class FullStack : public PresetGroup
{
  Q_OBJECT

protected:
  void populate() override;
};

}  // namespace hyu_rviz_presets
