#pragma once

#include <cstdint>
#include <string>

#include "local_planner/local_path_builder.hpp"

namespace local_planner
{

enum class SourceMode
{
  kLiveCones,
  kSlamMap,
};

struct HeaderMetadata
{
  std::string frame_id;
  std::int32_t stamp_sec{0};
  std::uint32_t stamp_nanosec{0};
  double receive_age_sec{0.0};
};

struct OdomMetadata
{
  HeaderMetadata header;
  std::string child_frame_id;
  Point2 position;
  double yaw{0.0};
};

struct ValidationResult
{
  bool evaluated{false};
  bool valid{false};
  std::string reason{"not implemented"};
};

SourceMode parseSourceMode(const std::string & value);
bool acceptsSource(SourceMode configured, SourceMode incoming);
ValidationResult validateLiveInput(
  const HeaderMetadata & cones, const OdomMetadata & odom,
  double max_stamp_skew_sec = 0.1, double max_receive_age_sec = 0.5);
ValidationResult validateSlamMapInput(
  const HeaderMetadata & map, const OdomMetadata & odom,
  double max_receive_age_sec = 0.5);

}
