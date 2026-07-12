#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "geometry/curvilinear_coordinate_system.h"

namespace frenet_conversion
{

struct ReferenceWaypoint
{
  double x{0.0};
  double y{0.0};
  double s{0.0};
};

enum class VelocityFrame
{
  kMap,
  kBody
};

struct ClcsFrenetConfig
{
  bool closed_loop{true};
  double path_change_tolerance{1.0e-4};
  double duplicate_point_tolerance{1.0e-3};
  double min_path_length{0.5};
  double large_gap_factor{5.0};
  double max_projection_distance{20.0};
  double projection_domain_limit{20.0};
  double projection_domain_epsilon{0.1};
  double projection_domain_eps2{0.0};
  int projection_domain_method{1};
  double tangent_epsilon{0.05};
  bool publish_frenet_velocity{true};
  VelocityFrame velocity_frame{VelocityFrame::kBody};
};

struct ClcsBuildStats
{
  std::uint64_t path_version{0};
  std::size_t input_waypoint_count{0};
  std::size_t reference_point_count{0};
  std::size_t removed_duplicate_count{0};
  std::size_t invalid_point_count{0};
  std::size_t large_gap_count{0};
  std::size_t self_intersection_count{0};
  double track_length{0.0};
  double waypoint_s_max_error{0.0};
  double build_time_ms{0.0};
  bool closed_by_appending_first_point{false};
};

struct ClcsConversionInput
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  double linear_x{0.0};
  double linear_y{0.0};
  double yaw_rate{0.0};
};

struct ClcsConversionResult
{
  bool valid{false};
  int segment_index{-1};
  double s{0.0};
  double d{0.0};
  double raw_s{0.0};
  double reference_yaw{0.0};
  double heading_error{0.0};
  double v_s{0.0};
  double v_d{0.0};
  double yaw_rate{0.0};
  double track_length{0.0};
  double conversion_time_us{0.0};
  double clcs_build_time_ms{0.0};
  double waypoint_s_max_error{0.0};
  double reconstruction_error{0.0};
  std::uint64_t path_version{0};
  std::string error_message;
};

class ClcsFrenetConverter
{
public:
  using Ptr = std::shared_ptr<ClcsFrenetConverter>;
  using ConstPtr = std::shared_ptr<const ClcsFrenetConverter>;

  static Ptr create(
    const std::vector<ReferenceWaypoint> & waypoints,
    const ClcsFrenetConfig & config,
    std::uint64_t path_version);

  ClcsConversionResult convert(const ClcsConversionInput & input) const;

  const ClcsBuildStats & stats() const { return stats_; }
  const std::vector<ReferenceWaypoint> & source_waypoints() const { return source_waypoints_; }
  const std::vector<ReferenceWaypoint> & reference_points() const { return reference_points_; }

  static bool pathChanged(
    const std::vector<ReferenceWaypoint> & previous,
    const std::vector<ReferenceWaypoint> & current,
    double tolerance);

  static double normalizeAngle(double angle);

private:
  ClcsFrenetConverter(
    const ClcsFrenetConfig & config,
    const std::vector<ReferenceWaypoint> & source_waypoints,
    const std::vector<ReferenceWaypoint> & reference_points,
    std::shared_ptr<geometry::CurvilinearCoordinateSystem> clcs,
    const ClcsBuildStats & stats);

  static std::vector<ReferenceWaypoint> preprocessReferencePath(
    const std::vector<ReferenceWaypoint> & waypoints,
    const ClcsFrenetConfig & config,
    ClcsBuildStats & stats);

  static geometry::EigenPolyline toEigenPolyline(
    const std::vector<ReferenceWaypoint> & reference_points);

  static double computePathLength(const std::vector<ReferenceWaypoint> & points);
  static double computeWaypointSMaxError(const std::vector<ReferenceWaypoint> & points);
  static std::size_t countLargeGaps(
    const std::vector<ReferenceWaypoint> & points,
    double large_gap_factor);
  static std::size_t countSelfIntersections(
    const std::vector<ReferenceWaypoint> & points,
    bool closed_loop);

  double normalizeS(double s) const;
  double sForClcsQuery(double s) const;
  double referenceYaw(double raw_s) const;

  ClcsFrenetConfig config_;
  std::vector<ReferenceWaypoint> source_waypoints_;
  std::vector<ReferenceWaypoint> reference_points_;
  std::shared_ptr<geometry::CurvilinearCoordinateSystem> clcs_;
  ClcsBuildStats stats_;
};

VelocityFrame parseVelocityFrame(const std::string & value);
std::string toString(VelocityFrame value);

}  // namespace frenet_conversion
