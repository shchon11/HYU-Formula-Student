#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace ksae_local
{

/// A Cartesian 2D point or vector. The owning API must document its coordinate frame.
struct Point2D
{
  double x{0.0};
  double y{0.0};
};

/// A planar pose: translation (x, y) and counter-clockwise yaw in radians.
struct Pose2D
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

/// Full 2-by-2 covariance matrix in row-major field order.
struct Covariance2D
{
  double xx{0.0};
  double xy{0.0};
  double yx{0.0};
  double yy{0.0};
};

/// Per-color probability or accumulated normalized color evidence.
struct ConeColorProbability
{
  double blue{0.0};
  double yellow{0.0};
  double orange{0.0};
  double big_orange{0.0};
  double unknown{1.0};
};

/// One cone measurement in the sensor frame supplied to the planning core.
struct ConeObservation
{
  Point2D position_sensor;
  Covariance2D covariance_sensor;
  ConeColorProbability color_probability;
  /// Measurement time in seconds in the caller's monotonic time domain.
  double timestamp_s{0.0};
};

/// A temporally accumulated cone landmark expressed in the odom frame.
struct ConeLandmark
{
  std::uint64_t id{0U};
  Point2D position_odom;
  Covariance2D covariance_odom;
  ConeColorProbability accumulated_color;
  std::size_t observation_count{0U};
  double first_seen_s{0.0};
  double last_seen_s{0.0};
};

enum class BoundarySide
{
  UNKNOWN,
  LEFT,
  RIGHT,
};

enum class CandidateClassification
{
  UNCLASSIFIED,
  CLASSIFIED,
  AMBIGUOUS,
  REJECTED,
};

/// A landmark considered for a boundary, expressed in the odom frame.
struct BoundaryCandidate
{
  std::uint64_t landmark_id{0U};
  Point2D position_odom;
  double left_confidence{0.0};
  double right_confidence{0.0};
  CandidateClassification classification{CandidateClassification::UNCLASSIFIED};
};

/// An ordered boundary hypothesis in the odom frame.
struct BoundaryTrace
{
  BoundarySide side{BoundarySide::UNKNOWN};
  std::vector<BoundaryCandidate> candidates;
  double confidence{0.0};
  double length_m{0.0};
};

/// A boundary sampled at approximately uniform arc-length intervals in the odom frame.
struct SampledBoundary
{
  BoundarySide side{BoundarySide::UNKNOWN};
  std::vector<Point2D> points_odom;
  double sample_spacing_m{0.0};
  double confidence{0.0};
};

/// A left/right boundary association and its geometric quality values.
struct ConeMatch
{
  std::uint64_t left_landmark_id{0U};
  std::uint64_t right_landmark_id{0U};
  Point2D midpoint_odom;
  double track_width_m{0.0};
  double residual_m{0.0};
  double confidence{0.0};
};

/// One parameterized centerline sample expressed in the odom frame.
struct CenterlinePoint
{
  Point2D position_odom;
  double arc_length_m{0.0};
  double heading_rad{0.0};
  double curvature_inv_m{0.0};
  double confidence{0.0};
};

enum class PlannerMode
{
  BOOTSTRAP,
  BOTH_BOUNDARIES,
  LEFT_ONLY,
  RIGHT_ONLY,
  MEMORY_ONLY,
  INVALID,
  TF_UNAVAILABLE,
  ODOM_JUMP,
};

/// Extensible health and quality summary produced by the planning core.
struct PlannerStatus
{
  PlannerMode mode{PlannerMode::BOOTSTRAP};
  double confidence{0.0};
  double valid_path_length_m{0.0};
  std::size_t left_cone_count{0U};
  std::size_t right_cone_count{0U};
  std::size_t match_count{0U};
  double estimated_track_width_m{0.0};
  double residual_m{0.0};
  double mean_landmark_age_s{0.0};
  double max_landmark_age_s{0.0};
};

/// ROS-independent result. Centerline coordinates are in the odom frame.
struct PlannerResult
{
  PlannerStatus status;
  std::vector<CenterlinePoint> centerline_odom;
  std::string diagnostic;
};

}  // namespace ksae_local
