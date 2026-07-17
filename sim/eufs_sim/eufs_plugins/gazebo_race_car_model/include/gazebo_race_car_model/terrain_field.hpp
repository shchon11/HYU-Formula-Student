// Copyright 2026 shchon11
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef EUFS_SIM__EUFS_PLUGINS__GAZEBO_RACE_CAR_MODEL__INCLUDE__GAZEBO_RACE_CAR_MODEL__TERRAIN_FIELD_HPP_
#define EUFS_SIM__EUFS_PLUGINS__GAZEBO_RACE_CAR_MODEL__INCLUDE__GAZEBO_RACE_CAR_MODEL__TERRAIN_FIELD_HPP_

#include <string>
#include <utility>
#include <vector>

namespace gazebo_plugins {
namespace eufs_plugins {

/// A procedurally generated field of asphalt bumps.
///
/// One field serves two consumers that must agree: `sdf()` emits the Gazebo
/// collision/visual geometry the LiDAR ray-traces and the cameras render, and
/// `height()` answers the queries that place the car. Both describe the same
/// analytic surface, so the car cannot drive through a bump the LiDAR sees, or
/// ride one that is not there. That agreement is the whole point of generating
/// the field here rather than authoring geometry and a height model separately.
///
/// It replaces what the AR(1) road noise could never be: bumps that stay put.
/// The noise re-rolled a fresh perturbation every step, so the same patch of
/// track was never the same twice and the LiDAR never saw any of it. A field is
/// a function of position, so two passes over a bump agree, which is what makes
/// it usable as a regression fixture.
///
/// Each bump is one instance of the unit dome mesh (see
/// scripts/generate_bump_mesh.py) scaled by (radius, radius, height), profile
///
///     h(r) = height * 0.5 * (1 + cos(pi * r / radius))
///
/// whose slope vanishes at both the peak and the rim, so a bump joins the
/// surrounding asphalt without a slope discontinuity for the car to trip on.
class TerrainField {
 public:
  struct Config {
    bool enabled = false;
    unsigned int seed = 7u;
    /// Bumps per square metre of generation area.
    double density = 0.02;
    /// Points the field is grown around -- in practice the track's cones. Bumps
    /// land only within `track_margin` of one of them.
    ///
    /// This is what makes a useful density affordable. Spread over the whole
    /// +-60 m rectangle, a density high enough that the car actually meets
    /// bumps costs thousands of them, nearly all on asphalt no wheel will ever
    /// touch; Gazebo dies somewhere above a thousand. The car only ever drives
    /// between the cones, so that is the only place a bump can matter.
    ///
    /// Empty falls back to the half-extent rectangle below.
    std::vector<std::pair<double, double>> track_points;
    double track_margin = 3.5;
    /// Generation area half-extents [m], centred on the world origin. Used only
    /// when `track_points` is empty.
    double half_extent_x = 60.0;
    double half_extent_y = 60.0;
    /// Peak height [m], normally distributed and clamped at height_min.
    double height_mean = 0.020;
    double height_sigma = 0.010;
    double height_min = 0.004;
    /// Footprint radius [m], uniform over the range.
    double radius_min = 0.25;
    double radius_max = 0.60;
  };

  struct Bump {
    double x = 0.0;
    double y = 0.0;
    double radius = 0.0;  // footprint radius [m]
    double height = 0.0;  // peak height [m]
  };

  /// Deterministic in `seed`: the same seed is the same field, every run.
  void generate(const Config &config);

  /// Ground height at a world point. Zero on bare asphalt.
  ///
  /// Overlapping bumps take the max rather than summing, because the geometry
  /// is a union of solids: where two domes overlap the LiDAR sees the higher
  /// surface, so that is what the car must ride.
  double height(double x, double y) const;

  /// The `<model>` string inserted into the world, describing exactly the
  /// surface `height()` returns. `mesh_uri` must resolve to the unit dome.
  std::string sdf(const std::string &model_name, const std::string &mesh_uri) const;

  const std::vector<Bump> &bumps() const { return _bumps; }
  bool empty() const { return _bumps.empty(); }

 private:
  /// Bumps binned into a uniform grid so `height()` visits only the few that
  /// can cover the query point instead of all of them, on every wheel, every
  /// step.
  int cell(int ix, int iy) const;
  void index();

  std::vector<Bump> _bumps;
  std::vector<std::vector<int>> _cells;
  double _cell_size = 1.0;
  double _min_x = 0.0;
  double _min_y = 0.0;
  int _nx = 0;
  int _ny = 0;
};

}  // namespace eufs_plugins
}  // namespace gazebo_plugins

#endif  // EUFS_SIM__EUFS_PLUGINS__GAZEBO_RACE_CAR_MODEL__INCLUDE__GAZEBO_RACE_CAR_MODEL__TERRAIN_FIELD_HPP_
