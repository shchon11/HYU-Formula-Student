// Copyright 2026 shchon11
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "gazebo_race_car_model/terrain_field.hpp"

#include <algorithm>
#include <cmath>
#include <random>
#include <sstream>

namespace gazebo_plugins {
namespace eufs_plugins {

void TerrainField::generate(const Config &config) {
  _bumps.clear();
  _cells.clear();
  _nx = _ny = 0;

  if (!config.enabled || config.density <= 0.0 || config.half_extent_x <= 0.0 ||
      config.half_extent_y <= 0.0) {
    return;
  }

  // Sample over a rectangle and keep what lands near the track. Rejection
  // sampling rather than placing a quota around each cone, because the cone
  // disks overlap heavily along a track and a quota each would pile bumps up
  // wherever cones happen to be dense. Rejecting keeps the survivors uniform,
  // so `density` means the same thing whichever branch is taken.
  double min_x = -config.half_extent_x;
  double max_x = config.half_extent_x;
  double min_y = -config.half_extent_y;
  double max_y = config.half_extent_y;

  const bool bounded_by_track = !config.track_points.empty();
  if (bounded_by_track) {
    min_x = max_x = config.track_points.front().first;
    min_y = max_y = config.track_points.front().second;
    for (const auto &point : config.track_points) {
      min_x = std::min(min_x, point.first);
      max_x = std::max(max_x, point.first);
      min_y = std::min(min_y, point.second);
      max_y = std::max(max_y, point.second);
    }
    min_x -= config.track_margin;
    max_x += config.track_margin;
    min_y -= config.track_margin;
    max_y += config.track_margin;
  }

  std::mt19937 rng(config.seed);
  std::uniform_real_distribution<double> pick_x(min_x, max_x);
  std::uniform_real_distribution<double> pick_y(min_y, max_y);
  std::uniform_real_distribution<double> pick_radius(config.radius_min, config.radius_max);
  std::normal_distribution<double> pick_height(config.height_mean, config.height_sigma);

  const double margin_sq = config.track_margin * config.track_margin;
  const double area = (max_x - min_x) * (max_y - min_y);
  const auto candidates = static_cast<std::size_t>(std::lround(area * config.density));
  _bumps.reserve(candidates);

  for (std::size_t i = 0; i < candidates; ++i) {
    const double x = pick_x(rng);
    const double y = pick_y(rng);
    // Draw every bump's shape whether or not it is kept, so that the field
    // stays a pure function of the seed: a rejected candidate must not shift
    // the ones after it.
    const double radius = pick_radius(rng);
    const double height = std::max(pick_height(rng), config.height_min);

    if (bounded_by_track) {
      bool near_track = false;
      for (const auto &point : config.track_points) {
        const double dx = x - point.first;
        const double dy = y - point.second;
        if (dx * dx + dy * dy <= margin_sq) {
          near_track = true;
          break;
        }
      }
      if (!near_track) {
        continue;
      }
    }

    Bump bump;
    bump.x = x;
    bump.y = y;
    bump.radius = radius;
    // The normal draw can go non-positive; a bump of zero height is a bump that
    // is not there, and a negative one would be a hole the mesh cannot render.
    bump.height = height;
    _bumps.push_back(bump);
  }

  index();
}

void TerrainField::index() {
  if (_bumps.empty()) {
    return;
  }

  double max_x = _bumps.front().x;
  double max_y = _bumps.front().y;
  double max_radius = 0.0;
  _min_x = _bumps.front().x;
  _min_y = _bumps.front().y;

  for (const Bump &bump : _bumps) {
    _min_x = std::min(_min_x, bump.x - bump.radius);
    _min_y = std::min(_min_y, bump.y - bump.radius);
    max_x = std::max(max_x, bump.x + bump.radius);
    max_y = std::max(max_y, bump.y + bump.radius);
    max_radius = std::max(max_radius, bump.radius);
  }

  // A cell wider than the widest bump bounds a query to the 3x3 neighbourhood:
  // any bump covering the point has its centre within max_radius of it.
  _cell_size = std::max(2.0 * max_radius, 0.5);
  _nx = static_cast<int>(std::floor((max_x - _min_x) / _cell_size)) + 1;
  _ny = static_cast<int>(std::floor((max_y - _min_y) / _cell_size)) + 1;
  _cells.assign(static_cast<std::size_t>(_nx) * static_cast<std::size_t>(_ny), {});

  for (std::size_t i = 0; i < _bumps.size(); ++i) {
    const int ix = static_cast<int>(std::floor((_bumps[i].x - _min_x) / _cell_size));
    const int iy = static_cast<int>(std::floor((_bumps[i].y - _min_y) / _cell_size));
    const int index = cell(ix, iy);
    if (index >= 0) {
      _cells[static_cast<std::size_t>(index)].push_back(static_cast<int>(i));
    }
  }
}

int TerrainField::cell(int ix, int iy) const {
  if (ix < 0 || iy < 0 || ix >= _nx || iy >= _ny) {
    return -1;
  }
  return iy * _nx + ix;
}

double TerrainField::height(double x, double y) const {
  if (_bumps.empty()) {
    return 0.0;
  }

  const int ix = static_cast<int>(std::floor((x - _min_x) / _cell_size));
  const int iy = static_cast<int>(std::floor((y - _min_y) / _cell_size));

  double result = 0.0;
  for (int dy = -1; dy <= 1; ++dy) {
    for (int dx = -1; dx <= 1; ++dx) {
      const int index = cell(ix + dx, iy + dy);
      if (index < 0) {
        continue;
      }
      for (const int b : _cells[static_cast<std::size_t>(index)]) {
        const Bump &bump = _bumps[static_cast<std::size_t>(b)];
        const double offset_x = x - bump.x;
        const double offset_y = y - bump.y;
        const double distance_sq = offset_x * offset_x + offset_y * offset_y;
        if (distance_sq >= bump.radius * bump.radius) {
          continue;
        }
        const double r = std::sqrt(distance_sq) / bump.radius;
        const double z = bump.height * 0.5 * (1.0 + std::cos(M_PI * r));
        result = std::max(result, z);
      }
    }
  }
  return result;
}

std::string TerrainField::sdf(const std::string &model_name,
                             const std::string &mesh_uri) const {
  std::ostringstream out;
  out.precision(6);
  out << std::fixed;
  out << "<sdf version='1.6'><model name='" << model_name << "'>"
      << "<static>true</static><pose>0 0 0 0 0 0</pose><link name='link'>";

  for (std::size_t i = 0; i < _bumps.size(); ++i) {
    const Bump &bump = _bumps[i];
    std::ostringstream geometry;
    geometry << "<geometry><mesh><uri>" << mesh_uri << "</uri>"
             << "<scale>" << bump.radius << " " << bump.radius << " " << bump.height
             << "</scale></mesh></geometry>";
    std::ostringstream pose;
    pose << "<pose>" << bump.x << " " << bump.y << " 0 0 0 0</pose>";

    out << "<collision name='bump_" << i << "'>" << pose.str() << geometry.str()
        << "</collision>";
    out << "<visual name='bump_" << i << "'>" << pose.str() << geometry.str()
        << "<material><script>"
        << "<uri>file://media/materials/scripts/gazebo.material</uri>"
        << "<name>Gazebo/Grey</name></script></material>"
        << "</visual>";
  }

  out << "</link></model></sdf>";
  return out.str();
}

}  // namespace eufs_plugins
}  // namespace gazebo_plugins
