// Copyright 2026 shchon11
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef HYU_LOCALIZATION__CORRELATION_GRID_HPP_
#define HYU_LOCALIZATION__CORRELATION_GRID_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

#include "hyu_localization/local_submap.hpp"

namespace hyu_localization
{

// Correlative scan matching (Olson 2009) on cone constellations.
//
// The landmark map is rasterized into a fine correlation grid (each cone
// stamped as a Gaussian smear, so a cone occupies many cells — the QUT
// arXiv:2311.14276 trick that gives a sparse cone map matchable structure)
// plus a coarse grid whose cells hold the MAX of their fine block: an
// admissible upper bound, so a coarse pass can prune (x, y) offsets without
// ever discarding the true optimum. The query constellation (local submap:
// a ~20 m odometry-rigid cone trail) is then matched over a FULL (x, y, θ)
// window — every hypothesis in the window is scored, which is what makes
// this a strong proposal where per-cone NN association is a weak one.
//
// Scoring is the normalized correlation response (mean stamped-grid value
// over the query points, in [0, 1]) — NOT an inlier count. Acceptance
// combines a response threshold with the covariance of the response surface
// around the peak: on a degenerate section (uniform cone spacing along a
// straight) the response forms a ridge, the covariance blows up along it,
// and the match rejects itself — no dedicated aliasing gate needed.
//
// Header-only and ROS/g2o-free (mirrors gate_anchor.hpp) so the matcher is
// unit-testable in isolation.

struct CsmMatch
{
  GateSe2 pose{};              // body -> map, best hypothesis
  double response{0.0};        // normalized correlation at the peak
  // LOCAL response-surface second moments around the peak (Olson 2009 §IV):
  // large sigma along an axis = the data does not constrain that axis.
  double sigma_x{0.0};
  double sigma_y{0.0};
  double sigma_theta{0.0};
  // Best response OUTSIDE the peak's basin: near `response` = a second
  // hypothesis explains the constellation almost as well. For a SYMMETRIC
  // query (the orange gate rectangle) that is expected, and second_pose
  // lets the caller disambiguate the two with independent evidence.
  double second_peak{0.0};
  GateSe2 second_pose{};
  bool valid{false};
};

class CorrelationGrid
{
public:
  struct Params
  {
    double resolution{0.05};        // fine cell size (m)
    int coarse_factor{10};          // coarse cell = this many fine cells
    double smear_sigma{0.2};        // cone Gaussian smear (m)
    double coarse_response_min{0.35};
    double fine_response_min{0.45};
    int min_query_points{10};       // chain-size analog
    double theta_step_coarse{0.035};  // ~2 deg
    double theta_step_fine{0.009};    // ~0.5 deg
    // Blue/yellow corridors repeat at near-uniform spacing (in-color
    // translation ambiguity); the orange gate is the one geometrically
    // unique feature on an FS track. EXTREME weight: gate agreement must
    // dominate the response, corridors only refine it.
    double orange_weight{10.0};
    // Conjunctive scoring (gate stage): response = GEOMETRIC mean over ALL
    // query points, no coverage exemption. One unmatched point collapses
    // the score, so partial-credit optima -- a gate pair overlaid on the
    // WRONG pair, one side matched one side splayed -- can never outscore
    // the full overlay. Points must ALL fit to earn anything.
    bool conjunctive{false};
  };

  // Builds the grids over a bounding box that covers the search region:
  // targets outside it cannot influence a window-bounded match.
  CorrelationGrid(
    const std::vector<SubmapPoint> & targets,
    double center_x, double center_y, double half_extent,
    const Params & params)
  : params_(params)
  {
    resolution_ = std::max(0.01, params.resolution);
    coarse_factor_ = std::max(2, params.coarse_factor);
    origin_x_ = center_x - half_extent;
    origin_y_ = center_y - half_extent;
    fine_size_ = std::max(
      1, static_cast<int>(std::ceil(2.0 * half_extent / resolution_)));
    coarse_size_ = (fine_size_ + coarse_factor_ - 1) / coarse_factor_;
    for (int layer = 0; layer < kLayers; ++layer) {
      fine_[layer].assign(static_cast<std::size_t>(fine_size_) * fine_size_, 0.0F);
      coarse_[layer].assign(
        static_cast<std::size_t>(coarse_size_) * coarse_size_, 0.0F);
    }

    // Stamp each target as a Gaussian patch (max-composited: overlapping
    // cones must not sum beyond 1, or dense sections would out-score a
    // geometrically better fit elsewhere).
    const double sigma = std::max(0.02, params.smear_sigma);
    const int radius_cells =
      std::max(1, static_cast<int>(std::ceil(3.0 * sigma / resolution_)));
    const double inv_two_sigma_sq = 1.0 / (2.0 * sigma * sigma);
    for (const SubmapPoint & target : targets) {
      // An unknown-color target may be any cone: stamp it into EVERY layer
      // (layerOf's -1 is an index only for queries, never for storage).
      const int layer = layerOf(target.color);
      const int layer_begin = layer >= 0 ? layer : 0;
      const int layer_end = layer >= 0 ? layer + 1 : kLayers;
      const int cx = fineIndex(target.x - origin_x_);
      const int cy = fineIndex(target.y - origin_y_);
      if (cx < -radius_cells || cy < -radius_cells ||
        cx >= fine_size_ + radius_cells || cy >= fine_size_ + radius_cells)
      {
        continue;
      }
      ++stamped_targets_;
      for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
        const int iy = cy + dy;
        if (iy < 0 || iy >= fine_size_) {continue;}
        for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
          const int ix = cx + dx;
          if (ix < 0 || ix >= fine_size_) {continue;}
          const double gx = origin_x_ + (ix + 0.5) * resolution_;
          const double gy = origin_y_ + (iy + 0.5) * resolution_;
          const double dsq = (gx - target.x) * (gx - target.x) +
            (gy - target.y) * (gy - target.y);
          const float value = static_cast<float>(std::exp(-dsq * inv_two_sigma_sq));
          for (int stamp_layer = layer_begin; stamp_layer < layer_end; ++stamp_layer) {
            float & cell =
              fine_[stamp_layer][static_cast<std::size_t>(iy) * fine_size_ + ix];
            cell = std::max(cell, value);
          }
        }
      }
    }
    for (int layer = 0; layer < kLayers; ++layer) {
      for (int iy = 0; iy < fine_size_; ++iy) {
        for (int ix = 0; ix < fine_size_; ++ix) {
          const float value =
            fine_[layer][static_cast<std::size_t>(iy) * fine_size_ + ix];
          float & cell = coarse_[layer][
            static_cast<std::size_t>(iy / coarse_factor_) * coarse_size_ +
            ix / coarse_factor_];
          cell = std::max(cell, value);
        }
      }
    }
    // Coverage mask: "the old map has content near here", any color, dilated
    // by ~the corridor width. A query point inside coverage that fails to
    // match is CONTRADICTED and must stay in the denominator (scoring 0);
    // only points outside coverage (frontier the map never saw) are exempt.
    // Without this split, an aliased pose could shed its disagreeing points
    // (e.g. the orange gate) from the normalization and out-score the truth.
    coverage_.assign(static_cast<std::size_t>(coarse_size_) * coarse_size_, 0U);
    const int dilate = std::max(
      1, static_cast<int>(std::ceil(3.0 / (resolution_ * coarse_factor_))));
    for (int iy = 0; iy < coarse_size_; ++iy) {
      for (int ix = 0; ix < coarse_size_; ++ix) {
        bool near_content = false;
        for (int dy = -dilate; dy <= dilate && !near_content; ++dy) {
          const int jy = iy + dy;
          if (jy < 0 || jy >= coarse_size_) {continue;}
          for (int dx = -dilate; dx <= dilate; ++dx) {
            const int jx = ix + dx;
            if (jx < 0 || jx >= coarse_size_) {continue;}
            const std::size_t index =
              static_cast<std::size_t>(jy) * coarse_size_ + jx;
            for (int layer = 0; layer < kLayers; ++layer) {
              if (coarse_[layer][index] > 0.05F) {
                near_content = true;
                break;
              }
            }
            if (near_content) {break;}
          }
        }
        coverage_[static_cast<std::size_t>(iy) * coarse_size_ + ix] =
          near_content ? 1U : 0U;
      }
    }
  }

  int stampedTargets() const {return stamped_targets_;}

  // Mean fine-grid value over the query points of ONE layer at `pose`
  // (uncovered points count as 0 — strict). -1.0 when the query has no
  // points of that layer. Used to demand that the orange gate ITSELF
  // matched, independent of how well the corridors scored.
  double layerResponseAt(
    const std::vector<SubmapPoint> & query, const GateSe2 & pose,
    int layer) const
  {
    const double c = std::cos(pose.theta), s = std::sin(pose.theta);
    double sum = 0.0;
    int count = 0;
    for (const SubmapPoint & point : query) {
      if (layerOf(point.color) != layer) {
        continue;
      }
      const double px = pose.x + c * point.x - s * point.y;
      const double py = pose.y + s * point.x + c * point.y;
      sum += fineAt(layer, px, py);
      ++count;
    }
    return count > 0 ? sum / static_cast<double>(count) : -1.0;
  }

  // Full-window correlative match of `query` (body frame) around `seed`
  // (body -> map). window_xy / window_theta are half-widths.
  CsmMatch match(
    const std::vector<SubmapPoint> & query,
    const GateSe2 & seed,
    double window_xy,
    double window_theta) const
  {
    CsmMatch result;
    if (static_cast<int>(query.size()) < params_.min_query_points ||
      stamped_targets_ < params_.min_query_points)
    {
      return result;
    }

    const double coarse_step = resolution_ * coarse_factor_;
    const int coarse_range =
      std::max(1, static_cast<int>(std::ceil(window_xy / coarse_step)));

    struct Candidate
    {
      double x, y, theta, response;
    };
    std::vector<Candidate> candidates;

    // Coarse pass: full (theta, x, y) sweep against the upper-bound grid.
    for (double theta = seed.theta - window_theta;
      theta <= seed.theta + window_theta + 1e-9;
      theta += params_.theta_step_coarse)
    {
      const double c = std::cos(theta), s = std::sin(theta);
      rotate(query, c, s);
      for (int oy = -coarse_range; oy <= coarse_range; ++oy) {
        for (int ox = -coarse_range; ox <= coarse_range; ++ox) {
          const double tx = seed.x + ox * coarse_step;
          const double ty = seed.y + oy * coarse_step;
          const double response = coveredResponse(tx, ty, true);
          if (response >= params_.coarse_response_min) {
            candidates.push_back({tx, ty, theta, response});
          }
        }
      }
    }
    if (candidates.empty()) {
      return result;
    }
    // Refine only what can still win: candidates within one coarse response
    // notch of the best upper bound.
    double best_upper = 0.0;
    for (const Candidate & candidate : candidates) {
      best_upper = std::max(best_upper, candidate.response);
    }

    // Fine pass: evaluate every refined hypothesis, remember them all, then
    // derive (a) LOCAL moments around the winning basin — the sharpness of
    // the peak itself — and (b) the best response OUTSIDE that basin, the
    // multimodality (aliasing) signal. Folding far-away basins into one
    // global covariance punished sharp-but-multimodal surfaces twice and
    // blurred the two failure modes together.
    struct Evaluation
    {
      double x, y, theta, response;
    };
    std::vector<Evaluation> evaluations;
    double best = -1.0;
    GateSe2 best_pose{};
    const int fine_span = coarse_factor_ / 2;
    for (const Candidate & candidate : candidates) {
      if (candidate.response < best_upper - 0.1) {
        continue;
      }
      for (double theta = candidate.theta - params_.theta_step_coarse * 0.5;
        theta <= candidate.theta + params_.theta_step_coarse * 0.5 + 1e-9;
        theta += params_.theta_step_fine)
      {
        const double c = std::cos(theta), s = std::sin(theta);
        rotate(query, c, s);
        for (int oy = -fine_span; oy <= fine_span; ++oy) {
          for (int ox = -fine_span; ox <= fine_span; ++ox) {
            const double tx = candidate.x + ox * resolution_;
            const double ty = candidate.y + oy * resolution_;
            const double response = coveredResponse(tx, ty, false);
            if (response <= 0.0) {
              continue;
            }
            evaluations.push_back({tx, ty, theta, response});
            if (response > best) {
              best = response;
              best_pose = GateSe2{tx, ty, theta};
            }
          }
        }
      }
    }
    if (best < params_.fine_response_min) {
      return result;
    }
    const double basin_radius = resolution_ * coarse_factor_;
    const double basin_theta = params_.theta_step_coarse;
    double w_sum = 0.0, mx = 0.0, my = 0.0, mt = 0.0;
    double mxx = 0.0, myy = 0.0, mtt = 0.0;
    double second_peak = 0.0;
    for (const Evaluation & evaluation : evaluations) {
      const double dx = evaluation.x - best_pose.x;
      const double dy = evaluation.y - best_pose.y;
      const double dt = evaluation.theta - best_pose.theta;
      const bool in_basin = dx * dx + dy * dy <= basin_radius * basin_radius &&
        std::fabs(dt) <= basin_theta;
      if (in_basin) {
        const double weight = evaluation.response - params_.fine_response_min;
        if (weight > 0.0) {
          w_sum += weight;
          mx += weight * evaluation.x;
          my += weight * evaluation.y;
          mt += weight * evaluation.theta;
          mxx += weight * evaluation.x * evaluation.x;
          myy += weight * evaluation.y * evaluation.y;
          mtt += weight * evaluation.theta * evaluation.theta;
        }
      } else if (evaluation.response > second_peak) {
        second_peak = evaluation.response;
        result.second_pose = GateSe2{evaluation.x, evaluation.y, evaluation.theta};
      }
    }
    if (w_sum <= 0.0) {
      return result;
    }
    result.pose = best_pose;
    result.response = best;
    result.second_peak = second_peak;
    const double ex = mx / w_sum, ey = my / w_sum, et = mt / w_sum;
    result.sigma_x = std::sqrt(std::max(0.0, mxx / w_sum - ex * ex));
    result.sigma_y = std::sqrt(std::max(0.0, myy / w_sum - ey * ey));
    result.sigma_theta = std::sqrt(std::max(0.0, mtt / w_sum - et * et));
    result.valid = true;
    return result;
  }

private:
  // Partial-overlap normalization: score only the query points that land in
  // the stamped support of the target map. At a lap seam, half the submap
  // trail crosses ground the OLD map never covered — those points carry no
  // information about the seam and must not dilute the response — but the
  // covered subset must still be at least min_query_points, so a match can
  // never be carried by a handful of lucky cones.
  double coveredResponse(double tx, double ty, bool coarse) const
  {
    if (params_.conjunctive) {
      // Geometric mean over EVERY point, no coverage exemption: the caller
      // asserts all its points must be explained by the map.
      constexpr double kEps = 1e-3;
      double log_sum = 0.0;
      int n = 0;
      for (const RotatedPoint & point : rotated_) {
        const double value = coarse ?
          coarseAt(point.layer, tx + point.x, ty + point.y) :
          fineAt(point.layer, tx + point.x, ty + point.y);
        log_sum += std::log(std::max(value, kEps));
        ++n;
      }
      if (n < params_.min_query_points) {
        return 0.0;
      }
      return std::exp(log_sum / static_cast<double>(n));
    }
    double sum = 0.0;
    double weight_sum = 0.0;
    int covered = 0;
    for (const RotatedPoint & point : rotated_) {
      const double px = tx + point.x;
      const double py = ty + point.y;
      if (!inCoverage(px, py)) {
        continue;  // frontier: the old map has nothing to say here
      }
      const double value = coarse ?
        coarseAt(point.layer, px, py) : fineAt(point.layer, px, py);
      sum += point.weight * value;
      weight_sum += point.weight;
      ++covered;
    }
    if (covered < params_.min_query_points || weight_sum <= 0.0) {
      return 0.0;
    }
    return sum / weight_sum;
  }

  bool inCoverage(double x, double y) const
  {
    const int ix = fineIndex(x - origin_x_) / coarse_factor_;
    const int iy = fineIndex(y - origin_y_) / coarse_factor_;
    if (ix < 0 || iy < 0 || ix >= coarse_size_ || iy >= coarse_size_) {
      return false;
    }
    return coverage_[static_cast<std::size_t>(iy) * coarse_size_ + ix] != 0U;
  }

  // Layers: 0 blue, 1 yellow, 2 orange+big-orange (the big/small distinction
  // misclassifies at range, so they share a layer). Unknown color -> -1,
  // scored against the best of all layers.
  static int layerOf(std::uint8_t color)
  {
    if (color == 0U) {return 0;}
    if (color == 1U) {return 1;}
    if (color == kSubmapColorOrange || color == kSubmapColorBigOrange) {return 2;}
    return -1;
  }

  struct RotatedPoint
  {
    double x, y, weight;
    int layer;
  };
  void rotate(const std::vector<SubmapPoint> & query, double c, double s) const
  {
    rotated_.clear();
    for (const SubmapPoint & point : query) {
      const int layer = layerOf(point.color);
      rotated_.push_back(
        RotatedPoint{
          c * point.x - s * point.y, s * point.x + c * point.y,
          layer == 2 ? params_.orange_weight : 1.0, layer});
    }
  }

  int fineIndex(double metric) const
  {
    return static_cast<int>(std::floor(metric / resolution_));
  }
  double fineAt(int layer, double x, double y) const
  {
    const int ix = fineIndex(x - origin_x_);
    const int iy = fineIndex(y - origin_y_);
    if (ix < 0 || iy < 0 || ix >= fine_size_ || iy >= fine_size_) {
      return 0.0;
    }
    const std::size_t index = static_cast<std::size_t>(iy) * fine_size_ + ix;
    if (layer >= 0) {
      return fine_[layer][index];
    }
    double best = 0.0;
    for (int i = 0; i < kLayers; ++i) {
      best = std::max(best, static_cast<double>(fine_[i][index]));
    }
    return best;
  }
  double coarseAt(int layer, double x, double y) const
  {
    const int ix = fineIndex(x - origin_x_) / coarse_factor_;
    const int iy = fineIndex(y - origin_y_) / coarse_factor_;
    if (ix < 0 || iy < 0 || ix >= coarse_size_ || iy >= coarse_size_) {
      return 0.0;
    }
    const std::size_t index = static_cast<std::size_t>(iy) * coarse_size_ + ix;
    if (layer >= 0) {
      return coarse_[layer][index];
    }
    double best = 0.0;
    for (int i = 0; i < kLayers; ++i) {
      best = std::max(best, static_cast<double>(coarse_[i][index]));
    }
    return best;
  }

  Params params_;
  double resolution_{0.05};
  int coarse_factor_{10};
  double origin_x_{0.0};
  double origin_y_{0.0};
  int fine_size_{0};
  int coarse_size_{0};
  int stamped_targets_{0};
  static constexpr int kLayers = 3;
  std::vector<float> fine_[kLayers];
  std::vector<float> coarse_[kLayers];
  std::vector<std::uint8_t> coverage_;
  mutable std::vector<RotatedPoint> rotated_;
};

}  // namespace hyu_localization

#endif  // HYU_LOCALIZATION__CORRELATION_GRID_HPP_
