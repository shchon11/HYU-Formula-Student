#pragma once

#include <Eigen/Core>

#include <g2o/core/base_binary_edge.h>
#include <g2o/types/slam2d/vertex_se2.h>

namespace hyu_localization
{

// GNSS fix as a measurement of the GAUGE-transformed pose. The fix lives in
// the ENU/absolute frame; the keyframe pose x lives in the map frame; a single
// SE2 gauge vertex g models the map->ENU registration. Predicted fix position
// is (g o x).translation(), so the edge constrains poses AND the gauge
// jointly:
//   - RTK healthy: many tight edges observe g, it converges and stays put --
//     poses get the usual absolute anchoring THROUGH g.
//   - GNSS outage: no edges arrive, g literally floats (the gauge-freedom
//     state), held only by its weak identity prior.
//   - Recovery after drift: in mapping the pre/post-outage edges negotiate
//     through g and the odometry chain redistributes the drift (loop-closure
//     behaviour); in localization the frozen landmarks hold the poses, so the
//     residual moves g instead of tearing the pose between GNSS and the map.
//   - Colored single-grade bias: lands in g (bounded by g's random-walk-ish
//     weak prior) instead of dragging the pose against the cone map.
// This replaces the direct EdgeSE2XYPrior, whose absolute pull is exactly the
// pose-vs-map fight that shredded association under degraded GNSS.
class EdgeSE2GaugeXY
  : public g2o::BaseBinaryEdge<2, Eigen::Vector2d, g2o::VertexSE2, g2o::VertexSE2>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  void computeError() override
  {
    const auto * pose = static_cast<const g2o::VertexSE2 *>(_vertices[0]);
    const auto * gauge = static_cast<const g2o::VertexSE2 *>(_vertices[1]);
    const g2o::SE2 predicted = gauge->estimate() * pose->estimate();
    _error = predicted.translation() - _measurement;
  }

  // Graph persistence is not used in this stack.
  bool read(std::istream &) override {return false;}
  bool write(std::ostream &) const override {return false;}
};

}  // namespace hyu_localization
