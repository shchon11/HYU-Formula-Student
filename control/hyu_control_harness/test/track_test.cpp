#include <gtest/gtest.h>

#include <cmath>

#include "hyu_control_harness/track.hpp"

namespace ch = hyu_control_harness;

namespace
{

// Circular test track: centerline radius 15 m, width 3.5 m, cones every ~10 deg.
ch::Track circularTrack()
{
  ch::Track track;
  constexpr double kRadius = 15.0;
  constexpr double kHalfWidth = 1.75;
  constexpr int kCones = 36;
  for (int i = 0; i < kCones; ++i) {
    const double angle = 2.0 * M_PI * static_cast<double>(i) / kCones;
    track.blue.push_back(
      {(kRadius + kHalfWidth) * std::cos(angle), (kRadius + kHalfWidth) * std::sin(angle)});
    track.yellow.push_back(
      {(kRadius - kHalfWidth) * std::cos(angle), (kRadius - kHalfWidth) * std::sin(angle)});
  }
  track.start = {kRadius, 0.0, M_PI / 2.0};
  return track;
}

}  // namespace

TEST(Centerline, CircularTrackGeometry)
{
  const auto centerline = ch::Centerline::build(circularTrack());
  ASSERT_TRUE(centerline.valid());
  // Ring length ~ 2*pi*15 (slightly shorter: polygon chord approximation).
  EXPECT_NEAR(centerline.length(), 2.0 * M_PI * 15.0, 1.5);

  // On the centerline: zero cte, half width 1.75.
  const auto on_line = centerline.project({15.0, 0.0});
  EXPECT_NEAR(on_line.cte_m, 0.0, 0.1);
  EXPECT_NEAR(on_line.half_width_m, 1.75, 0.05);

  // 1 m outside the centerline radius: cte ~ 1.
  const auto offset = centerline.project({16.0, 0.0});
  EXPECT_NEAR(offset.cte_m, 1.0, 0.15);
}

TEST(Centerline, WrappedDeltaHandlesTheSeam)
{
  const auto centerline = ch::Centerline::build(circularTrack());
  ASSERT_TRUE(centerline.valid());
  const double length = centerline.length();
  EXPECT_NEAR(centerline.wrappedDelta(length - 1.0, 1.0), 2.0, 1e-9);
  EXPECT_NEAR(centerline.wrappedDelta(1.0, length - 1.0), -2.0, 1e-9);
  EXPECT_NEAR(centerline.wrappedDelta(10.0, 12.5), 2.5, 1e-9);
}

TEST(Track, MissingFileFailsCleanly)
{
  EXPECT_FALSE(ch::loadTrackCsv("/nonexistent/track.csv").has_value());
}
