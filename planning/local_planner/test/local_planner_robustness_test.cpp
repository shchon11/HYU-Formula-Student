#include <cstddef>

#include <gtest/gtest.h>

#include "local_planner/local_path_builder.hpp"

namespace local_planner
{

TEST(LocalPathBuilderRobustness, OversizedBoundaryInputFailsClosed)
{
  ConeSet cones;
  for (std::size_t index = 0; index <= kMaxBoundaryCones; ++index) {
    cones.blue.push_back({0.01 * static_cast<double>(index), 1.5});
  }

  const auto result = buildLocalPath(cones);

  ASSERT_TRUE(result.evaluated);
  EXPECT_FALSE(result.valid);
  EXPECT_TRUE(result.waypoints.empty());
  EXPECT_EQ(result.reason, "cone input exceeds bounded planner capacity");
}

TEST(LocalPathBuilderRobustness, ExcessiveGapHeadingAndSelfIntersectionFailClosed)
{
  ConeSet gap;
  gap.blue = {{0.0, 1.5}, {1.0, 1.5}, {8.0, 1.5}};
  const auto gap_result = buildLocalPath(gap);
  ASSERT_TRUE(gap_result.evaluated);
  EXPECT_FALSE(gap_result.valid);

  ConeSet heading;
  heading.yellow = {{0.0, -1.5}, {1.0, -1.5}, {1.1, 2.0}};
  const auto heading_result = buildLocalPath(heading);
  ASSERT_TRUE(heading_result.evaluated);
  EXPECT_FALSE(heading_result.valid);

  EXPECT_TRUE(pathSelfIntersects({{0.0, 0.0}, {2.0, 2.0}, {0.0, 2.0}, {2.0, 0.0}}));
}

}
