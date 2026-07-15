// Copyright 2026 shchon11
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "gazebo_race_car_model/terrain_field.hpp"

#include <cmath>
#include <string>

#include <gtest/gtest.h>

namespace {

using gazebo_plugins::eufs_plugins::TerrainField;

TerrainField::Config baseConfig() {
  TerrainField::Config config;
  config.enabled = true;
  config.seed = 7u;
  config.density = 0.05;
  config.half_extent_x = 20.0;
  config.half_extent_y = 20.0;
  config.height_mean = 0.03;
  config.height_sigma = 0.005;
  config.radius_min = 0.30;
  config.radius_max = 0.50;
  return config;
}

TEST(TerrainField, DisabledIsBareAsphalt) {
  TerrainField field;
  TerrainField::Config config = baseConfig();
  config.enabled = false;
  field.generate(config);

  EXPECT_TRUE(field.empty());
  EXPECT_DOUBLE_EQ(field.height(0.0, 0.0), 0.0);
  EXPECT_DOUBLE_EQ(field.height(12.3, -4.5), 0.0);
}

// The whole promise of a field over the old road noise: a bump is a place. Two
// runs of the same seed must agree, or a regression test on it means nothing.
TEST(TerrainField, SameSeedIsTheSameTrack) {
  TerrainField a, b;
  a.generate(baseConfig());
  b.generate(baseConfig());

  ASSERT_FALSE(a.empty());
  ASSERT_EQ(a.bumps().size(), b.bumps().size());
  for (std::size_t i = 0; i < a.bumps().size(); ++i) {
    EXPECT_DOUBLE_EQ(a.bumps()[i].x, b.bumps()[i].x);
    EXPECT_DOUBLE_EQ(a.bumps()[i].y, b.bumps()[i].y);
    EXPECT_DOUBLE_EQ(a.bumps()[i].height, b.bumps()[i].height);
    EXPECT_DOUBLE_EQ(a.bumps()[i].radius, b.bumps()[i].radius);
  }
}

TEST(TerrainField, DifferentSeedIsADifferentTrack) {
  TerrainField a, b;
  TerrainField::Config config = baseConfig();
  a.generate(config);
  config.seed = 8u;
  b.generate(config);

  ASSERT_FALSE(a.empty());
  ASSERT_FALSE(b.empty());
  EXPECT_NE(a.bumps().front().x, b.bumps().front().x);
}

// height() must be the surface the mesh renders, because the LiDAR sees the mesh
// and the car rides height(). Peak, rim and outside are the three points where
// disagreement would be obvious.
TEST(TerrainField, ProfileMatchesTheRaisedCosine) {
  TerrainField field;
  field.generate(baseConfig());
  ASSERT_FALSE(field.empty());

  const TerrainField::Bump &bump = field.bumps().front();

  EXPECT_NEAR(field.height(bump.x, bump.y), bump.height, 1e-9);
  EXPECT_NEAR(field.height(bump.x + bump.radius * 0.999, bump.y), 0.0, 1e-4);
  EXPECT_DOUBLE_EQ(field.height(bump.x + bump.radius * 1.001, bump.y), 0.0);

  // Half a radius out, a raised cosine is at exactly half its peak.
  EXPECT_NEAR(field.height(bump.x + bump.radius * 0.5, bump.y), bump.height * 0.5, 1e-9);
}

TEST(TerrainField, HeightIsNeverAHole) {
  TerrainField field;
  TerrainField::Config config = baseConfig();
  // A mean this close to zero makes the normal draw go negative often, which
  // without the floor would invert a bump into a pit the mesh cannot render.
  config.height_mean = 0.002;
  config.height_sigma = 0.02;
  config.height_min = 0.004;
  field.generate(config);

  ASSERT_FALSE(field.empty());
  for (const TerrainField::Bump &bump : field.bumps()) {
    EXPECT_GE(bump.height, config.height_min);
  }
  for (double x = -20.0; x <= 20.0; x += 0.37) {
    for (double y = -20.0; y <= 20.0; y += 0.41) {
      EXPECT_GE(field.height(x, y), 0.0);
    }
  }
}

// Overlapping domes are a union of solids, so the surface is the higher of the
// two. Summing would build a spike where the geometry has none, and the car
// would climb something the LiDAR cannot see.
TEST(TerrainField, OverlappingBumpsTakeTheHigherSurface) {
  TerrainField field;
  TerrainField::Config config = baseConfig();
  config.density = 2.0;  // dense enough to guarantee overlaps
  config.half_extent_x = 3.0;
  config.half_extent_y = 3.0;
  field.generate(config);
  ASSERT_GT(field.bumps().size(), 2u);

  double tallest = 0.0;
  for (const TerrainField::Bump &bump : field.bumps()) {
    tallest = std::max(tallest, bump.height);
  }
  for (double x = -3.0; x <= 3.0; x += 0.05) {
    for (double y = -3.0; y <= 3.0; y += 0.05) {
      EXPECT_LE(field.height(x, y), tallest + 1e-9);
    }
  }
}

TEST(TerrainField, BumpsStayNearTheTrack) {
  TerrainField field;
  TerrainField::Config config = baseConfig();
  config.density = 0.5;
  config.track_margin = 2.0;
  config.track_points = {{0.0, 0.0}, {5.0, 0.0}, {10.0, 3.0}};
  field.generate(config);

  ASSERT_FALSE(field.empty());
  for (const TerrainField::Bump &bump : field.bumps()) {
    double nearest = 1.0e9;
    for (const auto &point : config.track_points) {
      nearest = std::min(nearest, std::hypot(bump.x - point.first, bump.y - point.second));
    }
    EXPECT_LE(nearest, config.track_margin + 1e-9);
  }
}

// Bounding to the track must not change what `density` means, or tuning it
// against one track would mislead on the next.
TEST(TerrainField, TrackBoundIsFarCheaperThanTheRectangle) {
  TerrainField bounded, whole;
  TerrainField::Config config = baseConfig();
  config.density = 0.5;
  config.track_margin = 2.0;
  config.track_points = {{0.0, 0.0}, {5.0, 0.0}, {10.0, 3.0}};
  bounded.generate(config);

  config.track_points.clear();
  whole.generate(config);

  EXPECT_LT(bounded.bumps().size(), whole.bumps().size() / 10);
}

TEST(TerrainField, SdfDescribesEveryBumpOnce) {
  TerrainField field;
  field.generate(baseConfig());
  ASSERT_FALSE(field.empty());

  const std::string sdf = field.sdf("eufs_terrain", "file:///tmp/bump_dome.stl");

  EXPECT_NE(sdf.find("<model name='eufs_terrain'>"), std::string::npos);
  EXPECT_NE(sdf.find("<static>true</static>"), std::string::npos);

  std::size_t collisions = 0;
  for (std::size_t at = sdf.find("<collision name='bump_"); at != std::string::npos;
       at = sdf.find("<collision name='bump_", at + 1)) {
    ++collisions;
  }
  EXPECT_EQ(collisions, field.bumps().size());

  // The mesh is scaled per bump; a uniform scale would make every bump as tall
  // as it is wide.
  const TerrainField::Bump &bump = field.bumps().front();
  EXPECT_NE(bump.radius, bump.height);
  EXPECT_NE(sdf.find("/tmp/bump_dome.stl"), std::string::npos);
}

}  // namespace

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
