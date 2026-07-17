#include <stdexcept>

#include <gtest/gtest.h>

#include "hyu_local_planner/input_policy.hpp"

namespace hyu_local_planner
{
namespace
{

HeaderMetadata header(const std::string & frame, std::int32_t sec, std::uint32_t nanosec, double age)
{
  return HeaderMetadata{frame, sec, nanosec, age};
}

OdomMetadata odom(double age = 0.0)
{
  return OdomMetadata{header("map", 10, 0, age), "base_footprint", {1.0, 2.0}, 0.25};
}

TEST(InputPolicy, LiveInputRejectsWrongFramesZeroStampsSkewAndStaleness)
{
  const auto valid = validateLiveInput(header("base_footprint", 10, 50000000U, 0.1), odom());
  ASSERT_TRUE(valid.evaluated);
  EXPECT_TRUE(valid.valid) << valid.reason;

  EXPECT_FALSE(validateLiveInput(header("camera", 10, 0, 0.0), odom()).valid);
  EXPECT_FALSE(validateLiveInput(header("base_footprint", 0, 0, 0.0), odom()).valid);
  EXPECT_FALSE(
    validateLiveInput(header("base_footprint", 10, 110000001U, 0.0), odom()).valid);
  EXPECT_FALSE(
    validateLiveInput(header("base_footprint", 10, 0, 0.51), odom()).valid);
  auto wrong_odom = odom();
  wrong_odom.header.frame_id = "odom";
  EXPECT_FALSE(validateLiveInput(header("base_footprint", 10, 0, 0.0), wrong_odom).valid);
}

TEST(InputPolicy, LiveInputRejectsStaleAndSkewedPairsThenAcceptsFreshRecovery)
{
  const auto fresh_cones = header("base_footprint", 10, 0, 0.0);
  const auto fresh_odom = odom();
  EXPECT_TRUE(validateLiveInput(fresh_cones, fresh_odom).valid);

  EXPECT_FALSE(validateLiveInput(
      header("base_footprint", 10, 0, 0.500001), fresh_odom).valid);
  EXPECT_TRUE(validateLiveInput(fresh_cones, fresh_odom).valid);

  EXPECT_FALSE(validateLiveInput(
      header("base_footprint", 10, 100000001U, 0.0), fresh_odom).valid);
  EXPECT_TRUE(validateLiveInput(fresh_cones, fresh_odom).valid);
}

TEST(InputPolicy, SlamMapUsesLatchedMapAndFreshOdomWithoutAutoSwitch)
{
  const auto latched_map = header("map", 1, 1, 50.0);
  const auto valid = validateSlamMapInput(latched_map, odom(0.1));
  ASSERT_TRUE(valid.evaluated);
  EXPECT_TRUE(valid.valid) << valid.reason;
  EXPECT_TRUE(acceptsSource(SourceMode::kSlamMap, SourceMode::kSlamMap));
  EXPECT_FALSE(acceptsSource(SourceMode::kSlamMap, SourceMode::kLiveCones));
  EXPECT_FALSE(acceptsSource(SourceMode::kLiveCones, SourceMode::kSlamMap));
  EXPECT_THROW(parseSourceMode("automatic"), std::invalid_argument);
  EXPECT_FALSE(validateSlamMapInput(header("base_footprint", 1, 1, 0.0), odom()).valid);
  EXPECT_FALSE(validateSlamMapInput(header("map", 0, 0, 0.0), odom()).valid);
  EXPECT_FALSE(validateSlamMapInput(latched_map, odom(0.51)).valid);
}

}
}
