#include <cmath>
#include <limits>

#include "gtest/gtest.h"
#include "hyu_tmpc_state_bridge/heading_conversion.hpp"

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kTolerance = 1.0e-12;

TEST(HeadingConversion, MapsRosCardinalHeadingsToFormulaConvention)
{
  EXPECT_NEAR(
    hyu_tmpc_state_bridge::RosYawToFormulaHeading(0.0),
    -0.5 * kPi, kTolerance);  // east
  EXPECT_NEAR(
    hyu_tmpc_state_bridge::RosYawToFormulaHeading(0.5 * kPi),
    0.0, kTolerance);  // north
  EXPECT_NEAR(
    hyu_tmpc_state_bridge::RosYawToFormulaHeading(kPi),
    0.5 * kPi, kTolerance);  // west
  EXPECT_NEAR(
    hyu_tmpc_state_bridge::RosYawToFormulaHeading(-0.5 * kPi),
    -kPi, kTolerance);  // south
}

TEST(HeadingConversion, NormalizesAcrossPlusMinusPiBoundary)
{
  EXPECT_NEAR(
    hyu_tmpc_state_bridge::RosYawToFormulaHeading(2.0 * kPi + 0.5 * kPi),
    0.0, kTolerance);
  EXPECT_NEAR(
    hyu_tmpc_state_bridge::RosYawToFormulaHeading(-2.0 * kPi + 0.5 * kPi),
    0.0, kTolerance);
  EXPECT_NEAR(hyu_tmpc_state_bridge::NormalizeAngle(kPi), -kPi, kTolerance);
  EXPECT_NEAR(hyu_tmpc_state_bridge::NormalizeAngle(-kPi), -kPi, kTolerance);
}

TEST(HeadingConversion, PreservesNonFiniteInputsForCallerValidation)
{
  EXPECT_TRUE(std::isnan(hyu_tmpc_state_bridge::RosYawToFormulaHeading(
    std::numeric_limits<double>::quiet_NaN())));
  EXPECT_TRUE(std::isinf(hyu_tmpc_state_bridge::RosYawToFormulaHeading(
    std::numeric_limits<double>::infinity())));
}
}  // namespace
