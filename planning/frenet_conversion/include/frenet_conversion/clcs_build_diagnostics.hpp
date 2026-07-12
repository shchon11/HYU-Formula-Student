#pragma once

#include "clcs_frenet_converter.hpp"
#include "rclcpp/logger.hpp"

namespace frenet_conversion
{

void logClcsBuildStats(const rclcpp::Logger & logger, const ClcsBuildStats & stats);

}
