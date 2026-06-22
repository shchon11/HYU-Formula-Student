/*MIT License
 *
 * Copyright (c) 2019 Edinburgh University Formula Student (EUFS)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 *         of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 *         to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *         copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 *         copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *         AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.*/

#ifndef EUFS_PLUGINS_GAZEBO_CONE_PLUGINS_INCLUDE_GAZEBO_CONE_PLUGINS_CONE_MARKERS_HPP_
#define EUFS_PLUGINS_GAZEBO_CONE_PLUGINS_INCLUDE_GAZEBO_CONE_PLUGINS_CONE_MARKERS_HPP_

#include <eufs_msgs/msg/cone_array_with_covariance.hpp>
#include <eufs_msgs/msg/cone_with_covariance.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <std_msgs/msg/header.hpp>
#include <string>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace gazebo_plugins {
namespace eufs_plugins {
namespace cone_markers {

inline std_msgs::msg::ColorRGBA makeColor(float r, float g, float b, float a)
{
  std_msgs::msg::ColorRGBA color;
  color.r = r;
  color.g = g;
  color.b = b;
  color.a = a;
  return color;
}

inline visualization_msgs::msg::Marker makeClearMarker(const std_msgs::msg::Header & header)
{
  visualization_msgs::msg::Marker marker;
  marker.header = header;
  marker.action = visualization_msgs::msg::Marker::DELETEALL;
  return marker;
}

inline visualization_msgs::msg::Marker makeConeListMarker(
  const std_msgs::msg::Header & header,
  const std::string & ns,
  int id,
  const std::vector<eufs_msgs::msg::ConeWithCovariance> & cones,
  const std_msgs::msg::ColorRGBA & color,
  double scale)
{
  visualization_msgs::msg::Marker marker;
  marker.header = header;
  marker.ns = ns;
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = scale;
  marker.scale.y = scale;
  marker.scale.z = scale;
  marker.color = color;

  for (const auto & cone : cones) {
    geometry_msgs::msg::Point point = cone.point;
    point.z += scale * 0.5;
    marker.points.push_back(point);
  }

  return marker;
}

inline visualization_msgs::msg::MarkerArray fromConeArray(
  const eufs_msgs::msg::ConeArrayWithCovariance & cones)
{
  visualization_msgs::msg::MarkerArray markers;
  markers.markers.push_back(makeClearMarker(cones.header));
  markers.markers.push_back(
    makeConeListMarker(cones.header, "blue", 0, cones.blue_cones,
                       makeColor(0.0, 0.2, 1.0, 0.95), 0.25));
  markers.markers.push_back(
    makeConeListMarker(cones.header, "yellow", 1, cones.yellow_cones,
                       makeColor(1.0, 0.85, 0.0, 0.95), 0.25));
  markers.markers.push_back(
    makeConeListMarker(cones.header, "orange", 2, cones.orange_cones,
                       makeColor(1.0, 0.45, 0.0, 0.95), 0.25));
  markers.markers.push_back(
    makeConeListMarker(cones.header, "big_orange", 3, cones.big_orange_cones,
                       makeColor(1.0, 0.25, 0.0, 0.95), 0.35));
  markers.markers.push_back(
    makeConeListMarker(cones.header, "unknown", 4, cones.unknown_color_cones,
                       makeColor(0.8, 0.8, 0.8, 0.95), 0.25));
  return markers;
}

}  // namespace cone_markers
}  // namespace eufs_plugins
}  // namespace gazebo_plugins

#endif  // EUFS_PLUGINS_GAZEBO_CONE_PLUGINS_INCLUDE_GAZEBO_CONE_PLUGINS_CONE_MARKERS_HPP_
