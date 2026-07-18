#include "hyu_global_planner/trajectory_csv.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <sstream>

#include "rclcpp/rclcpp.hpp"

namespace hyu_global_planner
{
namespace
{

constexpr std::size_t kExpectedColumnCount = 7;
// Optional trailing columns extend the 7-column trajectory_generator layout.
constexpr std::size_t kMaxColumnCount = 9;
constexpr std::array<const char *, kMaxColumnCount> kExpectedHeader = {
  "s_m", "x_m", "y_m", "psi_rad", "kappa_radpm", "vx_mps", "ax_mps2",
  "d_left_m", "d_right_m"};

std::string trim(const std::string & value)
{
  const auto first = value.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) {
    return "";
  }
  const auto last = value.find_last_not_of(" \t\r\n");
  return value.substr(first, last - first + 1);
}

std::string removeUtf8Bom(std::string value)
{
  if (value.size() >= 3U &&
    static_cast<unsigned char>(value[0]) == 0xEF &&
    static_cast<unsigned char>(value[1]) == 0xBB &&
    static_cast<unsigned char>(value[2]) == 0xBF)
  {
    value.erase(0, 3);
  }
  return value;
}

std::vector<std::string> splitSemicolon(const std::string & line)
{
  std::vector<std::string> tokens;
  std::stringstream stream(line);
  std::string token;
  while (std::getline(stream, token, ';')) {
    tokens.push_back(trim(token));
  }
  return tokens;
}

bool parseFiniteDouble(const std::string & token, double & output)
{
  try {
    std::size_t parsed_chars = 0;
    const double value = std::stod(token, &parsed_chars);
    if (parsed_chars != token.size() || !std::isfinite(value)) {
      return false;
    }
    output = value;
    return true;
  } catch (const std::exception &) {
    return false;
  }
}

bool isExpectedHeader(const std::vector<std::string> & tokens)
{
  if (tokens.size() != kExpectedColumnCount && tokens.size() != kMaxColumnCount) {
    return false;
  }
  for (std::size_t i = 0; i < tokens.size(); ++i) {
    if (tokens[i] != kExpectedHeader[i]) {
      return false;
    }
  }
  return true;
}

double pointDistance(const TrajectoryPoint & a, const TrajectoryPoint & b)
{
  return std::hypot(b.x - a.x, b.y - a.y);
}

bool parseTrajectoryRow(const std::vector<std::string> & tokens, TrajectoryPoint & point)
{
  if (!(parseFiniteDouble(tokens[0], point.s) &&
    parseFiniteDouble(tokens[1], point.x) &&
    parseFiniteDouble(tokens[2], point.y) &&
    parseFiniteDouble(tokens[3], point.psi) &&
    parseFiniteDouble(tokens[4], point.kappa) &&
    parseFiniteDouble(tokens[5], point.velocity) &&
    parseFiniteDouble(tokens[6], point.acceleration)))
  {
    return false;
  }
  if (tokens.size() == kMaxColumnCount) {
    return parseFiniteDouble(tokens[7], point.d_left) &&
           parseFiniteDouble(tokens[8], point.d_right);
  }
  return true;
}

}  // namespace

bool loadTrajectoryCsv(
  const std::filesystem::path & path,
  const rclcpp::Logger & logger,
  std::vector<TrajectoryPoint> & points,
  std::string & error_message)
{
  std::ifstream file(path);
  if (!file.is_open()) {
    error_message = "failed to open trajectory CSV: " + path.string();
    return false;
  }

  points.clear();
  bool header_seen = false;
  std::string line;
  std::size_t line_number = 0;
  std::size_t malformed_rows = 0;
  while (std::getline(file, line)) {
    ++line_number;
    line = trim(removeUtf8Bom(line));
    if (line.empty()) {
      continue;
    }
    if (!line.empty() && line.front() == '#') {
      line = trim(line.substr(1));
      if (line.empty()) {
        continue;
      }
    }

    const auto tokens = splitSemicolon(line);
    if (!header_seen) {
      if (isExpectedHeader(tokens)) {
        header_seen = true;
        continue;
      }
      if (tokens.size() == kExpectedColumnCount && tokens.front() == kExpectedHeader.front()) {
        error_message = "CSV header invalid at line " + std::to_string(line_number);
        return false;
      }
      continue;
    }

    if (tokens.size() != kExpectedColumnCount && tokens.size() != kMaxColumnCount) {
      ++malformed_rows;
      RCLCPP_WARN(
        logger, "Skipping malformed trajectory row %zu: expected %zu or %zu columns, got %zu",
        line_number, kExpectedColumnCount, kMaxColumnCount, tokens.size());
      continue;
    }

    TrajectoryPoint point;
    if (!parseTrajectoryRow(tokens, point)) {
      ++malformed_rows;
      RCLCPP_WARN(
        logger, "Skipping malformed trajectory row %zu: numeric conversion failed or non-finite value",
        line_number);
      continue;
    }
    points.push_back(point);
  }

  if (!header_seen) {
    error_message = "CSV header invalid or missing";
    return false;
  }
  if (malformed_rows > 0U) {
    RCLCPP_WARN(logger, "Skipped %zu malformed trajectory row(s).", malformed_rows);
  }
  return true;
}

bool validateTrajectory(
  std::vector<TrajectoryPoint> & points,
  const TrajectoryValidationOptions & options,
  const rclcpp::Logger & logger,
  std::string & error_message)
{
  if (points.size() < static_cast<std::size_t>(options.min_waypoint_count)) {
    error_message =
      "valid waypoint count is below min_waypoint_count: " + std::to_string(points.size());
    return false;
  }

  std::vector<TrajectoryPoint> deduplicated;
  deduplicated.reserve(points.size());
  deduplicated.push_back(points.front());
  std::size_t duplicate_count = 0;
  for (std::size_t i = 1; i < points.size(); ++i) {
    if (!std::isfinite(points[i].s) || !std::isfinite(points[i].x) || !std::isfinite(points[i].y)) {
      error_message = "trajectory contains non-finite s/x/y after parsing";
      return false;
    }
    if (pointDistance(deduplicated.back(), points[i]) <= options.duplicate_point_tolerance) {
      ++duplicate_count;
      continue;
    }
    deduplicated.push_back(points[i]);
  }

  if (duplicate_count > 0U) {
    RCLCPP_WARN(
      logger, "Removed %zu duplicate trajectory waypoint(s); remaining=%zu",
      duplicate_count, deduplicated.size());
  }
  if (deduplicated.size() < static_cast<std::size_t>(options.min_waypoint_count)) {
    error_message =
      "waypoint count after duplicate removal is below min_waypoint_count: " +
      std::to_string(deduplicated.size());
    return false;
  }

  const bool s_is_strictly_increasing = std::adjacent_find(
    deduplicated.begin(), deduplicated.end(),
    [](const TrajectoryPoint & a, const TrajectoryPoint & b) {return !(b.s > a.s);}) ==
    deduplicated.end();
  if (!s_is_strictly_increasing) {
    if (!options.recompute_s_if_invalid) {
      error_message = "trajectory s_m is not strictly increasing";
      return false;
    }
    deduplicated.front().s = 0.0;
    for (std::size_t i = 1; i < deduplicated.size(); ++i) {
      deduplicated[i].s = deduplicated[i - 1].s +
        pointDistance(deduplicated[i - 1], deduplicated[i]);
    }
    RCLCPP_WARN(logger, "Invalid s_m sequence detected; recomputed s from x/y distance.");
  }

  points = std::move(deduplicated);
  return true;
}

}  // namespace hyu_global_planner
