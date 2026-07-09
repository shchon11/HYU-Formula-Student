#include "global_planner_debug_visualizer_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <functional>
#include <sstream>
#include <system_error>

#include "geometry_msgs/msg/point.hpp"

namespace global_planner
{
namespace
{

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

std::vector<std::string> splitDelimiter(const std::string & line, char delimiter)
{
  std::vector<std::string> tokens;
  std::stringstream stream(line);
  std::string token;
  while (std::getline(stream, token, delimiter)) {
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

}  // namespace

GlobalPlannerDebugVisualizerNode::GlobalPlannerDebugVisualizerNode()
: Node("global_planner_debug_visualizer_node")
{
  declareParameters();
  loadParameters();
  buildLayerSpecs();

  const auto latched_qos = rclcpp::QoS(1).reliable().transient_local();
  markers_pub_ =
    create_publisher<visualization_msgs::msg::MarkerArray>(markers_topic_, latched_qos);

  RCLCPP_INFO(get_logger(), "markers_topic=%s", markers_topic_.c_str());
  RCLCPP_INFO(get_logger(), "map directory: %s", resolveMapDir().string().c_str());

  layer_states_.assign(layers_.size(), FileState{});

  checkAndReload();

  const auto period = std::chrono::duration<double>(std::max(0.1, reload_period_sec_));
  reload_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&GlobalPlannerDebugVisualizerNode::checkAndReload, this));
}

void GlobalPlannerDebugVisualizerNode::declareParameters()
{
  declare_parameter<std::string>("output_root", "");
  declare_parameter<std::string>("map_name", "");
  declare_parameter<std::string>("frame_id", "map");
  declare_parameter<std::string>("markers_topic", "/global_planner/debug/markers");
  declare_parameter<double>("marker_line_width", 0.1);
  declare_parameter<double>("reload_period_sec", 1.0);
  declare_parameter<bool>("close_loops", true);
  // Per-layer filename overrides (relative to <output_root>/<map_name>).
  declare_parameter<std::string>("left_boundary_filename", "left_boundary_spline.csv");
  declare_parameter<std::string>("right_boundary_filename", "right_boundary_spline.csv");
  declare_parameter<std::string>("centerline_filename", "centerline.csv");
  declare_parameter<std::string>("raceline_filename", "traj_race_cl.csv");
}

void GlobalPlannerDebugVisualizerNode::loadParameters()
{
  output_root_ = get_parameter("output_root").as_string();
  map_name_ = get_parameter("map_name").as_string();
  frame_id_ = get_parameter("frame_id").as_string();
  markers_topic_ = get_parameter("markers_topic").as_string();
  marker_line_width_ = get_parameter("marker_line_width").as_double();
  reload_period_sec_ = get_parameter("reload_period_sec").as_double();
  close_loops_ = get_parameter("close_loops").as_bool();

  if (marker_line_width_ <= 0.0) {
    RCLCPP_WARN(get_logger(), "marker_line_width must be > 0. Using 0.1.");
    marker_line_width_ = 0.1;
  }
}

void GlobalPlannerDebugVisualizerNode::buildLayerSpecs()
{
  // Formula Student colours: blue = left boundary, yellow = right boundary.
  // centerline = red, minimum-curvature raceline = green.
  layers_.clear();
  layers_.push_back(
    LayerSpec{"left_boundary", "left_boundary_filename",
      get_parameter("left_boundary_filename").as_string(), ',', 0, 1, 0.1F, 0.1F, 1.0F});
  layers_.push_back(
    LayerSpec{"right_boundary", "right_boundary_filename",
      get_parameter("right_boundary_filename").as_string(), ',', 0, 1, 1.0F, 0.9F, 0.1F});
  layers_.push_back(
    LayerSpec{"centerline", "centerline_filename",
      get_parameter("centerline_filename").as_string(), ',', 0, 1, 1.0F, 0.1F, 0.1F});
  layers_.push_back(
    LayerSpec{"min_curvature", "raceline_filename",
      get_parameter("raceline_filename").as_string(), ';', 1, 2, 0.1F, 1.0F, 0.1F});
}

std::filesystem::path GlobalPlannerDebugVisualizerNode::resolveMapDir() const
{
  return std::filesystem::path(output_root_) / std::filesystem::path(map_name_);
}

std::filesystem::path GlobalPlannerDebugVisualizerNode::layerPath(const LayerSpec & layer) const
{
  return resolveMapDir() / std::filesystem::path(layer.default_file);
}

FileState GlobalPlannerDebugVisualizerNode::statFile(const std::filesystem::path & path) const
{
  FileState state;
  std::error_code exists_error;
  state.exists = std::filesystem::exists(path, exists_error) && !exists_error;
  if (state.exists) {
    std::error_code time_error;
    state.mtime = std::filesystem::last_write_time(path, time_error);
    if (time_error) {
      state.exists = false;
    }
  }
  return state;
}

bool GlobalPlannerDebugVisualizerNode::loadXyCsv(
  const LayerSpec & layer,
  std::vector<Xy> & points,
  std::string & error_message) const
{
  const auto path = layerPath(layer);
  std::ifstream file(path);
  if (!file.is_open()) {
    error_message = "failed to open " + path.string();
    return false;
  }

  points.clear();
  const std::size_t needed_columns = std::max(layer.x_index, layer.y_index) + 1U;

  std::string line;
  std::size_t line_number = 0;
  while (std::getline(file, line)) {
    ++line_number;
    line = trim(removeUtf8Bom(line));
    if (line.empty() || line.front() == '#') {
      continue;
    }

    const auto tokens = splitDelimiter(line, layer.delimiter);
    if (tokens.size() < needed_columns) {
      continue;
    }

    Xy point;
    if (!parseFiniteDouble(tokens[layer.x_index], point.x) ||
      !parseFiniteDouble(tokens[layer.y_index], point.y))
    {
      // Non-numeric header rows (e.g. "x_m,y_m") land here and are skipped.
      continue;
    }
    points.push_back(point);
  }

  if (points.size() < 2U) {
    error_message = "fewer than 2 usable points parsed from " + path.string();
    return false;
  }
  return true;
}

visualization_msgs::msg::Marker GlobalPlannerDebugVisualizerNode::buildLayerMarker(
  int id,
  const LayerSpec & layer,
  const std::vector<Xy> & points) const
{
  visualization_msgs::msg::Marker marker;
  // Static map layers should render against the latest transform in RViz,
  // independent of whether the simulator and this node share sim time.
  marker.header.stamp.sec = 0;
  marker.header.stamp.nanosec = 0;
  marker.header.frame_id = frame_id_;
  marker.ns = layer.ns;
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker.action = points.empty() ?
    visualization_msgs::msg::Marker::DELETE :
    visualization_msgs::msg::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = marker_line_width_;
  marker.color.r = layer.r;
  marker.color.g = layer.g;
  marker.color.b = layer.b;
  marker.color.a = 1.0F;

  marker.points.reserve(points.size() + 1U);
  for (const auto & point : points) {
    geometry_msgs::msg::Point marker_point;
    marker_point.x = point.x;
    marker_point.y = point.y;
    marker_point.z = 0.0;
    marker.points.push_back(marker_point);
  }
  if (close_loops_ && marker.points.size() >= 2U) {
    marker.points.push_back(marker.points.front());
  }
  return marker;
}

void GlobalPlannerDebugVisualizerNode::checkAndReload()
{
  std::vector<FileState> current(layers_.size());
  bool changed = !published_once_;
  for (std::size_t i = 0; i < layers_.size(); ++i) {
    current[i] = statFile(layerPath(layers_[i]));
    const auto & previous = layer_states_[i];
    if (current[i].exists != previous.exists ||
      (current[i].exists && current[i].mtime != previous.mtime))
    {
      changed = true;
    }
  }
  if (!changed) {
    if (has_last_markers_) {
      markers_pub_->publish(last_markers_);
    }
    return;
  }

  visualization_msgs::msg::MarkerArray markers;
  std::size_t published_layers = 0;
  for (std::size_t i = 0; i < layers_.size(); ++i) {
    const auto & layer = layers_[i];
    std::vector<Xy> points;
    std::string error_message;
    if (current[i].exists && loadXyCsv(layer, points, error_message)) {
      markers.markers.push_back(buildLayerMarker(static_cast<int>(i), layer, points));
      ++published_layers;
      RCLCPP_INFO(
        get_logger(), "Layer '%s': %zu point(s) from %s",
        layer.ns.c_str(), points.size(), layer.default_file.c_str());
    } else {
      // Emit a DELETE so RViz clears a layer whose file vanished or went invalid.
      markers.markers.push_back(buildLayerMarker(static_cast<int>(i), layer, {}));
      if (!error_message.empty()) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000, "Layer '%s' unavailable: %s",
          layer.ns.c_str(), error_message.c_str());
      }
    }
  }

  last_markers_ = markers;
  has_last_markers_ = true;
  markers_pub_->publish(last_markers_);
  layer_states_ = current;
  published_once_ = true;
  RCLCPP_INFO(
    get_logger(), "Published debug markers to %s (%zu/%zu layer(s) drawn)",
    markers_topic_.c_str(), published_layers, layers_.size());
}

}  // namespace global_planner

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<global_planner::GlobalPlannerDebugVisualizerNode>());
  rclcpp::shutdown();
  return 0;
}
