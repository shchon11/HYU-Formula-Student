#pragma once

#include <cstddef>
#include <filesystem>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace hyu_global_planner
{

struct Xy
{
  double x{0.0};
  double y{0.0};
};

// One rendered layer: which file to read, how to parse it, and its RViz color.
struct LayerSpec
{
  std::string ns;
  std::string param_name;      // parameter that overrides the filename
  std::string default_file;    // filename relative to the resolved map directory
  char delimiter{','};
  std::size_t x_index{0};
  std::size_t y_index{1};
  float r{1.0F};
  float g{1.0F};
  float b{1.0F};
};

struct FileState
{
  bool exists{false};
  std::filesystem::file_time_type mtime;
};

class GlobalPlannerDebugVisualizerNode : public rclcpp::Node
{
public:
  GlobalPlannerDebugVisualizerNode();

private:
  void declareParameters();
  void loadParameters();
  void buildLayerSpecs();
  std::filesystem::path resolveMapDir() const;
  std::filesystem::path layerPath(const LayerSpec & layer) const;
  FileState statFile(const std::filesystem::path & path) const;
  bool loadXyCsv(
    const LayerSpec & layer, std::vector<Xy> & points, std::string & error_message) const;
  visualization_msgs::msg::Marker buildLayerMarker(
    int id, const LayerSpec & layer, const std::vector<Xy> & points) const;
  void checkAndReload();

  std::string output_root_;
  std::string map_name_;
  std::string frame_id_;
  std::string markers_topic_;
  double marker_line_width_{0.1};
  double reload_period_sec_{1.0};
  bool close_loops_{true};

  std::vector<LayerSpec> layers_;
  std::vector<FileState> layer_states_;
  bool published_once_{false};
  visualization_msgs::msg::MarkerArray last_markers_;
  bool has_last_markers_{false};

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
  rclcpp::TimerBase::SharedPtr reload_timer_;
};

}  // namespace hyu_global_planner
