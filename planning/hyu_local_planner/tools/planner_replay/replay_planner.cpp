// Offline replay of the slam_map-mode local planner over a recorded bag.
// Emulates LocalPlannerInputs (slam mode) + LocalPlannerNode::processSlamMap.
#include <cmath>
#include <cstdio>
#include <cstring>
#include <deque>
#include <map>
#include <string>
#include <vector>

#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <hyu_msgs/msg/cone_array_with_covariance.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/string.hpp>

#include "hyu_local_planner/input_policy.hpp"
#include "hyu_local_planner/local_path_builder.hpp"
#include "hyu_local_planner/ros_inputs.hpp"
#include "local_path_builder_internal.hpp"

using namespace hyu_local_planner;
using ConeArray = hyu_msgs::msg::ConeArrayWithCovariance;
using Odometry = nav_msgs::msg::Odometry;

static int64_t keyOf(const builtin_interfaces::msg::Time & t)
{
  return static_cast<int64_t>(t.sec) * 1000000000LL + t.nanosec;
}

static double yawOf(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

int main(int argc, char ** argv)
{
  if (argc < 3) {
    fprintf(stderr, "usage: %s <bag_dir> <out.jsonl> [--no-live] [--as-unknown] [--merge R] [--max-age S] [--dump-cones]\n", argv[0]);
    return 1;
  }
  std::string bag = argv[1];
  std::string out_path = argv[2];
  bool use_live = true, dump_cones = false, reconcile = false; double explain_t = -1.0; bool explained = false;
  LiveExtensionConfig live_cfg;
  live_cfg.as_unknown = false;
  live_cfg.merge_radius_m = 1.0;
  double live_max_age = 0.4; double cfg_max_dev = 0.5, cfg_max_turn = 1.4, cfg_unk_range = 6.0;
  for (int i = 3; i < argc; ++i) {
    if (!strcmp(argv[i], "--no-live")) use_live = false;
    else if (!strcmp(argv[i], "--as-unknown")) live_cfg.as_unknown = true;
    else if (!strcmp(argv[i], "--merge") && i + 1 < argc) live_cfg.merge_radius_m = atof(argv[++i]);
    else if (!strcmp(argv[i], "--max-age") && i + 1 < argc) live_max_age = atof(argv[++i]);
    else if (!strcmp(argv[i], "--dump-cones")) dump_cones = true;
    else if (!strcmp(argv[i], "--reconcile")) reconcile = true;
    else if (!strcmp(argv[i], "--explain") && i + 1 < argc) explain_t = atof(argv[++i]);
    else if (!strcmp(argv[i], "--max-dev") && i + 1 < argc) cfg_max_dev = atof(argv[++i]);
    else if (!strcmp(argv[i], "--max-turn") && i + 1 < argc) cfg_max_turn = atof(argv[++i]);
    else if (!strcmp(argv[i], "--unk-range") && i + 1 < argc) cfg_unk_range = atof(argv[++i]);
  }

  // hyu_local_planner.yaml (trackdrive) on top of the node defaults.
  PlannerConfig cfg;
  cfg.roi_min_x = -1.0; cfg.roi_max_x = 20.0; cfg.roi_abs_y = 8.0;
  cfg.endpoint_match_tolerance_m = 0.05; cfg.min_track_width_m = 2.0; cfg.max_track_width_m = 6.0;
  cfg.duplicate_tolerance_m = 0.05; cfg.min_forward_projection_m = 0.10; cfg.max_traversal_gap_m = 6.0;
  cfg.centerline_max_link_gap_m = 6.0; cfg.max_heading_change_rad = 1.047; cfg.max_u_turn_heading_change_rad = 2.618;
  cfg.waypoint_spacing_m = 0.5; cfg.max_start_distance_m = 4.0; cfg.two_sided_horizon_m = 20.0;
  cfg.fallback_horizon_m = 8.0; cfg.fallback_offset_m = 1.5; cfg.two_sided_speed_mps = 6.0; cfg.fallback_speed_mps = 4.5;
  cfg.max_lateral_accel_mps2 = 3.0; cfg.min_speed_mps = 1.0; cfg.allow_partial_boundary = true;
  cfg.use_unknown_cones = true; cfg.unknown_absorb_lateral_m = 0.75; cfg.unknown_geom_deadband_m = 0.75;
  cfg.use_orange_cones = true; cfg.extend_straight_to_horizon = false; cfg.stop_at_path_end = false;
  cfg.live_extension_max_deviation_m = cfg_max_dev; cfg.live_extension_max_turn_rad = cfg_max_turn;
  cfg.unknown_geom_max_range_m = cfg_unk_range;
  const double standstill_recovery_speed = 0.5;

  rosbag2_cpp::Reader reader;
  rosbag2_storage::StorageOptions so; so.uri = bag; so.storage_id = "sqlite3";
  reader.open(so);
  rclcpp::Serialization<ConeArray> cones_ser;
  rclcpp::Serialization<Odometry> odom_ser;
  rclcpp::Serialization<std_msgs::msg::String> str_ser;

  FILE * out = fopen(out_path.c_str(), "w");
  if (!out) { perror("fopen"); return 1; }

  // --- emulated LocalPlannerInputs state ---
  ConeArray::SharedPtr latched_map; int64_t latched_map_recv = 0; bool have_map = false;
  ConeArray::SharedPtr latest_live; int64_t latest_live_recv = 0;
  std::deque<std::pair<int64_t, OdomMetadata>> odom_hist;
  std::string last_status; bool reset_floor_valid = false; int64_t reset_key = 0; int64_t latest_odom_key = 0;
  int64_t t0 = -1; size_t frames = 0, valid_frames = 0;

  while (reader.has_next()) {
    auto msg = reader.read_next();
    if (t0 < 0) t0 = msg->time_stamp;
    rclcpp::SerializedMessage ser(*msg->serialized_data);
    if (msg->topic_name == "/localization/cone_map") {
      auto m = std::make_shared<ConeArray>();
      cones_ser.deserialize_message(&ser, m.get());
      const int64_t k = keyOf(m->header.stamp);
      if (reset_floor_valid && k <= reset_key) continue;
      latched_map = m; latched_map_recv = msg->time_stamp; have_map = true;
    } else if (msg->topic_name == "/localization/status") {
      std_msgs::msg::String s; str_ser.deserialize_message(&ser, &s);
      const bool entering_mapping = s.data == "mapping" && !last_status.empty() && last_status != "mapping";
      if (entering_mapping) { latched_map.reset(); have_map = false; reset_key = latest_odom_key; reset_floor_valid = true; }
      if (s.data == "localization") reset_floor_valid = false;
      last_status = s.data;
      fprintf(out, "{\"ev\":\"status\",\"t\":%.3f,\"status\":\"%s\"}\n", (msg->time_stamp - t0) * 1e-9, s.data.c_str());
    } else if (msg->topic_name == "/perception/cones") {
      auto m = std::make_shared<ConeArray>();
      cones_ser.deserialize_message(&ser, m.get());
      latest_live = m; latest_live_recv = msg->time_stamp;
    } else if (msg->topic_name == "/localization/ego_odom") {
      auto m = std::make_shared<Odometry>();
      odom_ser.deserialize_message(&ser, m.get());
      latest_odom_key = keyOf(m->header.stamp);
      // receive time: emulate steady clock with bag receive time; age 0 for this odom
      OdomMetadata om;
      om.header = HeaderMetadata{m->header.frame_id, m->header.stamp.sec, m->header.stamp.nanosec, 0.0};
      om.child_frame_id = m->child_frame_id;
      om.position = {m->pose.pose.position.x, m->pose.pose.position.y};
      om.yaw = yawOf(m->pose.pose.orientation);
      odom_hist.emplace_back(latest_odom_key, om);
      while (odom_hist.size() > 64U) odom_hist.pop_front();
      const double t = (msg->time_stamp - t0) * 1e-9;
      ++frames;
      if (!have_map) {
        fprintf(out, "{\"ev\":\"frame\",\"t\":%.3f,\"valid\":0,\"reason\":\"waiting for slam cone map\"}\n", t);
        continue;
      }
      HeaderMetadata mh{latched_map->header.frame_id, latched_map->header.stamp.sec, latched_map->header.stamp.nanosec, (msg->time_stamp - latched_map_recv) * 1e-9};
      auto val = validateSlamMapInput(mh, om, 0.5);
      if (!val.valid) {
        fprintf(out, "{\"ev\":\"frame\",\"t\":%.3f,\"valid\":0,\"reason\":\"%s\"}\n", t, val.reason.c_str());
        continue;
      }
      ConeSet cs = slamConeSet(*latched_map, om);
      const ConeSet map_cs = cs;
      const size_t map_n[5] = {cs.blue.size(), cs.yellow.size(), cs.orange.size(), cs.big_orange.size(), cs.unknown.size()};
      size_t live_added = 0; double live_age = -1; double live_gap = -1;
      bool live_used = false;
      if (use_live && latest_live) {
        // nearest odom by stamp
        const int64_t lk = keyOf(latest_live->header.stamp);
        int64_t best_gap = INT64_MAX; OdomMetadata live_odom;
        for (const auto & e : odom_hist) { const int64_t g = std::llabs(e.first - lk); if (g < best_gap) { best_gap = g; live_odom = e.second; } }
        live_gap = best_gap * 1e-9;
        live_age = (msg->time_stamp - latest_live_recv) * 1e-9;
        if (best_gap <= 250000000LL && live_age <= live_max_age) {
          const size_t before = cs.blue.size() + cs.yellow.size() + cs.orange.size() + cs.big_orange.size() + cs.unknown.size();
          extendConeSetWithLiveCones(cs, *latest_live, live_odom, om, live_cfg);
          live_added = cs.blue.size() + cs.yellow.size() + cs.orange.size() + cs.big_orange.size() + cs.unknown.size() - before;
          live_used = true;
        }
      }
      PlannerConfig c = cfg;
      const double speed = std::hypot(m->twist.twist.linear.x, m->twist.twist.linear.y);
      if (speed < standstill_recovery_speed) c.max_heading_change_rad = std::max(c.max_heading_change_rad, c.max_u_turn_heading_change_rad);
      if (explain_t >= 0.0 && !explained && t >= explain_t) {
        explained = true;
        auto explain_set = [&](const char * label, const ConeSet & set) {
          auto blue = internal::cropToRoi(set.blue, c);
          auto yellow = internal::cropToRoi(set.yellow, c);
          const size_t nb0 = blue.size(), ny0 = yellow.size();
          std::vector<Point2> colorless = internal::cropToRoi(set.unknown, c);
          std::vector<Point2> markers;
          for (const auto & q : internal::cropToRoi(set.orange, c)) markers.push_back(q);
          for (const auto & q : internal::cropToRoi(set.big_orange, c)) markers.push_back(q);
          internal::classifyColorlessCones(blue, yellow, colorless, markers, c);
          fprintf(stderr, "=== %s t=%.2f v=%.2f maxhead=%.2f: blue %zu(raw %zu) yellow %zu(raw %zu) colorless %zu\n", label, t, speed, c.max_heading_change_rad, blue.size(), nb0, yellow.size(), ny0, colorless.size());
          fprintf(stderr, "  blue:"); for (auto & q : blue) fprintf(stderr, " (%.1f,%.1f)", q.x, q.y); fprintf(stderr, "\n");
          fprintf(stderr, "  yellow:"); for (auto & q : yellow) fprintf(stderr, " (%.1f,%.1f)", q.x, q.y); fprintf(stderr, "\n");
          fprintf(stderr, "  colorless:"); for (auto & q : colorless) fprintf(stderr, " (%.1f,%.1f)", q.x, q.y); fprintf(stderr, "\n");
          const auto chain = internal::boundaryMidpoints(blue, yellow, c);
          fprintf(stderr, "  midpoint chain (%zu):", chain.size()); for (auto & q : chain) fprintf(stderr, " (%.1f,%.1f)", q.x, q.y); fprintf(stderr, "\n");
        };
        explain_set("MAP-ONLY", map_cs);
        if (live_added > 0) explain_set("MAP+LIVE", cs);
      }
      std::string note;
      BuildResult r = reconcile ? planWithLiveExtension(map_cs, live_added > 0 ? &cs : nullptr, c, &note) : buildLocalPath(cs, c);
      for (char & ch : note) if (ch == '"') ch = '\'';
      const char * kind = r.kind == PathKind::kTwoSided ? "two_sided" : r.kind == PathKind::kBlueOnly ? "blue_only" : r.kind == PathKind::kYellowOnly ? "yellow_only" : "none";
      fprintf(out, "{\"ev\":\"frame\",\"t\":%.3f,\"stamp\":%.3f,\"x\":%.3f,\"y\":%.3f,\"yaw\":%.4f,\"v\":%.2f,\"valid\":%d,\"kind\":\"%s\",\"reason\":\"%s\",\"map_n\":[%zu,%zu,%zu,%zu,%zu],\"live_used\":%d,\"live_added\":%zu,\"live_age\":%.3f,\"live_gap\":%.3f,\"status\":\"%s\",\"note\":\"%s\"",
        t, latest_odom_key * 1e-9, om.position.x, om.position.y, om.yaw, speed, r.valid ? 1 : 0, kind, r.reason.c_str(),
        map_n[0], map_n[1], map_n[2], map_n[3], map_n[4], live_used ? 1 : 0, live_added, live_age, live_gap, last_status.c_str(), note.c_str());
      if (r.valid) {
        ++valid_frames;
        fprintf(out, ",\"wp\":[");
        for (size_t i = 0; i < r.waypoints.size(); ++i) {
          const auto & w = r.waypoints[i];
          fprintf(out, "%s[%.3f,%.3f,%.4f,%.4f,%.2f]", i ? "," : "", w.x, w.y, w.psi, w.kappa, w.speed);
        }
        fprintf(out, "]");
      }
      if (dump_cones) {
        const char * names[5] = {"blue", "yellow", "orange", "big_orange", "unknown"};
        const std::vector<Point2> * buckets[5] = {&cs.blue, &cs.yellow, &cs.orange, &cs.big_orange, &cs.unknown};
        for (int b = 0; b < 5; ++b) {
          fprintf(out, ",\"%s\":[", names[b]);
          for (size_t i = 0; i < buckets[b]->size(); ++i) fprintf(out, "%s[%.2f,%.2f]", i ? "," : "", (*buckets[b])[i].x, (*buckets[b])[i].y);
          fprintf(out, "]");
        }
      }
      fprintf(out, "}\n");
    }
  }
  fclose(out);
  fprintf(stderr, "frames=%zu valid=%zu\n", frames, valid_frames);
  return 0;
}
