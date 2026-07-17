#pragma once

#include <chrono>
#include <cstdint>
#include <deque>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include <builtin_interfaces/msg/time.hpp>
#include <eufs_msgs/msg/cone_array_with_covariance.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/string.hpp>

#include "hyu_local_planner/input_policy.hpp"

namespace hyu_local_planner
{

using ConeArray = eufs_msgs::msg::ConeArrayWithCovariance;
using Odometry = nav_msgs::msg::Odometry;
using SteadyTime = std::chrono::steady_clock::time_point;

struct LocalPlannerInputTopics
{
  std::string cones;
  std::string slam_map;
  std::string odom;
  std::string slam_status;
};

struct LiveInputPair
{
  ConeArray::ConstSharedPtr cones;
  Odometry::ConstSharedPtr odom;
  SteadyTime cones_receive_time;
  SteadyTime odom_receive_time;
};

struct SlamMapInput
{
  ConeArray::SharedPtr map;
  Odometry::SharedPtr odom;
  SteadyTime map_receive_time;
  SteadyTime odom_receive_time;
  // Latest live perception frame (slam mode live-cone extension). Null when
  // the extension is disabled or nothing has arrived. live_odom is the pose
  // whose stamp is nearest the cone frame's stamp — the frame the cones were
  // measured in; valid only when live_odom_valid.
  ConeArray::ConstSharedPtr live_cones;
  SteadyTime live_cones_receive_time{};
  OdomMetadata live_odom{};
  bool live_odom_valid{false};
};

struct LiveExtensionConfig
{
  // Add live cones as unknown-color evidence (they then pass the conservative
  // classifyUnknownCones route) instead of trusting perception's colors.
  bool as_unknown{true};
  // A live cone within this radius of ANY map cone is dropped: the map wins,
  // and near-duplicate inputs must never reach boundary ordering.
  double merge_radius_m{1.0};
};

HeaderMetadata headerMetadata(
  const std_msgs::msg::Header & header, SteadyTime receive_time);
OdomMetadata odomMetadata(const Odometry & odom, SteadyTime receive_time);
ConeSet liveConeSet(const ConeArray & message);
ConeSet slamConeSet(const ConeArray & message, const OdomMetadata & odom);

// Extend a map-derived ego-frame ConeSet with live perception cones so the
// path can continue past the SLAM map frontier. Live cones are measured in
// the vehicle frame at `live_odom`; they are re-expressed in the CURRENT
// vehicle frame (`current_odom`) before merging. Never degrades the map-only
// result: map cones are untouched, deduplication drops anything the map
// already covers, and capacity pressure drops live cones instead of flagging
// input_overflow.
void extendConeSetWithLiveCones(
  ConeSet & cone_set, const ConeArray & live_cones, const OdomMetadata & live_odom,
  const OdomMetadata & current_odom, const LiveExtensionConfig & config);

class LocalPlannerInputs
{
public:
  using InvalidateCallback = std::function<void(const std::string &)>;
  using LivePairCallback = std::function<void(const LiveInputPair &)>;
  using SlamMapCallback = std::function<void(const SlamMapInput &)>;

  LocalPlannerInputs(
    rclcpp::Node & node, SourceMode source_mode, LocalPlannerInputTopics topics,
    double max_stamp_skew_sec, InvalidateCallback invalidate,
    LivePairCallback live_pair_callback, SlamMapCallback slam_map_callback,
    bool slam_live_extension = false);

private:
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<ConeArray, Odometry>;
  using Synchronizer = message_filters::Synchronizer<SyncPolicy>;

  void configureLiveSource();
  void configureSlamMapSource();
  void recordLiveCones(const ConeArray::ConstSharedPtr & message);
  void recordLiveOdom(const Odometry::ConstSharedPtr & message);
  void processLivePair(
    const ConeArray::ConstSharedPtr & cones, const Odometry::ConstSharedPtr & odom);
  void receiveSlamMap(const ConeArray::SharedPtr message);
  void receiveSlamStatus(const std_msgs::msg::String::ConstSharedPtr message);
  void receiveSlamLiveCones(const ConeArray::ConstSharedPtr & message);
  void processSlamOdom(const Odometry::SharedPtr message);
  SteadyTime receiptFor(
    const std::map<std::int64_t, SteadyTime> & receipts,
    const builtin_interfaces::msg::Time & stamp) const;

  rclcpp::Node & node_;
  LocalPlannerInputTopics topics_;
  double max_stamp_skew_sec_;
  InvalidateCallback invalidate_;
  LivePairCallback live_pair_callback_;
  SlamMapCallback slam_map_callback_;

  message_filters::Subscriber<ConeArray> live_cones_subscriber_;
  message_filters::Subscriber<Odometry> live_odom_subscriber_;
  std::unique_ptr<Synchronizer> synchronizer_;
  rclcpp::Subscription<ConeArray>::SharedPtr slam_map_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr slam_status_subscription_;
  rclcpp::Subscription<Odometry>::SharedPtr slam_odom_subscription_;

  std::mutex mutex_;
  std::map<std::int64_t, SteadyTime> cone_receipts_;
  std::map<std::int64_t, SteadyTime> odom_receipts_;
  ConeArray::SharedPtr latched_map_;
  SteadyTime latched_map_receive_time_{};
  // Live-cone extension state (slam mode only, opt-in): latest perception
  // frame plus a short odom-pose history so the cones can be re-expressed
  // from the pose they were measured at.
  bool slam_live_extension_{false};
  rclcpp::Subscription<ConeArray>::SharedPtr slam_live_cones_subscription_;
  ConeArray::ConstSharedPtr latest_live_cones_;
  SteadyTime latest_live_cones_receive_time_{};
  std::deque<std::pair<std::int64_t, OdomMetadata>> odom_pose_history_;
  std::string last_slam_status_;
  bool mapping_reset_floor_valid_{false};
  std::int64_t mapping_reset_stamp_key_{0};
  std::int64_t latest_odom_stamp_key_{0};
};

}
