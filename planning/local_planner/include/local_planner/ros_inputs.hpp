#pragma once

#include <chrono>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <string>
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

#include "local_planner/input_policy.hpp"

namespace local_planner
{

using ConeArray = eufs_msgs::msg::ConeArrayWithCovariance;
using Odometry = nav_msgs::msg::Odometry;
using SteadyTime = std::chrono::steady_clock::time_point;

struct LocalPlannerInputTopics
{
  std::string cones;
  std::string slam_map;
  std::string odom;
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
};

HeaderMetadata headerMetadata(
  const std_msgs::msg::Header & header, SteadyTime receive_time);
OdomMetadata odomMetadata(const Odometry & odom, SteadyTime receive_time);
ConeSet liveConeSet(const ConeArray & message);
ConeSet slamConeSet(const ConeArray & message, const OdomMetadata & odom);

class LocalPlannerInputs
{
public:
  using InvalidateCallback = std::function<void()>;
  using LivePairCallback = std::function<void(const LiveInputPair &)>;
  using SlamMapCallback = std::function<void(const SlamMapInput &)>;

  LocalPlannerInputs(
    rclcpp::Node & node, SourceMode source_mode, LocalPlannerInputTopics topics,
    double max_stamp_skew_sec, InvalidateCallback invalidate,
    LivePairCallback live_pair_callback, SlamMapCallback slam_map_callback);

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
  rclcpp::Subscription<Odometry>::SharedPtr slam_odom_subscription_;

  std::mutex mutex_;
  std::map<std::int64_t, SteadyTime> cone_receipts_;
  std::map<std::int64_t, SteadyTime> odom_receipts_;
  ConeArray::SharedPtr latched_map_;
  SteadyTime latched_map_receive_time_{};
};

}
