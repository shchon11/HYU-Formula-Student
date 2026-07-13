#include "local_planner/ros_inputs.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace local_planner
{
namespace
{

std::int64_t stampKey(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<std::int64_t>(stamp.sec) * 1000000000LL +
         static_cast<std::int64_t>(stamp.nanosec);
}

double steadyAge(SteadyTime receive_time)
{
  if (receive_time == SteadyTime{}) {
    return std::numeric_limits<double>::infinity();
  }
  return std::chrono::duration<double>(std::chrono::steady_clock::now() - receive_time).count();
}

double yawFromQuaternion(const geometry_msgs::msg::Quaternion & quaternion)
{
  const double values[] = {quaternion.x, quaternion.y, quaternion.z, quaternion.w};
  for (const double value : values) {
    if (!std::isfinite(value)) {
      return std::numeric_limits<double>::quiet_NaN();
    }
  }
  const double norm = std::sqrt(
    quaternion.x * quaternion.x + quaternion.y * quaternion.y +
    quaternion.z * quaternion.z + quaternion.w * quaternion.w);
  if (norm <= 1.0e-12) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  const double x = quaternion.x / norm;
  const double y = quaternion.y / norm;
  const double z = quaternion.z / norm;
  const double w = quaternion.w / norm;
  return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

Point2 mapToEgo(const Point2 & point, const OdomMetadata & odom)
{
  const double delta_x = point.x - odom.position.x;
  const double delta_y = point.y - odom.position.y;
  const double cosine = std::cos(odom.yaw);
  const double sine = std::sin(odom.yaw);
  return {
    cosine * delta_x + sine * delta_y,
    -sine * delta_x + cosine * delta_y,
  };
}

template<typename ConeContainer, typename Transform>
std::vector<Point2> conePoints(
  const ConeContainer & cones, Transform transform, bool & input_overflow)
{
  std::vector<Point2> points;
  const std::size_t count = std::min(cones.size(), kMaxBoundaryCones);
  points.reserve(count);
  for (std::size_t index = 0; index < count; ++index) {
    const auto & cone = cones[index];
    points.push_back(transform(Point2{cone.point.x, cone.point.y}));
  }
  input_overflow = input_overflow || cones.size() > kMaxBoundaryCones;
  return points;
}

}

HeaderMetadata headerMetadata(const std_msgs::msg::Header & header, SteadyTime receive_time)
{
  return HeaderMetadata{
    header.frame_id,
    header.stamp.sec,
    header.stamp.nanosec,
    steadyAge(receive_time),
  };
}

OdomMetadata odomMetadata(const Odometry & odom, SteadyTime receive_time)
{
  return OdomMetadata{
    headerMetadata(odom.header, receive_time),
    odom.child_frame_id,
    {odom.pose.pose.position.x, odom.pose.pose.position.y},
    yawFromQuaternion(odom.pose.pose.orientation),
  };
}

ConeSet liveConeSet(const ConeArray & message)
{
  const auto identity = [](const Point2 & point) {return point;};
  bool input_overflow = false;
  return ConeSet{
    conePoints(message.blue_cones, identity, input_overflow),
    conePoints(message.yellow_cones, identity, input_overflow),
    {},
    {},
    {},
    input_overflow,
  };
}

ConeSet slamConeSet(const ConeArray & message, const OdomMetadata & odom)
{
  const auto transform = [&odom](const Point2 & point) {return mapToEgo(point, odom);};
  bool input_overflow = false;
  return ConeSet{
    conePoints(message.blue_cones, transform, input_overflow),
    conePoints(message.yellow_cones, transform, input_overflow),
    {},
    {},
    {},
    input_overflow,
  };
}

LocalPlannerInputs::LocalPlannerInputs(
  rclcpp::Node & node, const SourceMode source_mode, LocalPlannerInputTopics topics,
  const double max_stamp_skew_sec, InvalidateCallback invalidate,
  LivePairCallback live_pair_callback, SlamMapCallback slam_map_callback)
: node_(node),
  topics_(std::move(topics)),
  max_stamp_skew_sec_(max_stamp_skew_sec),
  invalidate_(std::move(invalidate)),
  live_pair_callback_(std::move(live_pair_callback)),
  slam_map_callback_(std::move(slam_map_callback))
{
  if (source_mode == SourceMode::kLiveCones) {
    configureLiveSource();
  } else {
    configureSlamMapSource();
  }
}

void LocalPlannerInputs::configureLiveSource()
{
  live_cones_subscriber_.subscribe(
    &node_, topics_.cones, rclcpp::SensorDataQoS().get_rmw_qos_profile());
  live_odom_subscriber_.subscribe(
    &node_, topics_.odom,
    rclcpp::QoS(rclcpp::KeepLast(20)).reliable().durability_volatile().get_rmw_qos_profile());
  live_cones_subscriber_.registerCallback(&LocalPlannerInputs::recordLiveCones, this);
  live_odom_subscriber_.registerCallback(&LocalPlannerInputs::recordLiveOdom, this);
  synchronizer_ = std::make_unique<Synchronizer>(
    SyncPolicy(20), live_cones_subscriber_, live_odom_subscriber_);
  synchronizer_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(max_stamp_skew_sec_));
  synchronizer_->registerCallback(&LocalPlannerInputs::processLivePair, this);
}

void LocalPlannerInputs::configureSlamMapSource()
{
  const auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
  const auto odom_qos = rclcpp::QoS(rclcpp::KeepLast(20)).reliable().durability_volatile();
  slam_map_subscription_ = node_.create_subscription<ConeArray>(
    topics_.slam_map, map_qos,
    std::bind(&LocalPlannerInputs::receiveSlamMap, this, std::placeholders::_1));
  if (!topics_.slam_status.empty()) {
    slam_status_subscription_ = node_.create_subscription<std_msgs::msg::String>(
      topics_.slam_status, map_qos,
      std::bind(&LocalPlannerInputs::receiveSlamStatus, this, std::placeholders::_1));
  }
  slam_odom_subscription_ = node_.create_subscription<Odometry>(
    topics_.odom, odom_qos,
    std::bind(&LocalPlannerInputs::processSlamOdom, this, std::placeholders::_1));
}

void LocalPlannerInputs::recordLiveCones(const ConeArray::ConstSharedPtr & message)
{
  const auto now = std::chrono::steady_clock::now();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    cone_receipts_[stampKey(message->header.stamp)] = now;
    while (cone_receipts_.size() > 64U) {
      cone_receipts_.erase(cone_receipts_.begin());
    }
  }
}

void LocalPlannerInputs::recordLiveOdom(const Odometry::ConstSharedPtr & message)
{
  const auto now = std::chrono::steady_clock::now();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    odom_receipts_[stampKey(message->header.stamp)] = now;
    while (odom_receipts_.size() > 64U) {
      odom_receipts_.erase(odom_receipts_.begin());
    }
  }
}

SteadyTime LocalPlannerInputs::receiptFor(
  const std::map<std::int64_t, SteadyTime> & receipts,
  const builtin_interfaces::msg::Time & stamp) const
{
  const auto found = receipts.find(stampKey(stamp));
  return found == receipts.end() ? SteadyTime{} : found->second;
}

void LocalPlannerInputs::processLivePair(
  const ConeArray::ConstSharedPtr & cones, const Odometry::ConstSharedPtr & odom)
{
  LiveInputPair input{cones, odom, SteadyTime{}, SteadyTime{}};
  {
    std::lock_guard<std::mutex> lock(mutex_);
    input.cones_receive_time = receiptFor(cone_receipts_, cones->header.stamp);
    input.odom_receive_time = receiptFor(odom_receipts_, odom->header.stamp);
  }
  live_pair_callback_(input);
}

void LocalPlannerInputs::receiveSlamMap(const ConeArray::SharedPtr message)
{
  const auto map_stamp_key = stampKey(message->header.stamp);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (mapping_reset_floor_valid_ && map_stamp_key <= mapping_reset_stamp_key_) {
      return;
    }
    latched_map_ = message;
    latched_map_receive_time_ = std::chrono::steady_clock::now();
  }
}

void LocalPlannerInputs::receiveSlamStatus(
  const std_msgs::msg::String::ConstSharedPtr message)
{
  bool invalidate = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const bool entering_mapping = message->data == "mapping" &&
      !last_slam_status_.empty() && last_slam_status_ != "mapping";
    if (entering_mapping) {
      latched_map_.reset();
      latched_map_receive_time_ = SteadyTime{};
      mapping_reset_stamp_key_ = latest_odom_stamp_key_;
      mapping_reset_floor_valid_ = true;
      invalidate = true;
    }
    if (message->data == "localization") {
      mapping_reset_floor_valid_ = false;
    }
    last_slam_status_ = message->data;
  }
  if (invalidate) {
    invalidate_("slam restarted mapping; latched cone map dropped");
  }
}

void LocalPlannerInputs::processSlamOdom(const Odometry::SharedPtr message)
{
  SlamMapInput input{nullptr, message, SteadyTime{}, std::chrono::steady_clock::now()};
  {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_odom_stamp_key_ = stampKey(message->header.stamp);
    input.map = latched_map_;
    input.map_receive_time = latched_map_receive_time_;
  }
  if (!input.map) {
    invalidate_("waiting for slam cone map");
    return;
  }
  slam_map_callback_(input);
}

}
