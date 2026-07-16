#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>

#include "eufs_msgs/msg/car_state.hpp"
#include "eufs_msgs/msg/wheel_speeds_stamped.hpp"
#include "hyu_formular_control_msgs/msg/tum_vehicle_state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.h"
#include "tum_vehicle_state_bridge/heading_conversion.hpp"

namespace
{
constexpr double kQuaternionNormEpsilon = 1.0e-12;

bool IsFinite(double value)
{
  return std::isfinite(value);
}

bool IsFiniteQuaternion(const geometry_msgs::msg::Quaternion & quaternion)
{
  return IsFinite(quaternion.x) && IsFinite(quaternion.y) &&
         IsFinite(quaternion.z) && IsFinite(quaternion.w);
}
}  // namespace

class TumVehicleStateBridge : public rclcpp::Node
{
public:
  using CarState = eufs_msgs::msg::CarState;
  using Odometry = nav_msgs::msg::Odometry;
  using TumVehicleState = hyu_formular_control_msgs::msg::TumVehicleState;
  using WheelSpeedsStamped = eufs_msgs::msg::WheelSpeedsStamped;

  TumVehicleStateBridge()
  : Node("tum_vehicle_state_bridge")
  {
    DeclareAndReadParameters();
    ValidateConfig();

    const auto input_qos = rclcpp::SensorDataQoS();
    localization_sub_ = create_subscription<Odometry>(
      localization_topic_, input_qos,
      std::bind(&TumVehicleStateBridge::LocalizationCallback, this, std::placeholders::_1));
    car_state_sub_ = create_subscription<CarState>(
      car_state_topic_, input_qos,
      std::bind(&TumVehicleStateBridge::CarStateCallback, this, std::placeholders::_1));
    wheel_speeds_sub_ = create_subscription<WheelSpeedsStamped>(
      wheel_speeds_topic_, input_qos,
      std::bind(&TumVehicleStateBridge::WheelSpeedsCallback, this, std::placeholders::_1));
    vehicle_state_pub_ = create_publisher<TumVehicleState>(
      output_topic_, rclcpp::QoS(10).reliable());

    const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
    // Node-clock timer so the publish rate follows sim time under use_sim_time.
    timer_ = rclcpp::create_timer(
      this, get_clock(),
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&TumVehicleStateBridge::PublishVehicleState, this));

    RCLCPP_INFO(get_logger(), "TUM vehicle-state bridge started");
    RCLCPP_INFO(get_logger(), "  localization: %s", localization_topic_.c_str());
    RCLCPP_INFO(get_logger(), "  car state:    %s", car_state_topic_.c_str());
    RCLCPP_INFO(get_logger(), "  wheel speeds: %s", wheel_speeds_topic_.c_str());
    RCLCPP_INFO(get_logger(), "  output:       %s", output_topic_.c_str());
  }

private:
  void DeclareAndReadParameters()
  {
    localization_topic_ = declare_parameter<std::string>(
      "tum_vehicle_state_bridge/localization_topic", "/localization/ego_odom");
    car_state_topic_ = declare_parameter<std::string>(
      "tum_vehicle_state_bridge/car_state_topic", "/wheel_odometry/car_state");
    wheel_speeds_topic_ = declare_parameter<std::string>(
      "tum_vehicle_state_bridge/wheel_speeds_topic", "/ros_can/wheel_speeds");
    output_topic_ = declare_parameter<std::string>(
      "tum_vehicle_state_bridge/output_topic", "/tmpc/vehicle_state");

    publish_rate_hz_ = declare_parameter<double>(
      "tum_vehicle_state_bridge/publish_rate_hz", 100.0);
    localization_timeout_sec_ = declare_parameter<double>(
      "tum_vehicle_state_bridge/localization_timeout_sec", 0.2);
    car_state_timeout_sec_ = declare_parameter<double>(
      "tum_vehicle_state_bridge/car_state_timeout_sec", 0.2);
    wheel_speeds_timeout_sec_ = declare_parameter<double>(
      "tum_vehicle_state_bridge/wheel_speeds_timeout_sec", 0.2);

    const auto se_status = declare_parameter<int64_t>(
      "tum_vehicle_state_bridge/se_status", 2);
    const auto se_state = declare_parameter<int64_t>(
      "tum_vehicle_state_bridge/se_state", 1);
    se_status_ = static_cast<uint16_t>(se_status);
    se_state_ = static_cast<uint16_t>(se_state);
    valid_imu_default_ = declare_parameter<bool>(
      "tum_vehicle_state_bridge/valid_imu_default", true);
    min_speed_for_beta_mps_ = declare_parameter<double>(
      "tum_vehicle_state_bridge/min_speed_for_beta_mps", 0.1);
    ax_vel_filter_tau_sec_ = declare_parameter<double>(
      "tum_vehicle_state_bridge/ax_vel_filter_tau_sec", 0.1);
  }

  void ValidateConfig() const
  {
    if (localization_topic_.empty() || car_state_topic_.empty() ||
      wheel_speeds_topic_.empty() || output_topic_.empty())
    {
      throw std::invalid_argument("bridge topic names must not be empty");
    }
    if (!IsFinite(publish_rate_hz_) || publish_rate_hz_ <= 0.0) {
      throw std::invalid_argument("publish_rate_hz must be finite and greater than zero");
    }
    if (!IsFinite(localization_timeout_sec_) || localization_timeout_sec_ <= 0.0 ||
      !IsFinite(car_state_timeout_sec_) || car_state_timeout_sec_ <= 0.0 ||
      !IsFinite(wheel_speeds_timeout_sec_) || wheel_speeds_timeout_sec_ <= 0.0)
    {
      throw std::invalid_argument("input timeouts must be finite and greater than zero");
    }
    if (se_status_ > 2U) {
      throw std::invalid_argument("se_status must be in the range [0, 2]");
    }
    if (se_state_ > 4U) {
      throw std::invalid_argument("se_state must be in the range [0, 4]");
    }
    if (!IsFinite(min_speed_for_beta_mps_) || min_speed_for_beta_mps_ < 0.0) {
      throw std::invalid_argument("min_speed_for_beta_mps must be finite and non-negative");
    }
    if (!IsFinite(ax_vel_filter_tau_sec_) || ax_vel_filter_tau_sec_ < 0.0) {
      throw std::invalid_argument("ax_vel_filter_tau_sec must be finite and non-negative");
    }
  }

  void LocalizationCallback(const Odometry::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_localization_ = *msg;
    localization_received_time_ = now();
    ++localization_sequence_;
    has_localization_ = true;
  }

  void CarStateCallback(const CarState::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_car_state_ = *msg;
    car_state_received_time_ = now();
    has_car_state_ = true;
  }

  void WheelSpeedsCallback(const WheelSpeedsStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_wheel_speeds_ = *msg;
    wheel_speeds_received_time_ = now();
    has_wheel_speeds_ = true;
  }

  bool IsFresh(const rclcpp::Time & current_time, const rclcpp::Time & received_time,
    double timeout_sec) const
  {
    const double age_sec = (current_time - received_time).seconds();
    return age_sec >= 0.0 && age_sec <= timeout_sec;
  }

  bool InputsAreFinite(
    const Odometry & localization, const CarState & car_state,
    const WheelSpeedsStamped & wheel_speeds) const
  {
    const auto & pose = localization.pose.pose;
    const auto & twist = localization.twist.twist;
    const auto & acceleration = car_state.linear_acceleration;
    return IsFinite(pose.position.x) && IsFinite(pose.position.y) &&
           IsFiniteQuaternion(pose.orientation) &&
           IsFinite(twist.linear.x) && IsFinite(twist.linear.y) &&
           IsFinite(twist.angular.z) && IsFinite(acceleration.x) &&
           IsFinite(acceleration.y) && IsFinite(wheel_speeds.speeds.steering);
  }

  bool QuaternionToYaw(const geometry_msgs::msg::Quaternion & msg, double & yaw) const
  {
    tf2::Quaternion quaternion(msg.x, msg.y, msg.z, msg.w);
    if (quaternion.length2() <= kQuaternionNormEpsilon) {
      return false;
    }
    quaternion.normalize();
    yaw = tf2::getYaw(quaternion);
    return IsFinite(yaw);
  }

  rclcpp::Time LocalizationSampleTime(
    const Odometry & localization, const rclcpp::Time & received_time) const
  {
    const rclcpp::Time header_time(localization.header.stamp, get_clock()->get_clock_type());
    if (header_time.nanoseconds() > 0) {
      return header_time;
    }
    return received_time;
  }

  double UpdateAxVel(
    double speed_mps, const rclcpp::Time & sample_time, uint64_t localization_sequence)
  {
    if (localization_sequence == processed_localization_sequence_) {
      return filtered_ax_vel_mps2_;
    }
    processed_localization_sequence_ = localization_sequence;

    if (!ax_vel_initialized_) {
      previous_speed_mps_ = speed_mps;
      previous_speed_time_ = sample_time;
      filtered_ax_vel_mps2_ = 0.0;
      ax_vel_initialized_ = true;
      return filtered_ax_vel_mps2_;
    }

    const double dt_sec = (sample_time - previous_speed_time_).seconds();
    if (!IsFinite(dt_sec) || dt_sec <= 0.0 || dt_sec > localization_timeout_sec_) {
      previous_speed_mps_ = speed_mps;
      previous_speed_time_ = sample_time;
      filtered_ax_vel_mps2_ = 0.0;
      return filtered_ax_vel_mps2_;
    }

    const double raw_ax_vel_mps2 = (speed_mps - previous_speed_mps_) / dt_sec;
    const double alpha = ax_vel_filter_tau_sec_ <= 0.0 ?
      1.0 : dt_sec / (ax_vel_filter_tau_sec_ + dt_sec);
    filtered_ax_vel_mps2_ += alpha * (raw_ax_vel_mps2 - filtered_ax_vel_mps2_);
    previous_speed_mps_ = speed_mps;
    previous_speed_time_ = sample_time;
    return filtered_ax_vel_mps2_;
  }

  void ResetAxVelFilter()
  {
    ax_vel_initialized_ = false;
    filtered_ax_vel_mps2_ = 0.0;
  }

  void PublishVehicleState()
  {
    Odometry localization;
    CarState car_state;
    WheelSpeedsStamped wheel_speeds;
    rclcpp::Time localization_received_time;
    rclcpp::Time car_state_received_time;
    rclcpp::Time wheel_speeds_received_time;
    uint64_t localization_sequence = 0U;
    bool has_localization = false;
    bool has_car_state = false;
    bool has_wheel_speeds = false;

    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      localization = latest_localization_;
      car_state = latest_car_state_;
      wheel_speeds = latest_wheel_speeds_;
      localization_received_time = localization_received_time_;
      car_state_received_time = car_state_received_time_;
      wheel_speeds_received_time = wheel_speeds_received_time_;
      localization_sequence = localization_sequence_;
      has_localization = has_localization_;
      has_car_state = has_car_state_;
      has_wheel_speeds = has_wheel_speeds_;
    }

    const auto current_time = now();
    const bool localization_fresh = has_localization &&
      IsFresh(current_time, localization_received_time, localization_timeout_sec_);
    const bool car_state_fresh = has_car_state &&
      IsFresh(current_time, car_state_received_time, car_state_timeout_sec_);
    const bool wheel_speeds_fresh = has_wheel_speeds &&
      IsFresh(current_time, wheel_speeds_received_time, wheel_speeds_timeout_sec_);

    if (!localization_fresh || !car_state_fresh || !wheel_speeds_fresh) {
      ResetAxVelFilter();
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Waiting for fresh bridge inputs: localization=%d car_state=%d wheel_speeds=%d",
        localization_fresh, car_state_fresh, wheel_speeds_fresh);
      return;
    }

    if (!InputsAreFinite(localization, car_state, wheel_speeds)) {
      ResetAxVelFilter();
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000, "Bridge input contains a non-finite value");
      return;
    }

    double yaw_rad = 0.0;
    if (!QuaternionToYaw(localization.pose.pose.orientation, yaw_rad)) {
      ResetAxVelFilter();
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000, "Localization quaternion is invalid");
      return;
    }

    const double vx_mps = localization.twist.twist.linear.x;
    const double vy_mps = localization.twist.twist.linear.y;
    const double speed_mps = std::hypot(vx_mps, vy_mps);
    const double beta_rad = speed_mps < min_speed_for_beta_mps_ ?
      0.0 : std::atan2(vy_mps, vx_mps);
    const auto sample_time = LocalizationSampleTime(localization, localization_received_time);
    const double ax_vel_mps2 = UpdateAxVel(speed_mps, sample_time, localization_sequence);

    TumVehicleState output;
    output.se_status = se_status_;
    output.se_state = se_state_;
    output.x_m = localization.pose.pose.position.x;
    output.y_m = localization.pose.pose.position.y;
    output.psi_rad = tum_vehicle_state_bridge::RosYawToFormulaHeading(yaw_rad);
    output.dpsi_radps = localization.twist.twist.angular.z;
    output.vx_mps = vx_mps;
    output.vy_mps = vy_mps;
    output.v_mps = speed_mps;
    output.beta_rad = beta_rad;
    output.ax_mps2 = car_state.linear_acceleration.x;
    output.ay_mps2 = car_state.linear_acceleration.y;
    output.ax_vel_mps2 = ax_vel_mps2;
    output.delta_wheel_rad = wheel_speeds.speeds.steering;
    output.valid_imu = valid_imu_default_ && car_state_fresh;
    vehicle_state_pub_->publish(output);
  }

  std::string localization_topic_;
  std::string car_state_topic_;
  std::string wheel_speeds_topic_;
  std::string output_topic_;
  double publish_rate_hz_{100.0};
  double localization_timeout_sec_{0.2};
  double car_state_timeout_sec_{0.2};
  double wheel_speeds_timeout_sec_{0.2};
  uint16_t se_status_{2U};
  uint16_t se_state_{1U};
  bool valid_imu_default_{true};
  double min_speed_for_beta_mps_{0.1};
  double ax_vel_filter_tau_sec_{0.1};

  std::mutex data_mutex_;
  Odometry latest_localization_;
  CarState latest_car_state_;
  WheelSpeedsStamped latest_wheel_speeds_;
  rclcpp::Time localization_received_time_;
  rclcpp::Time car_state_received_time_;
  rclcpp::Time wheel_speeds_received_time_;
  uint64_t localization_sequence_{0U};
  bool has_localization_{false};
  bool has_car_state_{false};
  bool has_wheel_speeds_{false};

  uint64_t processed_localization_sequence_{0U};
  bool ax_vel_initialized_{false};
  double previous_speed_mps_{0.0};
  rclcpp::Time previous_speed_time_;
  double filtered_ax_vel_mps2_{0.0};

  rclcpp::Subscription<Odometry>::SharedPtr localization_sub_;
  rclcpp::Subscription<CarState>::SharedPtr car_state_sub_;
  rclcpp::Subscription<WheelSpeedsStamped>::SharedPtr wheel_speeds_sub_;
  rclcpp::Publisher<TumVehicleState>::SharedPtr vehicle_state_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TumVehicleStateBridge>());
  rclcpp::shutdown();
  return 0;
}
