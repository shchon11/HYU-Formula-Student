// Copyright 2026 shchon11
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.
//
// In-plugin camera realism: rolling shutter + exposure motion blur applied
// to the frame buffer INSIDE the stock gazebo_ros_camera pipeline, before
// publication. This replaces the external camera_realism topic shim, whose
// subscribe/republish hop halved the image rate and pushed YOLO bboxes past
// the perception pairing window (0.38 s vs the 0.25 s limit).
//
// The class subclasses gazebo_plugins::GazeboRosCamera and intercepts
// OnNewMultiFrame (the ZED is a multicamera sensor): the render buffer is
// warped and averaged in place, then handed to the stock implementation,
// so topics/QoS/camera_info behave exactly like the vanilla plugin. The
// body twist comes straight from the parent model's physics state — no
// topic, no latency, correct even before any ROS graph exists.
//
// Effect model (mirrors the retired scripts/camera_realism.py):
//  * pixel flow F = rotation (fx*wz, fy*wy) + translation about the FOE
//    with a ground-plane depth prior Z(row) = h*fy / (row - cy);
//  * rolling shutter: row time tau(row) = (row/H - 1/2) * readout_time;
//  * exposure: K taps over [-exposure/2, +exposure/2];
//  * one remap per tap with displacement F * (tau(row) + s_k), averaged.
// Known limits: the ego bodywork blurs like the world (a real hood stays
// sharp), and verticals above the ground line loom with the ground's depth.

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/PhysicsIface.hh>
#include <gazebo/sensors/Sensor.hh>
#include <gazebo_plugins/gazebo_ros_camera.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

namespace eufs_plugins
{

class GazeboCameraRealism : public gazebo_plugins::GazeboRosCamera
{
public:
  void Load(gazebo::sensors::SensorPtr _sensor, sdf::ElementPtr _sdf) override
  {
    gazebo_plugins::GazeboRosCamera::Load(_sensor, _sdf);

    motion_blur_ = _sdf->Get<bool>("motionBlur", true).first;
    rolling_shutter_ = _sdf->Get<bool>("rollingShutter", true).first;
    exposure_time_ = _sdf->Get<double>("exposureTime", 0.004).first;
    readout_time_ = _sdf->Get<double>("readoutTime", 0.020).first;
    blur_taps_ = std::max(1, _sdf->Get<int>("blurTaps", 3).first);
    cam_height_ = _sdf->Get<double>("cameraHeight", 0.85).first;
    max_shift_px_ = _sdf->Get<double>("maxShiftPx", 60.0).first;
    hfov_ = _sdf->Get<double>("horizontalFov", 1.91986).first;

    world_ = gazebo::physics::get_world(_sensor->WorldName());
    // "model::link" -> the model that carries this sensor.
    const std::string parent = _sensor->ParentName();
    model_name_ = parent.substr(0, parent.find("::"));
  }

protected:
  void OnNewMultiFrame(
    const unsigned char * _image,
    unsigned int _width, unsigned int _height,
    unsigned int _depth, const std::string & _format,
    const int _camera_num) override
  {
    if ((!motion_blur_ && !rolling_shutter_) || _depth != 3) {
      gazebo_plugins::GazeboRosCamera::OnNewMultiFrame(
        _image, _width, _height, _depth, _format, _camera_num);
      return;
    }
    ensureGeometry(static_cast<int>(_width), static_cast<int>(_height));
    // Both eyes render in the same sensor update and share one twist: the
    // maps are built once on the left frame and reused by the right.
    if (_camera_num == 0) {
      buildMaps();
    }

    const cv::Mat src(
      static_cast<int>(_height), static_cast<int>(_width), CV_8UC3,
      const_cast<unsigned char *>(_image));
    cv::Mat out;
    if (maps_.size() == 1U) {
      cv::remap(
        src, out, maps_[0].first, maps_[0].second,
        cv::INTER_LINEAR, cv::BORDER_REPLICATE);
    } else {
      for (std::size_t k = 0; k < maps_.size(); ++k) {
        cv::remap(
          src, tap_, maps_[k].first, maps_[k].second,
          cv::INTER_LINEAR, cv::BORDER_REPLICATE);
        if (k == 0U) {
          tap_.convertTo(accum_, CV_16UC3);
        } else {
          cv::add(accum_, tap_, accum_, cv::noArray(), CV_16UC3);
        }
      }
      accum_.convertTo(out, CV_8UC3, 1.0 / static_cast<double>(maps_.size()));
    }
    gazebo_plugins::GazeboRosCamera::OnNewMultiFrame(
      out.data, _width, _height, _depth, _format, _camera_num);
  }

private:
  void ensureGeometry(int width, int height)
  {
    if (width == width_ && height == height_) {
      return;
    }
    width_ = width;
    height_ = height;
    fx_ = width_ / (2.0 * std::tan(0.5 * hfov_));
    fy_ = fx_;
    cx_ = width_ / 2.0;
    cy_ = height_ / 2.0;
    inv_depth_row_.resize(height_);
    tau_row_.resize(height_);
    for (int row = 0; row < height_; ++row) {
      const double below = row - cy_;
      inv_depth_row_[row] =
        below > 1.0 ? below / (cam_height_ * fy_) : 0.0;
      tau_row_[row] = (static_cast<double>(row) / height_) - 0.5;
    }
  }

  void buildMaps()
  {
    double vx = 0.0, vy = 0.0, wy = 0.0, wz = 0.0;
    if (!model_ && world_) {
      model_ = world_->ModelByName(model_name_);
    }
    if (model_) {
      const auto lin = model_->RelativeLinearVel();
      const auto ang = model_->RelativeAngularVel();
      vx = lin.X();
      vy = lin.Y();
      wy = ang.Y();
      wz = ang.Z();
    }

    std::vector<double> taus;
    if (motion_blur_ && blur_taps_ > 1) {
      for (int k = 0; k < blur_taps_; ++k) {
        taus.push_back(
          (static_cast<double>(k) / (blur_taps_ - 1) - 0.5) * exposure_time_);
      }
    } else {
      taus.push_back(0.0);
    }
    maps_.resize(taus.size());
    for (auto & maps : maps_) {
      maps.first.create(height_, width_, CV_32FC1);
      maps.second.create(height_, width_, CV_32FC1);
    }

    const float max_shift = static_cast<float>(max_shift_px_);
    for (int row = 0; row < height_; ++row) {
      const double inv_z = inv_depth_row_[row];
      const double rs =
        rolling_shutter_ ? tau_row_[row] * readout_time_ : 0.0;
      const double flow_v =
        fy_ * wy + (row - cy_) * vx * inv_z;
      const double flow_u_base = fx_ * wz + fx_ * vy * inv_z;
      const double flow_u_slope = vx * inv_z;  // times (col - cx)
      for (std::size_t k = 0; k < taus.size(); ++k) {
        const double t = rs + taus[k];
        float * map_u = maps_[k].first.ptr<float>(row);
        float * map_v = maps_[k].second.ptr<float>(row);
        const float dv = std::clamp(
          static_cast<float>(flow_v * t), -max_shift, max_shift);
        for (int col = 0; col < width_; ++col) {
          const float du = std::clamp(
            static_cast<float>(
              (flow_u_base + flow_u_slope * (col - cx_)) * t),
            -max_shift, max_shift);
          map_u[col] = col - du;
          map_v[col] = row - dv;
        }
      }
    }
  }

  bool motion_blur_{true};
  bool rolling_shutter_{true};
  double exposure_time_{0.004};
  double readout_time_{0.020};
  int blur_taps_{3};
  double cam_height_{0.85};
  double max_shift_px_{60.0};
  double hfov_{1.91986};

  gazebo::physics::WorldPtr world_;
  gazebo::physics::ModelPtr model_;
  std::string model_name_;

  int width_{0};
  int height_{0};
  double fx_{0.0}, fy_{0.0}, cx_{0.0}, cy_{0.0};
  std::vector<double> inv_depth_row_;
  std::vector<double> tau_row_;
  std::vector<std::pair<cv::Mat, cv::Mat>> maps_;
  cv::Mat tap_;
  cv::Mat accum_;
};

}  // namespace eufs_plugins

// GZ_REGISTER_SENSOR_PLUGIN cannot be used here: GazeboRosCamera carries
// THREE SensorPlugin bases (Camera/DepthCamera/MultiCamera plugins), so the
// macro's implicit upcast is ambiguous. Register manually through one base.
extern "C" GZ_PLUGIN_VISIBLE gazebo::SensorPlugin * RegisterPlugin();
gazebo::SensorPlugin * RegisterPlugin()
{
  return static_cast<gazebo::CameraPlugin *>(
    new eufs_plugins::GazeboCameraRealism());
}
