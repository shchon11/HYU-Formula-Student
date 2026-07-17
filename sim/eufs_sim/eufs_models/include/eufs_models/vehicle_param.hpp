/*
 * AMZ-Driverless
 * Copyright (c) 2018 Authors:
 *   - Juraj Kabzan <kabzanj@gmail.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef EUFS_MODELS_INCLUDE_EUFS_MODELS_VEHICLE_PARAM_HPP_
#define EUFS_MODELS_INCLUDE_EUFS_MODELS_VEHICLE_PARAM_HPP_

#include <string>
#include "yaml-cpp/yaml.h"

namespace eufs {
namespace models {

struct Param {
  explicit Param(const std::string &yaml_file) {
    YAML::Node config = YAML::LoadFile(yaml_file);

    inertia = config["inertia"].as<Param::Inertia>();
    kinematic = config["kinematics"].as<Param::Kinematic>();
    tire = config["tire"].as<Param::Tire>();
    aero = config["aero"].as<Param::Aero>();
    // Optional: a config written before the body had a suspension keeps its
    // defaults rather than failing to load.
    if (config["suspension"]) {
      suspension = config["suspension"].as<Param::Suspension>();
    }
    input_ranges = config["input_ranges"].as<Param::InputRanges>();
  }

  struct Inertia {
    double m;
    double g;
    double I_z;
  };

  struct Kinematic {
    double l;
    double b_F;
    double b_R;
    double w_front;
    double l_F;
    double l_R;
    double axle_width;
  };

  struct Tire {
    double tire_coefficient;
    double B;
    double C;
    double D;
    double E;
    double radius;
    /// Slip ratio at which the tyre's longitudinal force peaks. Below the
    /// peak, kappa(utilization) is near-linear, which is all getWheelSpeeds
    /// models. ~0.15 is generic race-tyre physics, NOT an FSK measurement;
    /// pin it from real launch logs when they exist. 0 disables slip.
    double peak_slip_ratio = 0.15;
  };

  struct Aero {
    double c_down;
    double c_drag;
  };

  /// How the body leans, and whether that lean is allowed to change what the
  /// tyres can do.
  ///
  /// The defaults stand in for a Formula Student car and are used whenever a
  /// config predates this section, so an old config still loads -- it just gets
  /// a generic car's suspension rather than this one's.
  struct Suspension {
    /// Centre of gravity height [m]. This is the whole reason a car leans: it
    /// is the moment arm the inertial force acts through.
    double h_cg = 0.30;
    /// Resistance to leaning [N*m/rad]. At 300 kg and h_cg 0.30, cornering at
    /// 1 g is a 883 N*m moment, so k_roll = 30000 leans the body 1.7 deg.
    double k_roll = 30000.0;
    double k_pitch = 40000.0;
    /// How the body gets there: a spring-mass, not a lag. Formula Student cars
    /// sit around 2-3 Hz, underdamped enough to overshoot a step.
    double natural_freq_hz = 2.5;
    double damping_ratio = 0.5;
    /// Feed longitudinal load transfer back into the tyre normal loads, so
    /// braking really does buy front grip. Off by default: it changes how the
    /// car handles, and the controllers are tuned against it being off.
    bool load_transfer_to_tires = false;
  };

  struct InputRanges {
    struct Range {
      double min;
      double max;
    };

    Range acc;
    Range vel;
    Range delta;
  };

  Inertia inertia;
  Kinematic kinematic;
  Tire tire;
  Aero aero;
  Suspension suspension;
  InputRanges input_ranges;
};

}  // namespace models
}  // namespace eufs

namespace YAML {
template <>
struct convert<eufs::models::Param::Inertia> {
  static bool decode(const Node &node, eufs::models::Param::Inertia &cType) {
    cType.m = node["m"].as<double>();
    cType.g = node["g"].as<double>();
    cType.I_z = node["I_z"].as<double>();
    return true;
  }
};

template <>
struct convert<eufs::models::Param::Kinematic> {
  static bool decode(const Node &node, eufs::models::Param::Kinematic &cType) {
    cType.l = node["l"].as<double>();
    cType.b_F = node["b_F"].as<double>();
    cType.b_R = node["b_R"].as<double>();
    cType.w_front = node["w_front"].as<double>();
    cType.l_F = cType.l * (1 - cType.w_front);
    cType.l_R = cType.l * cType.w_front;
    cType.axle_width = node["axle_width"].as<double>();
    return true;
  }
};

template <>
struct convert<eufs::models::Param::Tire> {
  static bool decode(const Node &node, eufs::models::Param::Tire &cType) {
    cType.tire_coefficient = node["tire_coefficient"].as<double>();
    cType.B = node["B"].as<double>() / cType.tire_coefficient;
    cType.C = node["C"].as<double>();
    cType.D = node["D"].as<double>() * cType.tire_coefficient;
    cType.E = node["E"].as<double>();
    cType.radius = node["radius"].as<double>();
    if (node["peak_slip_ratio"]) {
      cType.peak_slip_ratio = node["peak_slip_ratio"].as<double>();
    }
    return true;
  }
};

template <>
struct convert<eufs::models::Param::Aero> {
  static bool decode(const Node &node, eufs::models::Param::Aero &cType) {
    cType.c_down = node["C_Down"].as<double>();
    cType.c_drag = node["C_drag"].as<double>();
    return true;
  }
};

template <>
struct convert<eufs::models::Param::Suspension> {
  static bool decode(const Node &node, eufs::models::Param::Suspension &cType) {
    // Every field is optional and keeps its default, so a config can specify
    // only what it actually knows about its car.
    if (node["h_cg"]) {
      cType.h_cg = node["h_cg"].as<double>();
    }
    if (node["k_roll"]) {
      cType.k_roll = node["k_roll"].as<double>();
    }
    if (node["k_pitch"]) {
      cType.k_pitch = node["k_pitch"].as<double>();
    }
    if (node["natural_freq_hz"]) {
      cType.natural_freq_hz = node["natural_freq_hz"].as<double>();
    }
    if (node["damping_ratio"]) {
      cType.damping_ratio = node["damping_ratio"].as<double>();
    }
    if (node["load_transfer_to_tires"]) {
      cType.load_transfer_to_tires = node["load_transfer_to_tires"].as<bool>();
    }
    return true;
  }
};

template <>
struct convert<eufs::models::Param::InputRanges> {
  static bool decode(const Node &node, eufs::models::Param::InputRanges &cType) {
    cType.acc.min = node["acceleration"]["min"].as<double>();
    cType.acc.max = node["acceleration"]["max"].as<double>();
    cType.vel.min = node["velocity"]["min"].as<double>();
    cType.vel.max = node["velocity"]["max"].as<double>();
    cType.delta.min = node["steering"]["min"].as<double>();
    cType.delta.max = node["steering"]["max"].as<double>();
    return true;
  }
};

}  // namespace YAML

#endif  // EUFS_MODELS_INCLUDE_EUFS_MODELS_VEHICLE_PARAM_HPP_
