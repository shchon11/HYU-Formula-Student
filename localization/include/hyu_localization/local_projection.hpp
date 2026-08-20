// Copyright 2026 shchon11
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.
//
// Geodetic (lat, lon) -> local tangent-plane North/East metres about a datum.
// Default is the WGS84 meridional / prime-vertical radii at the datum (what
// the retired SBG bridge used, so /localization/ins_odom keeps its scale);
// `spherical` reproduces the offline reference my_ekf.py (R = 6378137 for
// both axes) for bit-for-bit comparisons.

#ifndef HYU_LOCALIZATION__LOCAL_PROJECTION_HPP_
#define HYU_LOCALIZATION__LOCAL_PROJECTION_HPP_

#include <cmath>

namespace hyu_localization
{

class LocalProjection
{
public:
  static constexpr double kWgs84A = 6378137.0;
  static constexpr double kWgs84E2 = 6.69437999014e-3;

  explicit LocalProjection(bool spherical = false)
  : spherical_(spherical) {}

  bool valid() const {return valid_;}
  double originLatDeg() const {return lat0_deg_;}
  double originLonDeg() const {return lon0_deg_;}

  void setOrigin(double lat_deg, double lon_deg)
  {
    lat0_deg_ = lat_deg;
    lon0_deg_ = lon_deg;
    lat0_ = lat_deg * M_PI / 180.0;
    lon0_ = lon_deg * M_PI / 180.0;
    if (spherical_) {
      kN_ = kWgs84A;
      kE_ = kWgs84A * std::cos(lat0_);
    } else {
      const double s = std::sin(lat0_);
      const double denom = 1.0 - kWgs84E2 * s * s;
      kN_ = kWgs84A * (1.0 - kWgs84E2) / std::pow(denom, 1.5);  // meridional
      kE_ = kWgs84A / std::sqrt(denom) * std::cos(lat0_);        // prime vertical
    }
    valid_ = true;
  }

  /// (lat, lon) degrees -> (N, E) metres.
  void toNE(double lat_deg, double lon_deg, double & N, double & E) const
  {
    N = kN_ * (lat_deg * M_PI / 180.0 - lat0_);
    E = kE_ * (lon_deg * M_PI / 180.0 - lon0_);
  }
  /// (N, E) metres -> (lat, lon) degrees.
  void toLatLon(double N, double E, double & lat_deg, double & lon_deg) const
  {
    lat_deg = (lat0_ + N / kN_) * 180.0 / M_PI;
    lon_deg = (lon0_ + E / kE_) * 180.0 / M_PI;
  }

private:
  bool spherical_;
  bool valid_ = false;
  double lat0_deg_ = 0.0, lon0_deg_ = 0.0;
  double lat0_ = 0.0, lon0_ = 0.0;
  double kN_ = kWgs84A, kE_ = kWgs84A;
};

}  // namespace hyu_localization

#endif  // HYU_LOCALIZATION__LOCAL_PROJECTION_HPP_
