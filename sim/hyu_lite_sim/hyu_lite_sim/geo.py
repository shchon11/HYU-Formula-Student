"""Local tangent-plane projection, bit-for-bit the one sbg_raw_ekf uses.

hyu_localization/local_projection.hpp: WGS84 meridional / prime-vertical radii
at the origin, linear in (dlat, dlon). The simulator's world frame is that ENU
plane at the datum, so with the EKF given the same datum its output frame IS
the simulator's world frame (up to sensor noise).
"""
import math

WGS84_A = 6378137.0
WGS84_E2 = 6.69437999014e-3


class LocalProjection:
    def __init__(self, lat0_deg: float, lon0_deg: float):
        self.lat0_deg = float(lat0_deg)
        self.lon0_deg = float(lon0_deg)
        self._lat0 = math.radians(self.lat0_deg)
        self._lon0 = math.radians(self.lon0_deg)
        s = math.sin(self._lat0)
        denom = 1.0 - WGS84_E2 * s * s
        self.k_n = WGS84_A * (1.0 - WGS84_E2) / denom ** 1.5     # metres per rad of latitude
        self.k_e = WGS84_A / math.sqrt(denom) * math.cos(self._lat0)  # metres per rad of longitude

    def to_ne(self, lat_deg: float, lon_deg: float):
        return (self.k_n * (math.radians(lat_deg) - self._lat0),
                self.k_e * (math.radians(lon_deg) - self._lon0))

    def to_latlon(self, n: float, e: float):
        return (math.degrees(self._lat0 + n / self.k_n),
                math.degrees(self._lon0 + e / self.k_e))

    def enu_to_latlon(self, x_east: float, y_north: float):
        return self.to_latlon(y_north, x_east)


def wrap_pi(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def enu_yaw_to_ned_heading_deg(yaw_enu: float) -> float:
    """ENU yaw (CCW from +x=East) -> NED heading (CW from North), degrees [0,360)."""
    return (90.0 - math.degrees(yaw_enu)) % 360.0
