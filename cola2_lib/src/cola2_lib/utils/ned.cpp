/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#include "cola2_lib/utils/ned.h"
#include <cmath>

namespace cola2
{
namespace utils
{
NED::NED(const double lat, const double lon, const double height)
{
  // Save NED origin
  init_lat_ = degreesToRadians(lat);
  init_lon_ = degreesToRadians(lon);
  init_h_ = height;

  // Compute ECEF of NED origin
  geodetic2Ecef(lat, lon, height, init_ecef_x_, init_ecef_y_, init_ecef_z_);

  // Compute ECEF to NED and NED to ECEF matrices
  const double phiP = std::atan2(init_ecef_z_, std::sqrt(std::pow(init_ecef_x_, 2) + std::pow(init_ecef_y_, 2)));

  ecef_to_ned_matrix_ = nRe(phiP, init_lon_);
  ned_to_ecef_matrix_ = nRe(init_lat_, init_lon_).transpose();
}

void NED::NED::geodetic2Ecef(const double lat, const double lon, const double height, double &x, double &y,
                             double &z) const
{
  // Convert geodetic coordinates to ECEF.
  // http://code.google.com/p/pysatel/source/browse/trunk/coord.py?r=22
  const double lat_rad = degreesToRadians(lat);
  const double lon_rad = degreesToRadians(lon);
  const double xi = std::sqrt(1 - ESQ * std::sin(lat_rad) * std::sin(lat_rad));
  x = (A / xi + height) * std::cos(lat_rad) * std::cos(lon_rad);
  y = (A / xi + height) * std::cos(lat_rad) * std::sin(lon_rad);
  z = (A / xi * (1 - ESQ) + height) * std::sin(lat_rad);
}

void NED::NED::ecef2Geodetic(const double x, const double y, const double z, double &lat, double &lon,
                             double &height) const
{
  // Convert ECEF coordinates to geodetic.
  // J. Zhu, "Conversion of Earth-centered Earth-fixed coordinates
  // to geodetic coordinates," IEEE Transactions on Aerospace and
  // Electronic Systems, vol. 30, pp. 957-961, 1994.

  const double r = std::sqrt(x * x + y * y);
  const double Esq = A * A - B * B;
  const double FF = 54.0 * B * B * z * z;
  const double G = r * r + (1.0 - ESQ) * z * z - ESQ * Esq;
  const double C = (ESQ * ESQ * FF * r * r) / std::pow(G, 3);
  const double S = cbrt(1.0 + C + std::sqrt(C * C + 2.0 * C));
  const double P = FF / (3.0 * std::pow((S + 1.0 / S + 1.0), 2) * G * G);
  const double Q = std::sqrt(1.0 + 2.0 * ESQ * ESQ * P);
  const double r_0 =
      -(P * ESQ * r) / (1.0 + Q) +
      std::sqrt(0.5 * A * A * (1.0 + 1.0 / Q) - P * (1.0 - ESQ) * z * z / (Q * (1.0 + Q)) - 0.5 * P * r * r);
  const double U = std::sqrt(std::pow((r - ESQ * r_0), 2) + z * z);
  const double V = std::sqrt(std::pow((r - ESQ * r_0), 2) + (1.0 - ESQ) * z * z);
  const double Z_0 = B * B * z / (A * V);
  height = U * (1.0 - B * B / (A * V));
  lat = radiansToDegrees(std::atan((z + ELSQ * Z_0) / r));
  lon = radiansToDegrees(std::atan2(y, x));
}

void NED::NED::ecef2Ned(const double x, const double y, const double z, double &north, double &east,
                        double &depth) const
{
  // Converts ECEF coordinate pos into local-tangent-plane ENU
  // coordinates relative to another ECEF coordinate ref. Returns a tuple
  // (East, North, Up).

  Eigen::Vector3d vect;
  vect(0) = x - init_ecef_x_;
  vect(1) = y - init_ecef_y_;
  vect(2) = z - init_ecef_z_;
  const Eigen::Vector3d ret = ecef_to_ned_matrix_ * vect;
  north = ret(0);
  east = ret(1);
  depth = -ret(2);
}

void NED::NED::ned2Ecef(const double north, const double east, const double depth, double &x, double &y,
                        double &z) const
{
  // NED (north/east/down) to ECEF coordinate system conversion.
  Eigen::Vector3d ned;
  ned(0) = north;
  ned(1) = east;
  ned(2) = -depth;
  const Eigen::Vector3d ret = ned_to_ecef_matrix_ * ned;
  x = ret(0) + init_ecef_x_;
  y = ret(1) + init_ecef_y_;
  z = ret(2) + init_ecef_z_;
}

void NED::NED::geodetic2Ned(const double lat, const double lon, const double height, double &north, double &east,
                            double &depth) const
{
  // Geodetic position to a local NED system """
  double x, y, z;
  geodetic2Ecef(lat, lon, height, x, y, z);
  ecef2Ned(x, y, z, north, east, depth);
}

void NED::NED::ned2Geodetic(const double north, const double east, const double depth, double &lat, double &lon,
                            double &height) const
{
  // Local NED position to geodetic
  double x, y, z;
  ned2Ecef(north, east, depth, x, y, z);
  ecef2Geodetic(x, y, z, lat, lon, height);
}

double NED::NED::cbrt(const double x) const
{
  if (x >= 0.0)
  {
    return std::pow(x, 1.0 / 3.0);
  }
  else
  {
    return -std::pow(std::fabs(x), 1.0 / 3.0);
  }
}

Eigen::Matrix3d NED::NED::nRe(const double lat_rad, const double lon_rad) const
{
  const double sLat = std::sin(lat_rad);
  const double sLon = std::sin(lon_rad);
  const double cLat = std::cos(lat_rad);
  const double cLon = std::cos(lon_rad);

  Eigen::Matrix3d ret;
  ret(0, 0) = -sLat * cLon;
  ret(0, 1) = -sLat * sLon;
  ret(0, 2) = cLat;
  ret(1, 0) = -sLon;
  ret(1, 1) = cLon;
  ret(1, 2) = 0.0;
  ret(2, 0) = cLat * cLon;
  ret(2, 1) = cLat * sLon;
  ret(2, 2) = sLat;

  return ret;
}

double NED::getInitLatitude() const
{
  return radiansToDegrees(init_lat_);
}

double NED::getInitLongitude() const
{
  return radiansToDegrees(init_lon_);
}

Eigen::Vector3d NED::geodetic2Ned(const Eigen::Vector3d &geodetic) const
{
  double n, e, d;
  geodetic2Ned(geodetic(0), geodetic(1), geodetic(2), n, e, d);
  return Eigen::Vector3d(n, e, d);
}

Eigen::Vector3d NED::ned2Geodetic(const Eigen::Vector3d &ned) const
{
  double lat, lon, h;
  ned2Geodetic(ned(0), ned(1), ned(2), lat, lon, h);
  return Eigen::Vector3d(lat, lon, h);
}
}  // namespace utils
}  // namespace cola2
