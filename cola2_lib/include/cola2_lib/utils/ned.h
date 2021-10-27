/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

/**
 * @file
 * @brief This file contains a class to aid in the conversion to NED coordinates.
 */

#pragma once

#include <cola2_lib/utils/angles.h>
#include <Eigen/Dense>

namespace cola2
{
namespace utils
{
/**
 * @addtogroup utils
 * @{
 */

// Parameters of the WGS-84 ellipsoid
const double A = 6378137.0;
const double B = 6356752.3142;
const double ESQ = 6.69437999014 * 0.001;
const double ELSQ = 6.73949674228 * 0.001;
const double F = 1.0 / 298.257223563;

/**
 * @brief Class to represent a North East Down coordinate system and transform to/from it.
 */
class NED
{
protected:
  double init_lat_;
  double init_lon_;
  double init_h_;
  double init_ecef_x_;
  double init_ecef_y_;
  double init_ecef_z_;
  Eigen::Matrix3d ecef_to_ned_matrix_;
  Eigen::Matrix3d ned_to_ecef_matrix_;

  double cbrt(const double x) const;

  Eigen::Matrix3d nRe(const double lat_rad, const double lon_rad) const;

public:
  /**
   * @brief NED constructor.
   *
   * @param lat Latitude (degrees).
   * @param lon Longitude (degrees).
   * @param height Geodetic height (meters).
   */
  NED(const double lat, const double lon, const double height);

  /**
   * @brief Convert from geodetic representation (GPS) to ECEF.
   *
   * @param lat Latitude (degrees).
   * @param lon Longitude (degrees).
   * @param height Geodetic height (meters).
   * @param x Output ECEF x coordinate (meters).
   * @param y Output ECEF y coordinate (meters).
   * @param z Output ECEF z coordinate (meters).
   */
  void geodetic2Ecef(const double lat, const double lon, const double height, double &x, double &y, double &z) const;

  /**
   * @brief Convert from ECEF representation to geodetic (GPS).
   *
   * @param x ECEF x coordinate (meters).
   * @param y ECEF y coordinate (meters).
   * @param z ECEF z coordinate (meters).
   * @param lat Output latitude (degrees).
   * @param lon Output longitude (degrees).
   * @param height Output geodetic height (meters).
   */
  void ecef2Geodetic(const double x, const double y, const double z, double &lat, double &lon, double &height) const;

  /**
   * @brief Convert from ECEF representation to NED.
   *
   * @param x ECEF x coordinate (meters).
   * @param y ECEF y coordinate (meters).
   * @param z ECEF z coordinate (meters).
   * @param north Output NED north coordinate (meters).
   * @param east Output NED east coordinate (meters).
   * @param depth Output NED depth coordinate (meters).
   */
  void ecef2Ned(const double x, const double y, const double z, double &north, double &east, double &depth) const;

  /**
   * @brief Convert from NED representation to ECEF.
   *
   * @param north NED north coordinate (meters).
   * @param east NED east coordinate (meters).
   * @param depth NED depth coordinate (meters).
   * @param x Output ECEF x coordinate (meters).
   * @param y Output ECEF y coordinate (meters).
   * @param z Output ECEF z coordinate (meters).
   */
  void ned2Ecef(const double north, const double east, const double depth, double &x, double &y, double &z) const;

  /**
   * @brief Convert from geodetic representation (GPS) to NED.
   *
   * @param lat Latitude (degrees).
   * @param lon Longitude (degrees).
   * @param height Geodetic height (meters).
   * @param north Output NED north coordinate (meters).
   * @param east Output NED east coordinate (meters).
   * @param depth Output NED depth coordinate (meters).
   */
  void geodetic2Ned(const double lat, const double lon, const double height, double &north, double &east,
                    double &depth) const;

  /**
   * @brief Convert from geodetic representation (GPS) to NED.
   *
   * @param geodetic Vector of geodetic coordinates (lat, lon, height) in degrees and meters.
   * @return Returns a vector of NED coordinates (north, east, depth) in meters.
   */
  Eigen::Vector3d geodetic2Ned(const Eigen::Vector3d &geodetic) const;

  /**
   * @brief Convert from NED representation to geodetic (GPS).
   *
   * @param north NED north coordinate (meters).
   * @param east NED east coordinate (meters).
   * @param depth NED depth coordinate (meters).
   * @param lat Output latitude (degrees).
   * @param lon Output longitude (degrees).
   * @param height Output geodetic height (meters).
   */
  void ned2Geodetic(const double north, const double east, const double depth, double &lat, double &lon,
                    double &height) const;

  /**
   * @brief Convert from NED representation to geodetic (GPS).
   *
   * @param ned Vector of NED coordinates (north, east, depth) in meters.
   * @return Returns a vector of geodetic coordinates (lat, lon, height) in degrees and meters.
   */
  Eigen::Vector3d ned2Geodetic(const Eigen::Vector3d &ned) const;

  /**
   * @brief Get initial latitude.
   *
   * @return Returns the initial latitude.
   */
  double getInitLatitude() const;

  /**
   * @brief Get initial longitude.
   *
   * @return Returns the initial longitude.
   */
  double getInitLongitude() const;
};

/** @} */
}  // namespace utils
}  // namespace cola2
