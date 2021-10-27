/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

/**
 * @file
 * @brief This file contains C functions related to the World Magnetic Model.
 * @addtogroup wmm
 * @{
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Possible errors.
 */
enum
{
  MAGNETIC_DECLINATION_OK = 0,
  MAGNETIC_DECLINATION_LATITUDE_OUT_OF_RANGE = -1,
  MAGNETIC_DECLINATION_LONGITUDE_OUT_OF_RANGE = -2,
  MAGNETIC_DECLINATION_COF_DATA_NOT_FOUND = -3,
  MAGNETIC_DECLINATION_BAD_ALLOCATION = -4,
  MAGNETIC_DECLINATION_INVALID_DATE = -5,
  MAGNETIC_DECLINATION_INVALID_YEAR = -6
};

/**
 * @brief This function returns the magnetic declination in degrees given the latitude and longitude in degrees,
 * and the date. Geoid height is assumed.
 *
 * @param declination Magnetic declination in degrees.
 * @param latitude Latitude in degrees [-90.0, 90.0].
 * @param longitude Longitude in degrees [-180.0, 180.0].
 * @param year Year (must match the available years of the magnetic model).
 * @param month Month [1, 12].
 * @param day Day (range according to month and year).
 * @return Returns the error code.
 */
int computeMagneticDeclinationDeg(double *declination, const double latitude, const double longitude, const int year,
                                  const int month, const int day);

/**
 * @brief This function returns the magnetic declination in radians given the latitude and longitude in radians,
 * and the date. Geoid height is assumed.
 *
 * @param declination Magnetic declination in radians.
 * @param latitude Latitude in radians [-PI/2, PI/2].
 * @param longitude Longitude in radians [-PI, PI].
 * @param year Year (must match the available years of the magnetic model).
 * @param month Month [1, 12].
 * @param day Day (range according to month and year).
 * @return Returns the error code.
 */
int computeMagneticDeclination(double *declination, const double latitude, const double longitude, const int year,
                               const int month, const int day);

#ifdef __cplusplus
}
#endif

/** @} */
