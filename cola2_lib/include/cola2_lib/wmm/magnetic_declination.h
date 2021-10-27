/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

/**
 * @file
 * @brief This file contains C++ functions related to the World Magnetic Model.
 * @addtogroup wmm
 * @{
 */

#pragma once

/**
 * @brief This function returns the magnetic declination in degrees given the latitude and longitude in degrees,
 * and the date. Geoid height is assumed.
 *
 * @param latitude Latitude in degrees [-90.0, 90.0].
 * @param longitude Longitude in degrees [-180.0, 180.0].
 * @param year Year (must match the available years of the magnetic model).
 * @param month Month [1, 12].
 * @param day Day (range according to month and year).
 * @return Returns the magnetic declination in degrees.
 */
double computeMagneticDeclinationDeg(const double latitude, const double longitude, const int year, const int month,
                                     const int day);

/**
 * @brief This function returns the magnetic declination in radians given the latitude and longitude in radians,
 * and the date. Geoid height is assumed.
 *
 * @param latitude Latitude in radians [-PI/2, PI/2].
 * @param longitude Longitude in radians [-PI, PI].
 * @param year Year (must match the available years of the magnetic model).
 * @param month Month [1, 12].
 * @param day Day (range according to month and year).
 * @return Returns the magnetic declination in radians.
 */
double computeMagneticDeclination(const double latitude, const double longitude, const int year, const int month,
                                  const int day);

/** @} */
