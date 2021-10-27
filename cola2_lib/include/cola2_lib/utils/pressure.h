/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

/**
 * @file
 * @brief This file contains functions related to pressure conversions.
 */

#pragma once

namespace cola2
{
namespace utils
{
/**
 * @addtogroup utils
 * @{
 */

/**
 * @brief Conversion from pressure in pascals to meters of depth.
 *
 * @param pascals Pressure in pascals.
 * @param water_density Water density in kg/m^3 in range [900, 1200].
 * @return Meters of depth.
 */
double pascalsToMeters(const double pascals, const double water_density);

/**
 * @brief Conversion from meters of depth to pressure in pascals.
 *
 * @param meters Meters of depth.
 * @param water_density Water density in kg/m^3 in range [900, 1200].
 * @return Pressure in pascals.
 */
double metersToPascals(const double meters, const double water_density);

/**
 * @brief Conversion from pressure in bars to pascals.
 *
 * @param bars Pressure in bars.
 * @return Pressure in pascals.
 */
double barsToPascals(const double bars);

/**
 * @brief Conversion from pressure in pascals to bars.
 *
 * @param pascals Pressure in pascals.
 * @return Pressure in bars.
 */
double pascalsToBars(const double pascals);

/**
 * @brief Conversion from pressure in bars to meters of depth.
 *
 * @param bars Pressure in bars.
 * @param water_density Water density in kg/m^3 in range [900, 1200].
 * @return Meters of depth.
 */
double barsToMeters(const double bars, const double water_density);

/**
 * @brief Conversion from meters of depth to pressure in bars.
 *
 * @param meters Meters of depth.
 * @param water_density Water density in kg/m^3 in range [900, 1200].
 * @return Pressure in bars.
 */
double metersToBars(const double meters, const double water_density);

/** @} */
}  // namespace utils
}  // namespace cola2
