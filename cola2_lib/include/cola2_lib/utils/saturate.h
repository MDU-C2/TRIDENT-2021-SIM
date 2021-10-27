/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

/**
 * @file
 * @brief This file contains functions to saturate a variable.
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
 * @brief Saturates a value to the range [-min_max, min_max].
 *
 * @param x Input value.
 * @param min_max Absolute value limit.
 * @return Returns the original value if its absolute value is less than the specified limit. Otherwise, the value is
 * saturated to the specified limit.
 */
double saturate(const double x, const double min_max);

/**
 * @brief Saturates a value to the range [min_value, max_value].
 *
 * @param value Reference to the value that must be saturated between the min and max limit values.
 * @param max_value Upper limit.
 * @param min_value Lower limit.
 */
void saturate(double &value, const double max_value, const double min_value);

/** @} */
}  // namespace utils
}  // namespace cola2
