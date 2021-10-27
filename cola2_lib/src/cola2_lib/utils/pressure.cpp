/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#include <cola2_lib/utils/pressure.h>
#include <cassert>

double cola2::utils::pascalsToMeters(const double pascals, const double water_density)
{
  assert((water_density >= 900.0) && (water_density <= 1200));
  return pascals / (water_density * 9.80665);
}

double cola2::utils::metersToPascals(const double meters, const double water_density)
{
  assert((water_density >= 900.0) && (water_density <= 1200));
  return meters * water_density * 9.80665;
}

double cola2::utils::barsToPascals(const double bars)
{
  return 100000.0 * bars;
}

double cola2::utils::pascalsToBars(const double pascals)
{
  return 0.00001 * pascals;
}

double cola2::utils::barsToMeters(const double bars, const double water_density)
{
  const double pascals = cola2::utils::barsToPascals(bars);
  return cola2::utils::pascalsToMeters(pascals, water_density);
}

double cola2::utils::metersToBars(const double meters, const double water_density)
{
  const double pascals = cola2::utils::metersToPascals(meters, water_density);
  return cola2::utils::pascalsToBars(pascals);
}
