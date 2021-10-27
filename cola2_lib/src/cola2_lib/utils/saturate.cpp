/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#include <cola2_lib/utils/saturate.h>

double cola2::utils::saturate(const double x, const double min_max)
{
  if (x > min_max)
    return min_max;
  else if (x < -min_max)
    return -min_max;
  return x;
}

void cola2::utils::saturate(double& value, const double max_value, const double min_value)
{
  if (value > max_value)
    value = max_value;
  if (value < min_value)
    value = min_value;
}
