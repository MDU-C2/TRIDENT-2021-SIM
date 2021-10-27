/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#include "cola2_lib/wmm/magnetic_declination.h"
#include "cola2_lib/wmmc/magnetic_declination.h"
#include <stdexcept>

double computeMagneticDeclinationDeg(const double latitude, const double longitude, const int year, const int month,
                                     const int day)
{
  double declination;
  const int error = computeMagneticDeclinationDeg(&declination, latitude, longitude, year, month, day);
  if (error == MAGNETIC_DECLINATION_LATITUDE_OUT_OF_RANGE)
    throw std::runtime_error("MAGNETIC_DECLINATION_LATITUDE_OUT_OF_RANGE");
  else if (error == MAGNETIC_DECLINATION_LONGITUDE_OUT_OF_RANGE)
    throw std::runtime_error("MAGNETIC_DECLINATION_LONGITUDE_OUT_OF_RANGE");
  else if (error == MAGNETIC_DECLINATION_COF_DATA_NOT_FOUND)
    throw std::runtime_error("MAGNETIC_DECLINATION_COF_DATA_NOT_FOUND");
  else if (error == MAGNETIC_DECLINATION_BAD_ALLOCATION)
    throw std::runtime_error("MAGNETIC_DECLINATION_BAD_ALLOCATION");
  else if (error == MAGNETIC_DECLINATION_INVALID_DATE)
    throw std::runtime_error("MAGNETIC_DECLINATION_INVALID_DATE");
  else if (error == MAGNETIC_DECLINATION_INVALID_YEAR)
    throw std::runtime_error("MAGNETIC_DECLINATION_INVALID_YEAR");
  return declination;
}

double computeMagneticDeclination(const double latitude, const double longitude, const int year, const int month,
                                  const int day)
{
  double declination;
  const int error = computeMagneticDeclination(&declination, latitude, longitude, year, month, day);
  if (error == MAGNETIC_DECLINATION_LATITUDE_OUT_OF_RANGE)
    throw std::runtime_error("MAGNETIC_DECLINATION_LATITUDE_OUT_OF_RANGE");
  else if (error == MAGNETIC_DECLINATION_LONGITUDE_OUT_OF_RANGE)
    throw std::runtime_error("MAGNETIC_DECLINATION_LONGITUDE_OUT_OF_RANGE");
  else if (error == MAGNETIC_DECLINATION_COF_DATA_NOT_FOUND)
    throw std::runtime_error("MAGNETIC_DECLINATION_COF_DATA_NOT_FOUND");
  else if (error == MAGNETIC_DECLINATION_BAD_ALLOCATION)
    throw std::runtime_error("MAGNETIC_DECLINATION_BAD_ALLOCATION");
  else if (error == MAGNETIC_DECLINATION_INVALID_DATE)
    throw std::runtime_error("MAGNETIC_DECLINATION_INVALID_DATE");
  else if (error == MAGNETIC_DECLINATION_INVALID_YEAR)
    throw std::runtime_error("MAGNETIC_DECLINATION_INVALID_YEAR");
  return declination;
}
