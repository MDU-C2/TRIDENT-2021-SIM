/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#include "altitude.h"

uint8_t altitudeEncode(const double altitude)
{
  // invalid altitude
  if (altitude < 0.0)
  {
    return ALTITUDE_INVALID;
  }
  // more than last value
  if (altitude > LOOKUP_ALTITUDE[ALTITUDE_LOOKUP_SIZE - 1])
  {
    return ALTITUDE_MAXIMUM;
  }
  // less than first value
  if (altitude <= LOOKUP_ALTITUDE[0])
  {
    return 0;
  }
  // find exact code
  for (uint8_t i = 1; i < ALTITUDE_LOOKUP_SIZE; i++)
  {
    if (altitude < LOOKUP_ALTITUDE[i])
    {
      return i - 1;
    }
  }
  // should never arrive here
  return ALTITUDE_ENCODING_ERROR;
}

bool altitudeIsMaximum(const uint8_t encoded)
{
  return encoded == ALTITUDE_MAXIMUM;
}
bool altitudeIsInvalid(const uint8_t encoded)
{
  return encoded == ALTITUDE_INVALID;
}
bool altitudeIsEncodingError(const uint8_t encoded)
{
  return ((encoded >= ALTITUDE_LOOKUP_SIZE) && (!altitudeIsMaximum(encoded)) && (!altitudeIsInvalid(encoded)));
}

double altitudeDecode(const uint8_t encoded)
{
  // not a normal value
  if (altitudeIsMaximum(encoded) || altitudeIsInvalid(encoded) || altitudeIsEncodingError(encoded))
  {
    return -1.0;
  }
  // normal value
  return LOOKUP_ALTITUDE[encoded];
}
