/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#include "elapsed_time.h"

uint8_t elapsedTimeEncode(const int32_t elapsed_time)
{
  // invalid elapsed time
  if (elapsed_time < 0)
  {
    return ELAPSED_TIME_INVALID;
  }
  // to minutes
  const int32_t elapsed_time_minutes = elapsed_time / 60;
  // more than last value
  if (elapsed_time_minutes > LOOKUP_ELAPSED_TIME[ELAPSED_TIME_LOOKUP_SIZE - 1])
  {
    return ELAPSED_TIME_MAXIMUM;
  }
  // less than first value
  if (elapsed_time_minutes <= LOOKUP_ELAPSED_TIME[0])
  {
    return 0;
  }
  // find exact code
  for (uint8_t i = 1; i < ELAPSED_TIME_LOOKUP_SIZE; i++)
  {
    if (elapsed_time_minutes < LOOKUP_ELAPSED_TIME[i])
    {
      return i - 1;
    }
  }
  // should never arrive here
  return ELAPSED_TIME_ENCODING_ERROR;
}

bool elapsedTimeIsMaximum(const uint8_t encoded)
{
  return encoded == ELAPSED_TIME_MAXIMUM;
}
bool elapsedTimeIsInvalid(const uint8_t encoded)
{
  return encoded == ELAPSED_TIME_INVALID;
}
bool elapsedTimeIsEncodingError(const uint8_t encoded)
{
  return ((encoded >= ELAPSED_TIME_LOOKUP_SIZE) && (!elapsedTimeIsMaximum(encoded)) &&
          (!elapsedTimeIsInvalid(encoded)));
}

int32_t elapsedTimeDecode(const uint8_t encoded)
{
  // not a normal value
  if (elapsedTimeIsMaximum(encoded) || elapsedTimeIsInvalid(encoded) || elapsedTimeIsEncodingError(encoded))
  {
    return -1;
  }
  // normal value
  return LOOKUP_ELAPSED_TIME[encoded] * 60;  // to seconds
}
