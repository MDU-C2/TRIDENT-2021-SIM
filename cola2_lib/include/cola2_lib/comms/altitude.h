/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

/**
 * @file
 * @brief C++ encode/decode altitude values to/from uint8.
 * @addtogroup comms
 * @{
 */

#ifndef COLA2_COMMS_ALTITUDE_H
#define COLA2_COMMS_ALTITUDE_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

  /**
   * @brief Define values for altitude encoding.
   */
  enum
  {
    ALTITUDE_LOOKUP_SIZE = 208,     //!< size of the lookup table
    ALTITUDE_MAXIMUM = 253,         //!< value when altitude is bigger than the biggest value in lookup table
    ALTITUDE_INVALID = 254,         //!< value when no altitude data is available
    ALTITUDE_ENCODING_ERROR = 255,  //!< value for encoding errors, should not appear
  };

  /**
   * @brief Lookup table to encode and decode altitude values to and from uint8_t.
   *
   * Values are spread at different resolutions:
   *   0.25 in (0.25 - 5.00)
   *   0.50 in (5.00 - 30.0)
   *   1.00 in (30.0 - 100.)
   *   5.00 in (100. - 300.)
   *  10.00 in (300. - 500.)
   *  25.00 in (500. - 700.)
   */
  const double LOOKUP_ALTITUDE[ALTITUDE_LOOKUP_SIZE] = {
    0.25, 0.50, 0.75, 1.00, 1.25, 1.50, 1.75, 2.00, 2.25, 2.50, 2.75, 3.00, 3.25, 3.50, 3.75, 4.00,  // 0-15
    4.25, 4.50, 4.75, 5.00, 5.50, 6.00, 6.50, 7.00, 7.50, 8.00, 8.50, 9.00, 9.50, 10.0, 10.5, 11.0,  // 16-31
    11.5, 12.0, 12.5, 13.0, 13.5, 14.0, 14.5, 15.0, 15.5, 16.0, 16.5, 17.0, 17.5, 18.0, 18.5, 19.0,  // 32-47
    19.5, 20.0, 20.5, 21.0, 21.5, 22.0, 22.5, 23.0, 23.5, 24.0, 24.5, 25.0, 25.5, 26.0, 26.5, 27.0,  // 48-63
    27.5, 28.0, 28.5, 29.0, 29.5, 30.0, 31.0, 32.0, 33.0, 34.0, 35.0, 36.0, 37.0, 38.0, 39.0, 40.0,  // 64-79
    41.0, 42.0, 43.0, 44.0, 45.0, 46.0, 47.0, 48.0, 49.0, 50.0, 51.0, 52.0, 53.0, 54.0, 55.0, 56.0,  // 80-95
    57.0, 58.0, 59.0, 60.0, 61.0, 62.0, 63.0, 64.0, 65.0, 66.0, 67.0, 68.0, 69.0, 70.0, 71.0, 72.0,  // 96-111
    73.0, 74.0, 75.0, 76.0, 77.0, 78.0, 79.0, 80.0, 81.0, 82.0, 83.0, 84.0, 85.0, 86.0, 87.0, 88.0,  // 112-127
    89.0, 90.0, 91.0, 92.0, 93.0, 94.0, 95.0, 96.0, 97.0, 98.0, 99.0, 100., 105., 110., 115., 120.,  // 128-143
    125., 130., 135., 140., 145., 150., 155., 160., 165., 170., 175., 180., 185., 190., 195., 200.,  // 144-159
    205., 210., 215., 220., 225., 230., 235., 240., 245., 250., 255., 260., 265., 270., 275., 280.,  // 160-175
    285., 290., 295., 300., 310., 320., 330., 340., 350., 360., 370., 380., 390., 400., 410., 420.,  // 176-191
    430., 440., 450., 460., 470., 480., 490., 500., 525., 550., 575., 600., 625., 650., 675., 700.,  // 192-207
  };

  /**
   * @brief Encode an altitude value using the lookup table.
   *
   * @param altitude Value of the altitude in meters (-1.0 if invalid/unavailable).
   * @return uint8_t The encoded value.
   */
  uint8_t altitudeEncode(const double altitude);

  /**
   * @brief Check if the encoded value is bigger than the values in the table.
   *
   * @param encoded Encoded value.
   * @return Value bigger than the values in the table.
   */
  bool altitudeIsMaximum(const uint8_t encoded);

  /**
   * @brief Check if the altitude is not available or invalid.
   *
   * @param encoded Encoded value.
   * @return Altitude not available or invalid.
   */
  bool altitudeIsInvalid(const uint8_t encoded);

  /**
   * @brief Check if the encoded value is an encoding error because is not in the table.
   *
   * @param encoded Encoded value.
   * @return Value is an encoding error because is not in the table.
   */
  bool altitudeIsEncodingError(const uint8_t encoded);

  /**
   * @brief Decode the altitude value.
   *
   * @param encoded Encoded value.
   * @return double Lower bound of the altitude (-1.0 if it is maximum/invalid/encoding_error)
   */
  double altitudeDecode(const uint8_t encoded);

#ifdef __cplusplus
}
#endif

#endif  // COLA2_COMMS_ALTITUDE_H

/*! @} */
