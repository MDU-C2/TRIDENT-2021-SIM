/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

/**
 * @file
 * @brief C++ encode/decode elapsed time values to/from uint8.
 * @addtogroup comms
 * @{
 */

#ifndef COLA2_COMMS_ELAPSED_TIME_H
#define COLA2_COMMS_ELAPSED_TIME_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

  /**
   * @brief Define values for elapsed time encoding.
   */
  enum
  {
    ELAPSED_TIME_LOOKUP_SIZE = 176,     //!< size of the lookup table
    ELAPSED_TIME_MAXIMUM = 253,         //!< value when elapsed_time is bigger than the biggest value in lookup table
    ELAPSED_TIME_INVALID = 254,         //!< value when no elapsed_time data is available
    ELAPSED_TIME_ENCODING_ERROR = 255,  //!< value for encoding errors, should not appear
  };

  /**
   * @brief Lookup table to encode and decode altitude values to and from uint8_t.
   *
   * Values are spread at different resolutions:
   *   1 minutes in (0 - 60) minutes
   *   5 minutes in (60 - 635) minutes
   */
  const int32_t LOOKUP_ELAPSED_TIME[ELAPSED_TIME_LOOKUP_SIZE] = {
    0,   1,   2,   3,   4,   5,   6,   7,   8,   9,   10,  11,  12,  13,  14,  15,   // 0-15
    16,  17,  18,  19,  20,  21,  22,  23,  24,  25,  26,  27,  28,  29,  30,  31,   // 16-31
    32,  33,  34,  35,  36,  37,  38,  39,  40,  41,  42,  43,  44,  45,  46,  47,   // 32-47
    48,  49,  50,  51,  52,  53,  54,  55,  56,  57,  58,  59,  60,  65,  70,  75,   // 48-63
    80,  85,  90,  95,  100, 105, 110, 115, 120, 125, 130, 135, 140, 145, 150, 155,  // 64-79
    160, 165, 170, 175, 180, 185, 190, 195, 200, 205, 210, 215, 220, 225, 230, 235,  // 80-95
    240, 245, 250, 255, 260, 265, 270, 275, 280, 285, 290, 295, 300, 305, 310, 315,  // 96-111
    320, 325, 330, 335, 340, 345, 350, 355, 360, 365, 370, 375, 380, 385, 390, 395,  // 112-127
    400, 405, 410, 415, 420, 425, 430, 435, 440, 445, 450, 455, 460, 465, 470, 475,  // 128-143
    480, 485, 490, 495, 500, 505, 510, 515, 520, 525, 530, 535, 540, 545, 550, 555,  // 144-159
    560, 565, 570, 575, 580, 585, 590, 595, 600, 605, 610, 615, 620, 625, 630, 635,  // 160-175
  };

  /**
   * @brief Encode the elapsed time value using the lookup table.
   *
   * @param elapsed_time Value of the elapsed time in seconds (-1.0 if invalid/unavailable).
   * @return uint8_t The encoded value.
   */
  uint8_t elapsedTimeEncode(const int32_t elapsed_time);

  /**
   * @brief Check if the encoded value is bigger than the values in the table.
   *
   * @param encoded Encoded value.
   * @return Value bigger than the values in the table.
   */
  bool elapsedTimeIsMaximum(const uint8_t encoded);

  /**
   * @brief Check if the elapsed time is not available or invalid.
   *
   * @param encoded Encoded value.
   * @return Elapsed time not available or invalid.
   */
  bool elapsedTimeIsInvalid(const uint8_t encoded);

  /**
   * @brief Check if the encoded value is an encoding error because is not in the table.
   *
   * @param encoded Encoded value.
   * @return Value is an encoding error because is not in the table.
   */
  bool elapsedTimeIsEncodingError(const uint8_t encoded);

  /**
   * @brief Decode the elapsed time value.
   *
   * @param encoded Encoded value.
   * @return double Lower bound of the elapsed time (-1 if it is maximum/invalid/encoding_error)
   */
  int32_t elapsedTimeDecode(const uint8_t encoded);

#ifdef __cplusplus
}
#endif

#endif  // COLA2_COMMS_ELAPSED_TIME_H

/*! @} */
