/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

/**
 * @file
 * @brief C++ mission maneuver base definition.
 * @addtogroup mission
 * @{
 */

#pragma once

/**
 * @brief Defines a base class for a mission maneuver.
 */
class MissionManeuver
{
protected:
  unsigned int maneuver_type_;  //!< Type of maneuver.

public:
  static const unsigned int GOTO_MANEUVER = 0;     //!< Goto maneuver type.
  static const unsigned int SECTION_MANEUVER = 1;  //!< Section meneuver type.
  static const unsigned int PARK_MANEUVER = 2;     //!< Park maneuver type.

  /**
   * @brief Constructor taking maneuver type.
   *
   * @param maneuver_type Maneuver type.
   */
  explicit MissionManeuver(const unsigned int maneuver_type);
  ~MissionManeuver();

  /**
   * @brief Get the maneuver type.
   *
   * @return Maneuver type.
   */
  unsigned int getManeuverType() const;

  // This is used in the captain to create a path

  /**
   * @brief Get the maneuver x.
   *
   * @return Maneuver x.
   */
  virtual double x() = 0;

  /**
   * @brief Get the maneuver y.
   *
   * @return Maneuver y.
   */
  virtual double y() = 0;

  /**
   * @brief Get the maneuver z.
   *
   * @return Maneuver z.
   */
  virtual double z() = 0;
};

/** @} */
