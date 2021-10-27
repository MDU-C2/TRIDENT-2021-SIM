/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

/**
 * @file
 * @brief C++ mission goto maneuver definition.
 * @addtogroup mission
 * @{
 */

#pragma once

#include <cola2_lib/mission/mission_maneuver.h>

/**
 * @brief Implements a mission maneuver to go to a destination point.
 */
class MissionGoto : public MissionManeuver
{
public:
  /**
   * @brief Heav modes to go to the destination point.
   */
  enum class HeaveMode
  {
    DEPTH,     //!< Control the vehile in depth.
    ALTITUDE,  //!< Control the vehicle in altitude.
    BOTH       //!< Controls the vehicle in altitude if available. Otherwise, it controls the vehicle in depth.
  };

  /**
   * @brief Empty constructor
   */
  MissionGoto();
  ~MissionGoto();

  /**
   * @brief Get final latitude.
   *
   * @return Final latitude in decimal degrees.
   */
  double getFinalLatitude() const;

  /**
   * @brief Get final longitude.
   *
   * @return Final longitude in decimal degrees.
   */
  double getFinalLongitude() const;

  /**
   * @brief Get final depth.
   *
   * @return Final depth in meters.
   */
  double getFinalDepth() const;

  /**
   * @brief Get final altitude.
   *
   * @return Final altitude in meters.
   */
  double getFinalAltitude() const;

  /**
   * @brief Get the heave mode.
   *
   * @return Heave mode.
   */
  HeaveMode getHeaveMode() const;

  /**
   * @brief Get surge velocity.
   *
   * @return Surge velocity in meters per secnd.
   */
  double getSurgeVelocity() const;

  /**
   * @brief Get final point toloreance.
   *
   * @return Final position tolerance in meters.
   */
  double getToleranceXY() const;

  /**
   * @brief Get the behaviour when altitude is not available.
   *
   * @return No altitude goes up behaviour enabled.
   */
  bool getNoAltitudeGoesUp() const;

  /**
   * @brief Set the final latitude.
   *
   * @param final_latitude Final latitude in decimal degrees.
   */
  void setFinalLatitude(const double final_latitude);

  /**
   * @brief Set final longitude.
   *
   * @param final_longitude Final longitude in decimal degrees.
   */
  void setFinalLongitude(const double final_longitude);

  /**
   * @brief Set final depth.
   *
   * @param final_depth Final depth in meters.
   */
  void setFinalDepth(const double final_depth);

  /**
   * @brief Set final altitude.
   * @param final_altitude Final altitude in meters.
   */
  void setFinalAltitude(const double final_altitude);

  /**
   * @brief Set the heave mode.
   *
   * @param heave_mode Heave mode.
   */
  void setHeaveMode(const HeaveMode heave_mode);

  /**
   * @brief Set the surge velocity.
   *
   * @param surge_velocity Surge velocity in meters per second.
   */
  void setSurgeVelocity(const double surge_velocity);

  /**
   * @brief Set final point tolerance.
   *
   * @param tolerance_xy Final ponit tolerance in meters.
   */
  void setToleranceXY(const double tolerance_xy);

  /**
   * @brief Set no altitude goes up behaviour.
   *
   * @param no_altitude_goes_up No altitude goes up behaviour.
   */
  void setNoAltitudeGoesUp(const bool no_altitude_goes_up);

  /**
   * @brief Get the maneuver x as the final latitude.
   *
   * @return Final latitude in decimal degrees.
   */
  double x();

  /**
   * @brief Get the maneuver x as the final longitude.
   *
   * @return Final longitude in decimal degrees.
   */
  double y();

  /**
   * @brief Get the maneuver z as the final depth.
   *
   * @return Final depth in meters.
   */
  double z();

protected:
  double final_latitude_;     //!< Final latitude in decimal degrees.
  double final_longitude_;    //!< Final longitude in decimal degrees.
  double final_depth_;        //!< Final depth in meters.
  double final_altitude_;     //!< Final altitude in meters.
  HeaveMode heave_mode_;      //!< Heave mode to define depth/altitude behaviour.
  double surge_velocity_;     //!< Velocity in surge in m/s.
  double tolerance_xy_;       //!< Final position tolerance in meters.
  bool no_altitude_goes_up_;  //!< Behaviour when no altitude is avalable.
};

/** @} */
