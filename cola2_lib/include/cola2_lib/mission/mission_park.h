/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

/**
 * @file
 * @brief C++ mission park maneuver definition.
 * @addtogroup mission
 * @{
 */

#pragma once

#include <cola2_lib/mission/mission_maneuver.h>

/**
 * @brief Implements a mission maneuver to stay at a specific point during some time.
 */
class MissionPark : public MissionManeuver
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
  MissionPark();
  ~MissionPark();

  /**
   * @brief Get final goto latitude.
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
   * @brief Get final orientation w.r.t. the north.
   *
   * @return Final yaw in radians.
   */
  double getFinalYaw() const;

  /**
   * @brief Get use yaw.
   *
   * @return True/false depending on yaw usage during park.
   */
  bool getUseYaw() const;

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
   * @brief Get park time.
   *
   * @return Park time in seconds.
   */
  double getTime() const;

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
   *
   * @param final_altitude Final altitude in meters.
   */
  void setFinalAltitude(const double final_altitude);

  /**
   * @brief Set final yaw.
   *
   * @param final_yaw Final yaw in radians.
   */
  void setFinalYaw(const double final_yaw);

  /**
   * @brief Set use yaw.
   *
   * @param use_yaw Use yaw behaviour.
   */
  void setUseYaw(const bool use_yaw);

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
   * @brief Set the park time.
   *
   * @param time Park time in seconds.
   */
  void setTime(const double time);

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
  double final_yaw_;          //!< Final orietation w.r.t. north in radians.
  bool use_yaw_;              //!< Use yaw during park.
  HeaveMode heave_mode_;      //!< Heave mode to define depth/altitude behaviour.
  double surge_velocity_;     //!< Velocity in surge in m/s.
  double time_;               //!< Park duration in seconds.
  bool no_altitude_goes_up_;  //!< Behaviour when no altitude is avalable.
};

/** @} */
