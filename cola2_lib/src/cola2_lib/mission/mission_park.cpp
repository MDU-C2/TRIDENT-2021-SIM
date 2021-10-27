/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#include <cola2_lib/mission/mission_park.h>

MissionPark::MissionPark()
  : MissionManeuver(PARK_MANEUVER)
{
}

MissionPark::~MissionPark()
{
}

double MissionPark::getFinalLatitude() const
{
  return final_latitude_;
}

double MissionPark::getFinalLongitude() const
{
  return final_longitude_;
}

double MissionPark::getFinalDepth() const
{
  return final_depth_;
}

double MissionPark::getFinalAltitude() const
{
  return final_altitude_;
}

double MissionPark::getFinalYaw() const
{
  return final_yaw_;
}

bool MissionPark::getUseYaw() const
{
  return use_yaw_;
}

MissionPark::HeaveMode MissionPark::getHeaveMode() const
{
  return heave_mode_;
}

double MissionPark::getSurgeVelocity() const
{
  return surge_velocity_;
}

double MissionPark::getTime() const
{
  return time_;
}

bool MissionPark::getNoAltitudeGoesUp() const
{
  return no_altitude_goes_up_;
}

void MissionPark::setFinalLatitude(const double final_latitude)
{
  final_latitude_ = final_latitude;
}

void MissionPark::setFinalLongitude(const double final_longitude)
{
  final_longitude_ = final_longitude;
}

void MissionPark::setFinalDepth(const double final_depth)
{
  final_depth_ = final_depth;
}

void MissionPark::setFinalAltitude(const double final_altitude)
{
  final_altitude_ = final_altitude;
}

void MissionPark::setFinalYaw(const double final_yaw)
{
  final_yaw_ = final_yaw;
}

void MissionPark::setUseYaw(const bool use_yaw)
{
  use_yaw_ = use_yaw;
}

void MissionPark::setHeaveMode(const MissionPark::HeaveMode heave_mode)
{
  heave_mode_ = heave_mode;
}

void MissionPark::setSurgeVelocity(const double surge_velocity)
{
  surge_velocity_ = surge_velocity;
}

void MissionPark::setTime(const double t)
{
  time_ = t;
}

void MissionPark::setNoAltitudeGoesUp(const bool no_altitude_goes_up)
{
  no_altitude_goes_up_ = no_altitude_goes_up;
}

double MissionPark::x()
{
  return final_latitude_;
}

double MissionPark::y()
{
  return final_longitude_;
}

double MissionPark::z()
{
  return final_depth_;
}
