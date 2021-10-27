/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#include <cola2_lib/mission/mission_goto.h>

MissionGoto::MissionGoto()
  : MissionManeuver(GOTO_MANEUVER)
{
}

MissionGoto::~MissionGoto()
{
}

double MissionGoto::getFinalLatitude() const
{
  return final_latitude_;
}

double MissionGoto::getFinalLongitude() const
{
  return final_longitude_;
}

double MissionGoto::getFinalDepth() const
{
  return final_depth_;
}

double MissionGoto::getFinalAltitude() const
{
  return final_altitude_;
}

MissionGoto::HeaveMode MissionGoto::getHeaveMode() const
{
  return heave_mode_;
}

double MissionGoto::getSurgeVelocity() const
{
  return surge_velocity_;
}

double MissionGoto::getToleranceXY() const
{
  return tolerance_xy_;
}

bool MissionGoto::getNoAltitudeGoesUp() const
{
  return no_altitude_goes_up_;
}

void MissionGoto::setFinalLatitude(const double final_latitude)
{
  final_latitude_ = final_latitude;
}

void MissionGoto::setFinalLongitude(const double final_longitude)
{
  final_longitude_ = final_longitude;
}

void MissionGoto::setFinalDepth(const double final_depth)
{
  final_depth_ = final_depth;
}

void MissionGoto::setFinalAltitude(const double final_altitude)
{
  final_altitude_ = final_altitude;
}

void MissionGoto::setHeaveMode(const MissionGoto::HeaveMode heave_mode)
{
  heave_mode_ = heave_mode;
}

void MissionGoto::setSurgeVelocity(const double surge_velocity)
{
  surge_velocity_ = surge_velocity;
}

void MissionGoto::setToleranceXY(const double tolerance_xy)
{
  tolerance_xy_ = tolerance_xy;
}

void MissionGoto::setNoAltitudeGoesUp(const bool no_altitude_goes_up)
{
  no_altitude_goes_up_ = no_altitude_goes_up;
}

double MissionGoto::x()
{
  return final_latitude_;
}

double MissionGoto::y()
{
  return final_longitude_;
}

double MissionGoto::z()
{
  return final_depth_;
}
