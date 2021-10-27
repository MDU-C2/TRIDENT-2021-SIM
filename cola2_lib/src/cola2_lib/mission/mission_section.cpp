/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#include <cola2_lib/mission/mission_section.h>

MissionSection::MissionSection()
  : MissionManeuver(SECTION_MANEUVER)
{
}

MissionSection::~MissionSection()
{
}

double MissionSection::getInitialLatitude() const
{
  return initial_latitude_;
}

double MissionSection::getInitialLongitude() const
{
  return initial_longitude_;
}

double MissionSection::getInitialDepth() const
{
  return initial_depth_;
}

double MissionSection::getFinalLatitude() const
{
  return final_latitude_;
}

double MissionSection::getFinalLongitude() const
{
  return final_longitude_;
}

double MissionSection::getFinalDepth() const
{
  return final_depth_;
}

double MissionSection::getFinalAltitude() const
{
  return final_altitude_;
}

MissionSection::HeaveMode MissionSection::getHeaveMode() const
{
  return heave_mode_;
}

double MissionSection::getSurgeVelocity() const
{
  return surge_velocity_;
}

double MissionSection::getToleranceXY() const
{
  return tolerance_xy_;
}

bool MissionSection::getNoAltitudeGoesUp() const
{
  return no_altitude_goes_up_;
}

void MissionSection::setInitialLatitude(const double initial_latitude)
{
  initial_latitude_ = initial_latitude;
}

void MissionSection::setInitialLongitude(const double initial_longitude)
{
  initial_longitude_ = initial_longitude;
}

void MissionSection::setInitialDepth(const double initial_depth)
{
  initial_depth_ = initial_depth;
}

void MissionSection::setFinalLatitude(const double final_latitude)
{
  final_latitude_ = final_latitude;
}

void MissionSection::setFinalLongitude(const double final_longitude)
{
  final_longitude_ = final_longitude;
}

void MissionSection::setFinalDepth(const double final_depth)
{
  final_depth_ = final_depth;
}

void MissionSection::setFinalAltitude(const double final_altitude)
{
  final_altitude_ = final_altitude;
}

void MissionSection::setHeaveMode(const MissionSection::HeaveMode heave_mode)
{
  heave_mode_ = heave_mode;
}

void MissionSection::setSurgeVelocity(const double surge_velocity)
{
  surge_velocity_ = surge_velocity;
}

void MissionSection::setToleranceXY(const double tolerance_xy)
{
  tolerance_xy_ = tolerance_xy;
}

void MissionSection::setNoAltitudeGoesUp(const bool no_altitude_goes_up)
{
  no_altitude_goes_up_ = no_altitude_goes_up;
}

double MissionSection::x()
{
  return final_latitude_;
}

double MissionSection::y()
{
  return final_longitude_;
}

double MissionSection::z()
{
  return final_depth_;
}
