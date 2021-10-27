/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#include <cola2_lib/mission/mission_maneuver.h>

MissionManeuver::MissionManeuver(const unsigned int type)
  : maneuver_type_(type)
{
}

MissionManeuver::~MissionManeuver()
{
}

unsigned int MissionManeuver::getManeuverType() const
{
  return maneuver_type_;
}
