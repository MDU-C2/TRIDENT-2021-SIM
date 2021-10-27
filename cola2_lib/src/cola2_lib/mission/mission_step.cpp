/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#include <cola2_lib/mission/mission_step.h>

MissionStep::MissionStep()
{
}

MissionStep::~MissionStep()
{
}

std::shared_ptr<MissionManeuver> MissionStep::getManeuverPtr() const
{
  return maneuver_;
}

std::vector<MissionAction> MissionStep::getActions() const
{
  return actions_;
}

void MissionStep::setManeuverPtr(std::shared_ptr<MissionManeuver> maneuver)
{
  maneuver_ = maneuver;
}

void MissionStep::addAction(const MissionAction& action)
{
  actions_.push_back(action);
}
