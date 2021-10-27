/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#include <cola2_lib/mission/mission_action.h>

MissionAction::MissionAction()
  : is_trigger_(true)
{
}

MissionAction::MissionAction(const std::string& action_id, const std::vector<std::string>& parameters)
  : action_id_(action_id), parameters_(parameters), is_trigger_(parameters_.empty())
{
}

MissionAction::~MissionAction()
{
}

std::string MissionAction::getActionId() const
{
  return action_id_;
}

std::vector<std::string> MissionAction::getParameters() const
{
  return parameters_;
}

bool MissionAction::getIsTrigger() const
{
  return is_trigger_;
}

void MissionAction::setActionId(const std::string& action_id)
{
  action_id_ = action_id;
}

void MissionAction::addParameters(const std::string& param)
{
  parameters_.push_back(param);
  is_trigger_ = false;
}
