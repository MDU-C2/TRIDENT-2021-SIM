/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#include <cola2_safety/safety_rules/comms.h>
#include <cola2_lib_ros/param_loader.h>

namespace SafetyRules
{
  Comms::Comms(const std::string& rule_name)
  : SafetyRuleBaseClass(rule_name)
  , last_modem_data_(ros::Time::now().toSec())
  , modem_data_timeout_(false)
{
  loadConfigFromParamServer();

  const ParseList parse_list({
    {std::string("/safety/") + rule_name_, "last_modem_data", "last_modem_data", DataType::Double},
    {std::string("/safety/") + rule_name_, "modem_recovery_action", "modem_recovery_action", DataType::Int}
  });
  setParseList(parse_list);
}

void
Comms::periodicUpdate(const ros::Time&, std::uint32_t*)
{
  // Clear level and message
  level_ = SafetyLevel::NONE;
  message_.clear();

  // Check data
  if (last_valid_config_ == 0.0)
  {
    level_ = SafetyLevel::ABORT_AND_SURFACE;
    message_ = createMessage("invalid config");
  }
  else if (hasDouble("last_modem_data") &&
          (getDouble("last_modem_data") > modem_data_timeout_) &&
          (!hasInt("modem_recovery_action") || (getInt("modem_recovery_action") < static_cast<int>(SafetyLevel::ABORT_AND_SURFACE))))
  {
    level_ = SafetyLevel::ABORT_AND_SURFACE;
    message_ = createMessage("modem data age above threshold");
  }
  else if (hasInt("modem_recovery_action"))
  {
    const int safety_level = getInt("modem_recovery_action");
    if ((safety_level >= static_cast<int>(SafetyLevel::INFORMATIVE)) &&
        (safety_level <= static_cast<int>(SafetyLevel::DROP_WEIGHT)))
    {
      level_ = static_cast<SafetyLevel::Type>(safety_level);
      message_ = createMessage("modem recovery action");
    }
  }
}

bool
Comms::loadConfigFromParamServer()
{
  // Load config from param server
  double temp_modem_data_timeout;
  bool ok = true;
  ok &= cola2::ros::getParam(std::string("~") + rule_name_ + std::string("/modem_data_timeout"), temp_modem_data_timeout);

  // Check if valid
  if (!ok)
  {
    ROS_ERROR_STREAM(createMessage("invalid parameters! No changes applied"));
    return false;
  }
  modem_data_timeout_ = temp_modem_data_timeout;
  last_valid_config_ = ros::Time::now().toSec();
  return ok;
}
}  // namespace SafetyRules
