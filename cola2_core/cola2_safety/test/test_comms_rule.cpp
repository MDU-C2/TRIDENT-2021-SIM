/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#include <cola2_safety/safety_rules/common.h>
#include <cola2_safety/safety_rules/comms.h>
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <cstdint>
#include <string>

TEST(TESTSuite, test)
{
  // Node handle
  ros::NodeHandle nh("~");

  // Create error codes message
  std::string error_codes;

  // Create fake diagnostic msg
  diagnostic_msgs::DiagnosticArray diagnostic_array;
  diagnostic_array.header.stamp = ros::Time::now();
  diagnostic_msgs::DiagnosticStatus diagnostic_status;
  diagnostic_status.level = diagnostic_msgs::DiagnosticStatus::OK;
  diagnostic_status.name = "/safety/comms";
  diagnostic_status.message = "test_message";
  diagnostic_status.hardware_id = "test_hardware_id";
  diagnostic_msgs::KeyValue diagnostic_key_value;
  diagnostic_key_value.key = "last_modem_data";
  diagnostic_key_value.value = "0.0";
  diagnostic_status.values.push_back(diagnostic_key_value);
  diagnostic_key_value.key = "modem_recovery_action";
  diagnostic_key_value.value = "0";
  diagnostic_status.values.push_back(diagnostic_key_value);
  diagnostic_array.status.push_back(diagnostic_status);

  // Create rule
  SafetyRules::Comms comms("comms");
  if (comms.getLevel() != SafetyRules::SafetyLevel::NONE)
    error_codes += "01 ";

  // Without proper config in param server
  std::uint32_t status_code = 0;
  comms.periodicUpdate(ros::Time::now(), &status_code);
  if (comms.getLevel() != SafetyRules::SafetyLevel::ABORT_AND_SURFACE)
    error_codes += "02 ";

  // From now on, with proper config
  double timeout = 60.0;
  ros::param::set("~comms/modem_data_timeout", timeout);
  comms.loadConfigFromParamServer();

  // Simulate no diagnostics at the beginning (running fine)
  comms.periodicUpdate(ros::Time::now(), &status_code);
  if (comms.getLevel() != SafetyRules::SafetyLevel::NONE)
    error_codes += "03 ";

  // Simulate modem timeout
  diagnostic_array.status[0].values[0].value = std::to_string(timeout + 1.0);
  comms.diagnosticsUpdate(diagnostic_array);
  comms.periodicUpdate(ros::Time::now(), &status_code);
  if (comms.getLevel() != SafetyRules::SafetyLevel::ABORT_AND_SURFACE)
    error_codes += "04 ";

  // Simulate modem recovery action
  diagnostic_array.status[0].values[0].value = "0.0";
  diagnostic_array.status[0].values[1].value = "3";
  comms.diagnosticsUpdate(diagnostic_array);
  comms.periodicUpdate(ros::Time::now(), &status_code);
  if (comms.getLevel() != SafetyRules::SafetyLevel::ABORT_AND_SURFACE)
    error_codes += "05 ";

  // Use heap also so that both of the virtual destructor entries in the vtable are used
  SafetyRules::Comms* base_class_ptr = new SafetyRules::Comms("comms");
  delete base_class_ptr;

  if (!error_codes.empty())
  {
    ROS_FATAL_STREAM("Test failed with error codes: " << error_codes);
    FAIL();
  }
}

int main(int argc, char** argv)
{
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Fatal))
    ros::console::notifyLoggerLevelsChanged();
  ros::init(argc, argv, "test_comms_rule");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
