/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

/**
 * @file
 * @brief C++ mission action definition.
 * @addtogroup mission
 * @{
 */

#pragma once

#include <string>
#include <vector>

/**
 * @brief A mission action defines a call to a service during a mission.
 */
class MissionAction
{
protected:
  std::string action_id_;                //!< Name of the service to call.
  std::vector<std::string> parameters_;  //!< Parameters of the service to call if is_trigger_ is false.
  bool is_trigger_;                      //!< Flag to store if the service is a trigger service.

public:
  /**
   * @brief Empty constructor.
   */
  MissionAction();

  /**
   * @brief Constructor taking the action_id and the parameters.
   *
   * @param action_id Name of the service to call.
   * @param parameters Parameters of the service to call. If it is empty, the service is set to trigger type.
   */
  MissionAction(const std::string& action_id, const std::vector<std::string>& parameters);
  ~MissionAction();

  /**
   * @brief Get the action id.
   *
   * @return Name of the service to call.
   */
  std::string getActionId() const;

  /**
   * @brief Get parameters of service to call.
   *
   * @return Get the parameters of the service to call.
   */
  std::vector<std::string> getParameters() const;

  /**
   * @brief Returns true or false depending on the type of action: true if it is a trigger action, false otherwise.
   *
   * @return True if it is a trigger action, false otherwise.
   */
  bool getIsTrigger() const;

  /**
   * @brief Set the action id.
   *
   * @param action_id Name of the service to call.
   */
  void setActionId(const std::string& action_id);

  /**
   * @brief Add a parameter to the parameter list. This function sets the action to non-trigger.
   *
   * @param parameter Parameter to add to parameter list.
   */
  void addParameters(const std::string& parameter);
};

/** @} */
