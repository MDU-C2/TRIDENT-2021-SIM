/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

/**
 * @file
 * @brief C++ mission step definition.
 * @addtogroup mission
 * @{
 */

#pragma once

#include <cola2_lib/mission/mission_action.h>
#include <cola2_lib/mission/mission_maneuver.h>
#include <memory>
#include <vector>

/**
 * @brief A mission step implements a part of a mission that consist of a maneuver and actions to be done at the end of
 * the maneuver.
 */
class MissionStep
{
protected:
  std::shared_ptr<MissionManeuver> maneuver_;  //!< List of maneuvers.
  std::vector<MissionAction> actions_;         //!< List of actions.

public:
  /**
   * @brief Empty constructor.
   */
  MissionStep();
  ~MissionStep();

  /**
   * @brief Get the maneuver.
   *
   * @return Maneuver shared pointer.
   */
  std::shared_ptr<MissionManeuver> getManeuverPtr() const;

  /**
   * @brief Get the actions to do after the maneuver.
   *
   * @return Vector of actions to do.
   */
  std::vector<MissionAction> getActions() const;

  /**
   * @brief Set the maneuver of the mission step.
   *
   * @param maneuver Shared poitner of a maneuver.
   */
  void setManeuverPtr(std::shared_ptr<MissionManeuver> maneuver);

  /**
   * @brief Add an action to the actions list.
   *
   * @param action Mission action to add to the list.
   */
  void addAction(const MissionAction& action);
};

/** @} */
