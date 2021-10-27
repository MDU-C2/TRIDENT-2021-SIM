/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

/**
 * @file
 * @brief C++ mission definition.
 * @addtogroup mission
 * @{
 */

#pragma once

#include <cola2_lib/mission/mission_action.h>
#include <cola2_lib/mission/mission_goto.h>
#include <cola2_lib/mission/mission_maneuver.h>
#include <cola2_lib/mission/mission_park.h>
#include <cola2_lib/mission/mission_section.h>
#include <cola2_lib/mission/mission_step.h>
#include <tinyxml.h>
#include <memory>
#include <string>
#include <vector>

/**
 * @brief Definition of a mission that consist of a list of steps to execute during the mission.
 */
class Mission
{
protected:
  std::vector<std::shared_ptr<MissionStep> > mission_;  //!< List of mission steps.

public:
  /**
   * @brief Empty constructor.
   */
  Mission();
  ~Mission();

  /**
   * @brief Loads the mission from mission_file_name into the current mission. If there was a mission already, it is
   * erased.
   *
   * @param mission_file_name File name of the mission xml file.
   */
  void loadMission(const std::string& mission_file_name);

  /**
   * @brief Load a mission step from hDoc.
   *
   * @param hDoc TiXmlHandle pointing to a mission step on the xml document.
   * @param step_ptr Pointer to a MissionStep to save the loaded step to.
   */
  void loadStep(TiXmlHandle hDoc, std::shared_ptr<MissionStep> step_ptr);

  /**
   * @brief Load a goto maneuver from the hDoc.
   *
   * @param hDoc TiXmlHandle pointing to a maneuver on the xml document.
   * @param mission_goto_ptr Pointer to a MissionGoto to save the loaded step to.
   */
  void loadManeuverGoto(TiXmlHandle hDoc, std::shared_ptr<MissionGoto> mission_goto_ptr);

  /**
   * @brief Load a section maneuver  from the hDoc.
   *
   * @param hDoc TiXmlHandle pointing to a maneuver on the xml document.
   * @param mission_section_ptr Pointer to a MissionSection to save the loaded section to.
   */
  void loadManeuverSection(TiXmlHandle hDoc, std::shared_ptr<MissionSection> mission_section_ptr);

  /**
   * @brief Load a park maneuver  from the hDoc.
   *
   * @param hDoc TiXmlHandle pointing to a maneuver on the xml document.
   * @param mission_park_ptr Pointer to a MissionPark to save the loaded park to.
   */
  void loadManeuverPark(TiXmlHandle hDoc, std::shared_ptr<MissionPark> mission_park_ptr);

  /**
   * @brief Load a mission action from the hDoc.
   *
   * @param hDoc TiXmlHandle pointing to an action on the xml document.
   * @param action Pointer to a MissionAction to save the loaded action to.
   */
  void loadAction(TiXmlHandle hDoc, MissionAction* action);

  /**
   * @brief Write the current mission into the mission_file_name file.
   *
   * @param mission_file_name Mission file name to write to.
   */
  void writeMission(const std::string& mission_file_name);

  /**
   * @brief Add a mission step to the mission xml.
   *
   * @param mission Mission xml.
   * @param step Mission step to add.
   */
  void writeMissionStep(TiXmlElement* mission, const MissionStep& step);

  /**
   * @brief Add a mission goto to the mission xml.
   *
   * @param mission Mission xml.
   * @param g Mission goto to add.
   */
  void writeManeuverGoto(TiXmlElement* mission, const MissionGoto& g);

  /**
   * @brief Add a mission section to the mission xml.
   *
   * @param mission Mission xml.
   * @param sec Mission section to add.
   */
  void writeManeuverSection(TiXmlElement* mission, const MissionSection& sec);

  /**
   * @brief Add a mission park to the mission xml.
   *
   * @param mission Mission xml.
   * @param park Mission park to add.
   */
  void writeManeuverPark(TiXmlElement* mission, const MissionPark& park);

  /**
   * @brief Add a mission action to the mission xml.
   *
   * @param mission Mission xml.
   * @param action Mission action to add.
   */
  void writeAction(TiXmlElement* mission, const MissionAction& action);

  /**
   * @brief Append the mission step to the current mission.
   *
   * @param step Mission step to append.
   */
  void addStep(std::shared_ptr<MissionStep> step);

  /**
   * @brief Get the mission step at index i.
   * @param i Index of the mission step to get.
   * @return Misson step at index i.
   */
  std::shared_ptr<MissionStep> getStep(const std::size_t i);

  /**
   * @brief Number of mission steps in the current mission.
   * @return Number of mission steps in the current mission.
   */
  std::size_t size();
};

/** @} */
