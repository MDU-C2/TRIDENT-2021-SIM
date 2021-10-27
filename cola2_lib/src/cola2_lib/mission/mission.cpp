/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#include <cola2_lib/mission/mission.h>
#include <algorithm>
#include <iostream>
#include <set>
#include <stdexcept>

Mission::Mission()
{
}

Mission::~Mission()
{
}

void Mission::loadMission(const std::string& mission_file_name)
{
  // Load XML document file
  TiXmlDocument doc(mission_file_name.c_str());
  if (!doc.LoadFile())
    throw std::runtime_error("Invalid/Not found document");

  // Create XML handle
  TiXmlHandle hDoc(&doc);

  // If previous mission, erase it
  mission_.clear();

  // Read all children of mission tag
  TiXmlElement* pElem = hDoc.FirstChild("mission").FirstChild().Element();
  if (pElem)
  {
    if ((pElem->Value() != std::string("version")) || (pElem->GetText() != std::string("2.0")))
      throw std::runtime_error("Invalid/Not found mission file version");
    pElem->NextSiblingElement();
  }
  for (; pElem; pElem = pElem->NextSiblingElement())
  {
    const std::string m_name = pElem->Value();
    if (m_name == "mission_step")
    {
      auto step = std::make_shared<MissionStep>();
      loadStep(TiXmlHandle(pElem), step);
      addStep(step);
    }
  }
}

void Mission::loadStep(TiXmlHandle hDoc, std::shared_ptr<MissionStep> step_ptr)
{
  // Search for maneuver
  TiXmlElement* pElem;
  pElem = hDoc.FirstChild("maneuver").Element();
  if (!pElem)
    throw std::runtime_error("No maneuver element in mission step");

  // Load maneuver according to attribute type
  const std::string attribute = pElem->Attribute("type");
  if (attribute == "goto")
  {
    auto waypoint = std::make_shared<MissionGoto>();
    loadManeuverGoto(TiXmlHandle(pElem), waypoint);
    step_ptr->setManeuverPtr(waypoint);
  }
  else if (attribute == "section")
  {
    auto section = std::make_shared<MissionSection>();
    loadManeuverSection(TiXmlHandle(pElem), section);
    step_ptr->setManeuverPtr(section);
  }
  else if (attribute == "park")
  {
    auto park = std::make_shared<MissionPark>();
    loadManeuverPark(TiXmlHandle(pElem), park);
    step_ptr->setManeuverPtr(park);
  }
  else
    throw std::runtime_error("Invalid maneuver type: " + attribute);

  // Load actions
  pElem = hDoc.FirstChild("actions_list").FirstChild().Element();
  for (; pElem; pElem = pElem->NextSiblingElement())
  {
    const std::string m_name = pElem->Value();
    if (m_name == "action")
    {
      MissionAction action;
      loadAction(TiXmlHandle(pElem), &action);
      step_ptr->addAction(action);
    }
  }
}

void Mission::loadManeuverGoto(TiXmlHandle hDoc, std::shared_ptr<MissionGoto> mission_goto_ptr)
{
  // Keep track of what has been found
  std::set<std::string> missing_tags = {"final_latitude", "final_longitude", "final_depth", "final_altitude",
                                        "heave_mode", "surge_velocity", "tolerance_xy", "no_altitude_goes_up"};

  // Load tags
  TiXmlElement* pElem = hDoc.FirstChild().Element();
  for (; pElem; pElem = pElem->NextSiblingElement())
  {
    try
    {
      const std::string tag(pElem->Value());
      if (tag == "final_latitude")
      {
        mission_goto_ptr->setFinalLatitude(std::stod(pElem->GetText()));
        missing_tags.erase(tag);
      }
      else if (tag == "final_longitude")
      {
        mission_goto_ptr->setFinalLongitude(std::stod(pElem->GetText()));
        missing_tags.erase(tag);
      }
      else if (tag == "final_depth")
      {
        mission_goto_ptr->setFinalDepth(std::stod(pElem->GetText()));
        missing_tags.erase(tag);
      }
      else if (tag == "final_altitude")
      {
        mission_goto_ptr->setFinalAltitude(std::stod(pElem->GetText()));
        missing_tags.erase(tag);
      }
      else if (tag == "heave_mode")
      {
        int heave_mode_int = std::stoi(pElem->GetText());
        if (heave_mode_int == 0)
          mission_goto_ptr->setHeaveMode(MissionGoto::HeaveMode::DEPTH);
        else if (heave_mode_int == 1)
          mission_goto_ptr->setHeaveMode(MissionGoto::HeaveMode::ALTITUDE);
        else if (heave_mode_int == 2)
          mission_goto_ptr->setHeaveMode(MissionGoto::HeaveMode::BOTH);
        if ((heave_mode_int == 0) || (heave_mode_int == 1) || (heave_mode_int == 2))
          missing_tags.erase(tag);
      }
      else if (tag == "surge_velocity")
      {
        mission_goto_ptr->setSurgeVelocity(std::stod(pElem->GetText()));
        missing_tags.erase(tag);
      }
      else if (tag == "tolerance_xy")
      {
        mission_goto_ptr->setToleranceXY(std::stod(pElem->GetText()));
        missing_tags.erase(tag);
      }
      else if (tag == "no_altitude_goes_up")
      {
        std::string data(pElem->GetText());
        std::transform(data.begin(), data.end(), data.begin(), ::tolower);
        mission_goto_ptr->setNoAltitudeGoesUp(data == "true");
        missing_tags.erase(tag);
      }
    }
    catch (...)
    {
    }
    if (missing_tags.empty())
      break;
  }

  // Check if something is missing
  if (!missing_tags.empty())
  {
    std::string msg("Goto without the following tags:");
    for (const auto& tag : missing_tags)
      msg += " " + tag;
    throw std::runtime_error(msg);
  }
}

void Mission::loadManeuverSection(TiXmlHandle hDoc, std::shared_ptr<MissionSection> mission_section_ptr)
{
  // Keep track of what has been found
  std::set<std::string> missing_tags = {"initial_latitude", "initial_longitude", "initial_depth", "final_latitude",
                                        "final_longitude", "final_depth", "final_altitude", "heave_mode",
                                        "surge_velocity", "tolerance_xy", "no_altitude_goes_up"};

  // Load tags
  TiXmlElement* pElem = hDoc.FirstChild().Element();
  for (; pElem; pElem = pElem->NextSiblingElement())
  {
    try
    {
      const std::string tag(pElem->Value());
      if (tag == "initial_latitude")
      {
        mission_section_ptr->setInitialLatitude(std::stod(pElem->GetText()));
        missing_tags.erase(tag);
      }
      else if (tag == "initial_longitude")
      {
        mission_section_ptr->setInitialLongitude(std::stod(pElem->GetText()));
        missing_tags.erase(tag);
      }
      else if (tag == "initial_depth")
      {
        mission_section_ptr->setInitialDepth(std::stod(pElem->GetText()));
        missing_tags.erase(tag);
      }
      else if (tag == "final_latitude")
      {
        mission_section_ptr->setFinalLatitude(std::stod(pElem->GetText()));
        missing_tags.erase(tag);
      }
      else if (tag == "final_longitude")
      {
        mission_section_ptr->setFinalLongitude(std::stod(pElem->GetText()));
        missing_tags.erase(tag);
      }
      else if (tag == "final_depth")
      {
        mission_section_ptr->setFinalDepth(std::stod(pElem->GetText()));
        missing_tags.erase(tag);
      }
      else if (tag == "final_altitude")
      {
        mission_section_ptr->setFinalAltitude(std::stod(pElem->GetText()));
        missing_tags.erase(tag);
      }
      else if (tag == "heave_mode")
      {
        int heave_mode_int = std::stoi(pElem->GetText());
        if (heave_mode_int == 0)
          mission_section_ptr->setHeaveMode(MissionSection::HeaveMode::DEPTH);
        else if (heave_mode_int == 1)
          mission_section_ptr->setHeaveMode(MissionSection::HeaveMode::ALTITUDE);
        else if (heave_mode_int == 2)
          mission_section_ptr->setHeaveMode(MissionSection::HeaveMode::BOTH);
        if ((heave_mode_int == 0) || (heave_mode_int == 1) || (heave_mode_int == 2))
          missing_tags.erase(tag);
      }
      else if (tag == "surge_velocity")
      {
        mission_section_ptr->setSurgeVelocity(std::stod(pElem->GetText()));
        missing_tags.erase(tag);
      }
      else if (tag == "tolerance_xy")
      {
        mission_section_ptr->setToleranceXY(std::stod(pElem->GetText()));
        missing_tags.erase(tag);
      }
      else if (tag == "no_altitude_goes_up")
      {
        std::string data(pElem->GetText());
        std::transform(data.begin(), data.end(), data.begin(), ::tolower);
        mission_section_ptr->setNoAltitudeGoesUp(data == "true");
        missing_tags.erase(tag);
      }
    }
    catch (...)
    {
    }
    if (missing_tags.empty())
      break;
  }

  // Check if something is missing
  if (!missing_tags.empty())
  {
    std::string msg("Section without the following tags:");
    for (const auto& tag : missing_tags)
      msg += " " + tag;
    throw std::runtime_error(msg);
  }
}

void Mission::loadManeuverPark(TiXmlHandle hDoc, std::shared_ptr<MissionPark> mission_park_ptr)
{
  // Keep track of what has been found
  std::set<std::string> missing_tags = {"final_latitude", "final_longitude", "final_depth", "final_altitude",
                                        "final_yaw", "use_yaw", "heave_mode", "surge_velocity", "time",
                                        "no_altitude_goes_up"};

  // Load tags
  TiXmlElement* pElem = hDoc.FirstChild().Element();
  for (; pElem; pElem = pElem->NextSiblingElement())
  {
    try
    {
      const std::string tag(pElem->Value());
      if (tag == "final_latitude")
      {
        mission_park_ptr->setFinalLatitude(std::stod(pElem->GetText()));
        missing_tags.erase(tag);
      }
      else if (tag == "final_longitude")
      {
        mission_park_ptr->setFinalLongitude(std::stod(pElem->GetText()));
        missing_tags.erase(tag);
      }
      else if (tag == "final_depth")
      {
        mission_park_ptr->setFinalDepth(std::stod(pElem->GetText()));
        missing_tags.erase(tag);
      }
      else if (tag == "final_altitude")
      {
        mission_park_ptr->setFinalAltitude(std::stod(pElem->GetText()));
        missing_tags.erase(tag);
      }
      else if (tag == "final_yaw")
      {
        mission_park_ptr->setFinalYaw(std::stod(pElem->GetText()));
        missing_tags.erase(tag);
      }
      else if (tag == "use_yaw")
      {
        std::string data(pElem->GetText());
        std::transform(data.begin(), data.end(), data.begin(), ::tolower);
        mission_park_ptr->setUseYaw(data == "true");
        missing_tags.erase(tag);
      }
      else if (tag == "heave_mode")
      {
        int heave_mode_int = std::stoi(pElem->GetText());
        if (heave_mode_int == 0)
          mission_park_ptr->setHeaveMode(MissionPark::HeaveMode::DEPTH);
        else if (heave_mode_int == 1)
          mission_park_ptr->setHeaveMode(MissionPark::HeaveMode::ALTITUDE);
        else if (heave_mode_int == 2)
          mission_park_ptr->setHeaveMode(MissionPark::HeaveMode::BOTH);
        if ((heave_mode_int == 0) || (heave_mode_int == 1) || (heave_mode_int == 2))
          missing_tags.erase(tag);
      }
      else if (tag == "surge_velocity")
      {
        mission_park_ptr->setSurgeVelocity(std::stod(pElem->GetText()));
        missing_tags.erase(tag);
      }
      else if (tag == "time")
      {
        mission_park_ptr->setTime(std::stod(pElem->GetText()));
        missing_tags.erase(tag);
      }
      else if (tag == "no_altitude_goes_up")
      {
        std::string data(pElem->GetText());
        std::transform(data.begin(), data.end(), data.begin(), ::tolower);
        mission_park_ptr->setNoAltitudeGoesUp(data == "true");
        missing_tags.erase(tag);
      }
    }
    catch (...)
    {
    }
    if (missing_tags.empty())
      break;
  }

  // Check if something is missing
  if (!missing_tags.empty())
  {
    std::string msg("Park without the following tags:");
    for (const auto& tag : missing_tags)
      msg += " " + tag;
    throw std::runtime_error(msg);
  }
}

void Mission::loadAction(TiXmlHandle hDoc, MissionAction* action)
{
  // Find action in action_list
  TiXmlElement* pElem = hDoc.FirstChild().Element();
  if (!pElem)
    throw std::runtime_error("No action in action_list");

  // Find action_id
  const std::string action_tag = pElem->Value();
  if (action_tag != "action_id")
    throw std::runtime_error("Expected action_id");
  action->setActionId(pElem->GetText());

  // Check params
  pElem = pElem->NextSiblingElement();
  if (!pElem || (std::string(pElem->Value()) != "parameters"))
    return;  // No parameters
  hDoc = TiXmlHandle(pElem);

  // TODO: Check that parameters are really stored!
  pElem = hDoc.FirstChild().Element();
  for (; pElem; pElem = pElem->NextSiblingElement())
  {
    const std::string param_tag = pElem->Value();
    const std::string param = pElem->GetText();
    if (param_tag == "param")
    {
      action->addParameters(param);
    }
  }
}

void Mission::writeMission(const std::string& mission_file_name)
{
  TiXmlDocument doc;
  TiXmlDeclaration* decl = new TiXmlDeclaration("1.0", "", "");
  doc.LinkEndChild(decl);
  TiXmlElement* mission = new TiXmlElement("mission");
  doc.LinkEndChild(mission);
  TiXmlElement* version = new TiXmlElement("version");
  version->LinkEndChild(new TiXmlText(std::to_string(2.0)));
  mission->LinkEndChild(version);
  for (const auto& step_ptr : mission_)
    writeMissionStep(mission, *step_ptr);
  doc.SaveFile(mission_file_name.c_str());
}

void Mission::writeMissionStep(TiXmlElement* mission, const MissionStep& step)
{
  TiXmlElement* mission_step = new TiXmlElement("mission_step");

  // Write mission step maneuver
  if (step.getManeuverPtr()->getManeuverType() == MissionManeuver::GOTO_MANEUVER)
  {
    const auto g = std::dynamic_pointer_cast<MissionGoto>(step.getManeuverPtr());
    writeManeuverGoto(mission_step, *g);
  }
  else if (step.getManeuverPtr()->getManeuverType() == MissionManeuver::SECTION_MANEUVER)
  {
    const auto sec = std::dynamic_pointer_cast<MissionSection>(step.getManeuverPtr());
    writeManeuverSection(mission_step, *sec);
  }
  else if (step.getManeuverPtr()->getManeuverType() == MissionManeuver::PARK_MANEUVER)
  {
    const auto park = std::dynamic_pointer_cast<MissionPark>(step.getManeuverPtr());
    writeManeuverPark(mission_step, *park);
  }

  // Write action_list if available
  const std::vector<MissionAction> actions = step.getActions();
  if (!actions.empty())
  {
    TiXmlElement* action_list = new TiXmlElement("actions_list");
    for (const auto& action : actions)
      writeAction(action_list, action);
    mission_step->LinkEndChild(action_list);
  }
  mission->LinkEndChild(mission_step);
}

void Mission::writeManeuverGoto(TiXmlElement* mission, const MissionGoto& g)
{
  TiXmlElement* element = new TiXmlElement("maneuver");
  element->SetAttribute("type", "goto");
  mission->LinkEndChild(element);

  TiXmlElement* final_latitude = new TiXmlElement("final_latitude");
  final_latitude->LinkEndChild(new TiXmlText(std::to_string(g.getFinalLatitude())));
  element->LinkEndChild(final_latitude);
  TiXmlElement* final_longitude = new TiXmlElement("final_longitude");
  final_longitude->LinkEndChild(new TiXmlText(std::to_string(g.getFinalLongitude())));
  element->LinkEndChild(final_longitude);
  TiXmlElement* final_depth = new TiXmlElement("final_depth");
  final_depth->LinkEndChild(new TiXmlText(std::to_string(g.getFinalDepth())));
  element->LinkEndChild(final_depth);
  TiXmlElement* final_altitude = new TiXmlElement("final_altitude");
  final_altitude->LinkEndChild(new TiXmlText(std::to_string(g.getFinalAltitude())));
  element->LinkEndChild(final_altitude);
  int heave_mode_int = 0;  // DEPTH
  if (g.getHeaveMode() == MissionGoto::HeaveMode::ALTITUDE)
    heave_mode_int = 1;
  else if (g.getHeaveMode() == MissionGoto::HeaveMode::BOTH)
    heave_mode_int = 2;
  TiXmlElement* heave_mode = new TiXmlElement("heave_mode");
  heave_mode->LinkEndChild(new TiXmlText(std::to_string(heave_mode_int)));
  element->LinkEndChild(heave_mode);
  TiXmlElement* surge_velocity = new TiXmlElement("surge_velocity");
  surge_velocity->LinkEndChild(new TiXmlText(std::to_string(g.getSurgeVelocity())));
  element->LinkEndChild(surge_velocity);
  TiXmlElement* tolerance_xy = new TiXmlElement("tolerance_xy");
  tolerance_xy->LinkEndChild(new TiXmlText(std::to_string(g.getToleranceXY())));
  element->LinkEndChild(tolerance_xy);
  TiXmlElement* no_altitude_goes_up = new TiXmlElement("no_altitude_goes_up");
  if (g.getNoAltitudeGoesUp())
    no_altitude_goes_up->LinkEndChild(new TiXmlText(std::string("true")));
  else
    no_altitude_goes_up->LinkEndChild(new TiXmlText(std::string("false")));
  element->LinkEndChild(no_altitude_goes_up);
}

void Mission::writeManeuverSection(TiXmlElement* mission, const MissionSection& sec)
{
  TiXmlElement* element = new TiXmlElement("maneuver");
  element->SetAttribute("type", "section");
  mission->LinkEndChild(element);

  TiXmlElement* initial_latitude = new TiXmlElement("initial_latitude");
  initial_latitude->LinkEndChild(new TiXmlText(std::to_string(sec.getInitialLatitude())));
  element->LinkEndChild(initial_latitude);
  TiXmlElement* initial_longitude = new TiXmlElement("initial_longitude");
  initial_longitude->LinkEndChild(new TiXmlText(std::to_string(sec.getInitialLongitude())));
  element->LinkEndChild(initial_longitude);
  TiXmlElement* initial_depth = new TiXmlElement("initial_depth");
  initial_depth->LinkEndChild(new TiXmlText(std::to_string(sec.getInitialDepth())));
  element->LinkEndChild(initial_depth);
  TiXmlElement* final_latitude = new TiXmlElement("final_latitude");
  final_latitude->LinkEndChild(new TiXmlText(std::to_string(sec.getFinalLatitude())));
  element->LinkEndChild(final_latitude);
  TiXmlElement* final_longitude = new TiXmlElement("final_longitude");
  final_longitude->LinkEndChild(new TiXmlText(std::to_string(sec.getFinalLongitude())));
  element->LinkEndChild(final_longitude);
  TiXmlElement* final_depth = new TiXmlElement("final_depth");
  final_depth->LinkEndChild(new TiXmlText(std::to_string(sec.getFinalDepth())));
  element->LinkEndChild(final_depth);
  TiXmlElement* final_altitude = new TiXmlElement("final_altitude");
  final_altitude->LinkEndChild(new TiXmlText(std::to_string(sec.getFinalAltitude())));
  element->LinkEndChild(final_altitude);
  int heave_mode_int = 0;  // DEPTH
  if (sec.getHeaveMode() == MissionSection::HeaveMode::ALTITUDE)
    heave_mode_int = 1;
  else if (sec.getHeaveMode() == MissionSection::HeaveMode::BOTH)
    heave_mode_int = 2;
  TiXmlElement* heave_mode = new TiXmlElement("heave_mode");
  heave_mode->LinkEndChild(new TiXmlText(std::to_string(heave_mode_int)));
  element->LinkEndChild(heave_mode);
  TiXmlElement* surge_velocity = new TiXmlElement("surge_velocity");
  surge_velocity->LinkEndChild(new TiXmlText(std::to_string(sec.getSurgeVelocity())));
  element->LinkEndChild(surge_velocity);
  TiXmlElement* tolerance_xy = new TiXmlElement("tolerance_xy");
  tolerance_xy->LinkEndChild(new TiXmlText(std::to_string(sec.getToleranceXY())));
  element->LinkEndChild(tolerance_xy);
  TiXmlElement* no_altitude_goes_up = new TiXmlElement("no_altitude_goes_up");
  if (sec.getNoAltitudeGoesUp())
    no_altitude_goes_up->LinkEndChild(new TiXmlText(std::string("true")));
  else
    no_altitude_goes_up->LinkEndChild(new TiXmlText(std::string("false")));
  element->LinkEndChild(no_altitude_goes_up);
}

void Mission::writeManeuverPark(TiXmlElement* mission, const MissionPark& park)
{
  TiXmlElement* element = new TiXmlElement("maneuver");
  element->SetAttribute("type", "park");
  mission->LinkEndChild(element);

  TiXmlElement* final_latitude = new TiXmlElement("final_latitude");
  final_latitude->LinkEndChild(new TiXmlText(std::to_string(park.getFinalLatitude())));
  element->LinkEndChild(final_latitude);
  TiXmlElement* final_longitude = new TiXmlElement("final_longitude");
  final_longitude->LinkEndChild(new TiXmlText(std::to_string(park.getFinalLongitude())));
  element->LinkEndChild(final_longitude);
  TiXmlElement* final_depth = new TiXmlElement("final_depth");
  final_depth->LinkEndChild(new TiXmlText(std::to_string(park.getFinalDepth())));
  element->LinkEndChild(final_depth);
  TiXmlElement* final_altitude = new TiXmlElement("final_altitude");
  final_altitude->LinkEndChild(new TiXmlText(std::to_string(park.getFinalAltitude())));
  element->LinkEndChild(final_altitude);
  TiXmlElement* final_yaw = new TiXmlElement("final_yaw");
  final_yaw->LinkEndChild(new TiXmlText(std::to_string(park.getFinalYaw())));
  element->LinkEndChild(final_yaw);
  TiXmlElement* use_yaw = new TiXmlElement("use_yaw");
  if (park.getUseYaw())
    use_yaw->LinkEndChild(new TiXmlText(std::string("true")));
  else
    use_yaw->LinkEndChild(new TiXmlText(std::string("false")));
  element->LinkEndChild(use_yaw);
  int heave_mode_int = 0;  // DEPTH
  if (park.getHeaveMode() == MissionPark::HeaveMode::ALTITUDE)
    heave_mode_int = 1;
  else if (park.getHeaveMode() == MissionPark::HeaveMode::BOTH)
    heave_mode_int = 2;
  TiXmlElement* heave_mode = new TiXmlElement("heave_mode");
  heave_mode->LinkEndChild(new TiXmlText(std::to_string(heave_mode_int)));
  element->LinkEndChild(heave_mode);
  TiXmlElement* surge_velocity = new TiXmlElement("surge_velocity");
  surge_velocity->LinkEndChild(new TiXmlText(std::to_string(park.getSurgeVelocity())));
  element->LinkEndChild(surge_velocity);
  TiXmlElement* t = new TiXmlElement("time");
  t->LinkEndChild(new TiXmlText(std::to_string(park.getTime())));
  element->LinkEndChild(t);
  TiXmlElement* no_altitude_goes_up = new TiXmlElement("no_altitude_goes_up");
  if (park.getNoAltitudeGoesUp())
    no_altitude_goes_up->LinkEndChild(new TiXmlText(std::string("true")));
  else
    no_altitude_goes_up->LinkEndChild(new TiXmlText(std::string("false")));
  element->LinkEndChild(no_altitude_goes_up);
}

void Mission::writeAction(TiXmlElement* mission, const MissionAction& action)
{
  TiXmlElement* element = new TiXmlElement("action");
  TiXmlElement* action_id = new TiXmlElement("action_id");
  action_id->LinkEndChild(new TiXmlText(action.getActionId()));
  element->LinkEndChild(action_id);
  if (!action.getParameters().empty())
  {
    TiXmlElement* parameters = new TiXmlElement("parameters");
    element->LinkEndChild(parameters);
    for (const auto& elem : action.getParameters())
    {
      TiXmlElement* param = new TiXmlElement("param");
      param->LinkEndChild(new TiXmlText(elem));
      parameters->LinkEndChild(param);
    }
  }
  mission->LinkEndChild(element);
}

void Mission::addStep(std::shared_ptr<MissionStep> step)
{
  mission_.push_back(step);
}

std::shared_ptr<MissionStep> Mission::getStep(const std::size_t i)
{
  return mission_.at(i);
}

std::size_t Mission::size()
{
  return mission_.size();
}
