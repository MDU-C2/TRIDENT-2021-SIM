# Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
#
# This file is subject to the terms and conditions defined in file
# 'LICENSE.txt', which is part of this source code package.

"""
Class for generating/reading an xml mission file.
A mission contains one or more mission steps.
A mission step contains one mission maneuver (waypoint, section or park) plus one or more mission actions.
"""

import logging
from lxml import etree as ET

GOTO_MANEUVER = 0  # Goto maneuver type.
SECTION_MANEUVER = 1  # Section meneuver type.
PARK_MANEUVER = 2  # Park maneuver type.

HEAVE_MODE_DEPTH = 0  # Control the vehile in depth.
HEAVE_MODE_ALTITUDE = 1  # Control the vehicle in altitude.
HEAVE_MODE_BOTH = 2  # Controls the vehicle in altitude if available. Otherwise, it controls the vehicle in depth.

logger = logging.getLogger(__name__)


class Parameter(object):
    """
    Parameters of a ManeuverAction.

    :param value: Value of the parameter, defaults to "".
    :type value: str, optional
    """

    def __init__(self, value=""):
        """Consructor."""
        self.value = value


class MissionManeuver(object):
    """
    Base class for a mission maneuver.

    :param m_type: Maneuver type.
    :type m_type: int
    """

    def __init__(self, m_type):
        """Consructor."""
        self.maneuver_type = m_type

    def __str__(self):
        logger.info("MissionManeuver To be overrode\n")

    def get_maneuver_type(self):
        """
        Get the maneuver type.

        | GOTO_MANEUVER = 0
        | SECTION_MANEUVER = 1
        | PARK_MANEUVER = 2

        :return: Maneuver type.
        :rtype: int
        """
        return self.maneuver_type


class MissionGoto(MissionManeuver):
    """
    Implements a mission maneuver to go to a destination point.

    :param final_latitude: Final latitude in decimal degrees, defaults to 0.0.
    :type final_latitude: float, optional
    :param final_longitude: Final longitude in decimal degrees, defaults to 0.0.
    :type final_longitude: float, optional
    :param final_depth: Final depth in meters, defaults to 0.0.
    :type final_depth: float, optional
    :param final_altitude: Final altitude in meters, defaults to 0.0.
    :type final_altitude: float, optional
    :param heave_mode: Heave mode to define depth/altitude behaviour, defaults to HEAVE_MODE_DEPTH.
    :type heave_mode: int, optional
    :param surge_velocity: Velocity in surge in m/s, defaults to 0.0.
    :type surge_velocity: float, optional
    :param tolerance_xy: Final position tolerance in meters, defaults to 0.0.
    :type tolerance_xy: float, optional
    :param no_altitude_goes_up: Behaviour when no altitude is avalable, defaults to True.
    :type no_altitude_goes_up: bool, optional
    """

    def __init__(self, final_latitude=0.0, final_longitude=0.0, final_depth=0.0, final_altitude=0.0,
                       heave_mode=HEAVE_MODE_DEPTH, surge_velocity=0.0, tolerance_xy=0.0, no_altitude_goes_up=True):
        """
        Constructor.
        """
        super(MissionGoto, self).__init__(GOTO_MANEUVER)
        self.final_latitude = final_latitude
        self.final_longitude = final_longitude
        self.final_depth = final_depth
        self.final_altitude = final_altitude
        self.heave_mode = heave_mode
        self.surge_velocity = surge_velocity
        self.tolerance_xy = tolerance_xy
        self.no_altitude_goes_up = no_altitude_goes_up

    def __str__(self):
        return ("Final latitude = " + str(self.final_latitude) + "\n" +
                "Final longitude = " + str(self.final_longitude) + "\n" +
                "Final depth = " + str(self.final_depth) + "\n" +
                "Final altitude = " + str(self.final_altitude) + "\n" +
                "Heave mode = " + str(self.heave_mode) + "\n" +
                "Surge velocity = " + str(self.surge_velocity) + "\n" +
                "Tolerance XY = " + str(self.tolerance_xy) + "\n" +
                "No altitude goes up = " + str(self.no_altitude_goes_up))


class MissionSection(MissionManeuver):
    """
    Implements a mission maneuver to go from a startin ponit to an end point.

    :param initial_latitude: Initial latitude in decimal degrees, defaults to 0.0.
    :type initial_latitude: float, optional
    :param initial_longitude: Initial longitude in decimal degrees, defaults to 0.0.
    :type initial_longitude: float, optional
    :param initial_depth: Initial depth in meters, defaults to 0.0.
    :type initial_depth: float, optional
    :param final_latitude: Final latitude in decimal degrees, defaults to 0.0.
    :type final_latitude: float, optional
    :param final_longitude: Final longitude in decimal degrees, defaults to 0.0.
    :type final_longitude: float, optional
    :param final_depth: Final depth in meters, defaults to 0.0.
    :type final_depth: float, optional
    :param final_altitude: Final altitude in meters, defaults to 0.0.
    :type final_altitude: float, optional
    :param heave_mode: Heave mode to define depth/altitude behaviour, defaults to HEAVE_MODE_DEPTH.
    :type heave_mode: int, optional
    :param surge_velocity: Velocity in surge in m/s, defaults to 0.0.
    :type surge_velocity: float, optional
    :param tolerance_xy: Final position tolerance in meters, defaults to 0.0.
    :type tolerance_xy: float, optional
    :param no_altitude_goes_up: Behaviour when no altitude is avalable, defaults to True.
    :type no_altitude_goes_up: bool, optional
    """

    def __init__(self, initial_latitude=0.0, initial_longitude=0.0, initial_depth=0.0,
                       final_latitude=0.0, final_longitude=0.0, final_depth=0.0, final_altitude=0.0,
                       heave_mode=HEAVE_MODE_DEPTH, surge_velocity=0.0, tolerance_xy=0.0, no_altitude_goes_up=True):
        """
        Constructor.
        """
        super(MissionSection, self).__init__(SECTION_MANEUVER)
        self.initial_latitude = initial_latitude
        self.initial_longitude = initial_longitude
        self.initial_depth = initial_depth
        self.final_latitude = final_latitude
        self.final_longitude = final_longitude
        self.final_depth = final_depth
        self.final_altitude = final_altitude
        self.heave_mode = heave_mode
        self.surge_velocity = surge_velocity
        self.tolerance_xy = tolerance_xy
        self.no_altitude_goes_up = no_altitude_goes_up

    def __str__(self):
        return ("Initial latitude = " + str(self.initial_latitude) + "\n" +
                "Initial longitude = " + str(self.initial_longitude) + "\n" +
                "Initial depth = " + str(self.initial_depth) + "\n" +
                "Final latitude = " + str(self.final_latitude) + "\n" +
                "Final longitude = " + str(self.final_longitude) + "\n" +
                "Final depth = " + str(self.final_depth) + "\n" +
                "Final altitude = " + str(self.final_altitude) + "\n" +
                "Heave mode = " + str(self.heave_mode) + "\n" +
                "Surge velocity = " + str(self.surge_velocity) + "\n" +
                "Tolerance XY = " + str(self.tolerance_xy) + "\n" +
                "No altitude goes up = " + str(self.no_altitude_goes_up))


class MissionPark(MissionManeuver):
    """
    Implements a mission maneuver to stay at a specific point during some time.

    :param final_latitude: Final latitude in decimal degrees, defaults to 0.0.
    :type final_latitude: float, optional
    :param final_longitude: Final longitude in decimal degrees, defaults to 0.0.
    :type final_longitude: float, optional
    :param final_depth: Final depth in meters, defaults to 0.0.
    :type final_depth: float, optional
    :param final_altitude: Final altitude in meters, defaults to 0.0.
    :type final_altitude: float, optional
    :param final_yaw: Final orietation w.r.t. north in radians, defaults to 0.0.
    :type final_yaw: float, optional
    :param use_yaw: Use yaw during park, defaults to False.
    :type use_yaw: bool, optional
    :param heave_mode: Heave mode to define depth/altitude behaviour, defaults to HEAVE_MODE_DEPTH.
    :type heave_mode: int, optional
    :param surge_velocity: Velocity in surge in m/s, defaults to 0.0.
    :type surge_velocity: float, optional
    :param time: Park duration in seconds, defaults to 0.0.
    :type time: float, optional
    :param no_altitude_goes_up: Behaviour when no altitude is avalable, defaults to True.
    :type no_altitude_goes_up: bool, optional
    """

    def __init__(self, final_latitude=0.0, final_longitude=0.0, final_depth=0.0, final_altitude=0.0,
                       final_yaw=0.0, use_yaw=False,
                       heave_mode=HEAVE_MODE_DEPTH, surge_velocity=0.0, time=0.0, no_altitude_goes_up=True):
        """
        Constructor.
        """
        super(MissionPark, self).__init__(PARK_MANEUVER)
        self.final_latitude = final_latitude
        self.final_longitude = final_longitude
        self.final_depth = final_depth
        self.final_altitude = final_altitude
        self.final_yaw = final_yaw
        self.use_yaw = use_yaw
        self.heave_mode = heave_mode
        self.surge_velocity = surge_velocity
        self.time = time
        self.no_altitude_goes_up = no_altitude_goes_up
        self.tolerance_xy = 2 # ignored

    def __str__(self):
        return ("Final latitude = " + str(self.final_latitude) + "\n" +
                "Final longitude = " + str(self.final_longitude) + "\n" +
                "Final depth = " + str(self.final_depth) + "\n" +
                "Final altitude = " + str(self.final_altitude) + "\n" +
                "Final yaw = " + str(self.final_yaw) + "\n" +
                "Use yaw = " + str(self.use_yaw) + "\n" +
                "Heave mode = " + str(self.heave_mode) + "\n" +
                "Surge velocity = " + str(self.surge_velocity) + "\n" +
                "Time = " + str(self.time) + "\n" +
                "No altitude goes up = " + str(self.no_altitude_goes_up))


class MissionAction(object):
    """
    A mission action defines a call to a service during a mission.

    :param action_id: Name of the service to call, defaults to "".
    :type action_id: str, optional
    :param parameters: Parameters of the service to call, defaults to None.
    :type parameters: list[str], optional
    """
    def __init__(self, action_id="", parameters=None):
        """
        Constructor.
        """
        if parameters is None:
            parameters = list()
        self.action_id = action_id
        self.parameters = parameters

    def __str__(self):
        ret = "Action " + self.action_id + "\n"
        for i in self.parameters:
            ret = ret + "\t" + i.value + "\n"
        return ret

    def set(self, action_id, parameters):
        """
        Set the mission action values.

        :param action_id: Action service id.
        :type action_id: str
        :param parameters: List of parameters.
        :type parameters: list[str]
        """
        self.action_id = action_id
        self.parameters = parameters

    def get_action_id(self):
        """
        Get Action id value.

        :return: Return action id.
        :rtype: str
        """
        return self.action_id

    def get_parameters(self):
        """
        Get list of parameters.

        :return: list of parameters.
        :rtype: list[str]
        """
        return self.parameters


class Mission(object):
    """
    Definition of a mission that consist of a list of steps to execute during the mission.
    """
    def __init__(self):
        """Consructor."""
        self.mission = list()
        self.num_steps = 0

    def __str__(self):
        ret = "MISSION: \n"
        for s in self.mission:
            ret = ret + str(s) + "\n\n"
        return ret

    def get_length(self):
        """
        Get the mission length.

        :return: Number of steps in mission
        :rtype: int
        """
        return self.num_steps

    def copy(self, mission):
        """
        Copy Mission mission to current mission.

        :param mission: Mission to copy.
        :type mission: Mission
        """
        self.mission = list(mission.mission)
        self.num_steps = mission.num_steps

    def add_step(self, step):
        """
        Add step to current mission.

        :param step: Step is a mission maneuver (MissionGoto, MissionSection or MissionPark).
        :type step: MissionManeuver
        """
        self.num_steps += 1
        self.mission.append(step)

    def get_step(self, step_id):
        """
        Get a mission step (maneuver) of current mission.

        :param step_id: Index of the mission step to get.
        :type step_id: int

        :return: Mission step at index step_id.
        :rtype: MissionManeuver
        """
        if 0 <= step_id < len(self.mission):
            return self.mission[step_id]
        else:
            return None

    def insert_step(self, step_id, step):
        """
        Insert a mission step in current mission.

        :param step_id: Id of step.
        :type step_id: id
        :param step: Step maneuver to insert.
        :type step: MissionManeuver
        """
        self.mission.insert(step_id, step)
        self.num_steps += 1

    def update_step(self, step_id, step):
        """
        Update a mission step in current mission.

        :param step_id: Id of step.
        :type step_id: id
        :param step: New data to update.
        :type step: MissionManeuver
        """
        self.remove_step(step_id)
        self.insert_step(step_id, step)

    def remove_step(self, step_id):
        """
        Remove a step of the mission.

        :param step_id: Id of step
        :type step_id: int
        """
        del self.mission[step_id]
        self.num_steps -= 1

    def size(self):
        """
        Number of mission steps in the current mission.

        :return: Size of the mission.
        :rtype: int
        """
        return len(self.mission)

    def load_mission(self, mission_file_name):
        """
        Load a mission from XML.

        :param mission_file_name: Name of the mission file.
        :type mission_file_name: str
        """
        tree = ET.parse(mission_file_name)
        root = tree.getroot()
        try:
            if not "2.0" in root.find('version').text:
                logger.error("Incorrect mission file version")
                return False
        except:
            logger.error("Mission file version not found")
            return False
        for mStep in root:
            if not "mission_step" in mStep.tag:
                continue
            mission_step = MissionStep()
            for child in mStep:
                if child.tag == 'actions_list':
                    for action in child:
                        mission_step.load_action(action)
                elif child.tag == 'maneuver':
                    if child.get('type') == 'goto':
                        mission_step.load_goto_maneuver(child)
                    if child.get('type') == 'section':
                        mission_step.load_section_maneuver(child)
                    if child.get('type') == 'park':
                        mission_step.load_park_maneuver(child)
            self.add_step(mission_step)
        return True

    def write_mission(self, mission_file_name):
        """
        Write a mission in a XML file.
        
        :param mission_file_name: Name of the mission file.
        :type mission_file_name: str
        """
        xml_mission = ET.Element('mission')
        xml_version = ET.SubElement(xml_mission, 'version')
        xml_version.text = str(2.0)
        for mission_step in self.mission:
            xml_mission_step = ET.SubElement(xml_mission, 'mission_step')

            if mission_step.maneuver.maneuver_type == GOTO_MANEUVER:
                self.write_goto_maneuver(xml_mission_step, mission_step.maneuver)
            elif mission_step.maneuver.maneuver_type == SECTION_MANEUVER:
                self.write_section_maneuver(xml_mission_step, mission_step.maneuver)
            elif mission_step.maneuver.maneuver_type == PARK_MANEUVER:
                self.write_park_maneuver(xml_mission_step, mission_step.maneuver)

            if mission_step.actions:
                xml_actions = ET.SubElement(xml_mission_step, 'actions_list')
                for action in mission_step.actions:
                    self.write_action(xml_actions, action)

        tree = ET.ElementTree(xml_mission)
        tree.write(mission_file_name, pretty_print=True)

    def write_action(self, root, step):
        """
        Write Action in a XML.

        :param root: Parent tag.
        :type root: str
        :param step: Action step.
        :type step: MissionAction
        """
        xml_action = ET.SubElement(root, 'action')
        xml_id = ET.SubElement(xml_action, 'action_id')
        xml_id.text = step.action_id
        if len(step.parameters) != 0:
            xml_parameters = ET.SubElement(xml_action, 'parameters')
            for param in step.parameters:
                xml_param = ET.SubElement(xml_parameters, 'param')
                xml_param.text = param.value

    def write_goto_maneuver(self, root, step):
        """
        Write a goto maneuver in a XML.

        :param root: Parent tag.
        :type root: str
        :param step: Mission goto step.
        :type step: MissionGoto
        """
        xml_maneuver = ET.SubElement(root, 'maneuver', {'type': 'goto'})
        xml_final_latitude = ET.SubElement(xml_maneuver, 'final_latitude')
        xml_final_latitude.text = str(step.final_latitude)
        xml_final_longitude = ET.SubElement(xml_maneuver, 'final_longitude')
        xml_final_longitude.text = str(step.final_longitude)
        xml_final_depth = ET.SubElement(xml_maneuver, 'final_depth')
        xml_final_depth.text = str(step.final_depth)
        xml_final_altitude = ET.SubElement(xml_maneuver, 'final_altitude')
        xml_final_altitude.text = str(step.final_altitude)
        xml_heave_mode = ET.SubElement(xml_maneuver, 'heave_mode')
        xml_heave_mode.text = str(step.heave_mode)
        xml_surge_velocity = ET.SubElement(xml_maneuver, 'surge_velocity')
        xml_surge_velocity.text = str(step.surge_velocity)
        xml_tolerance_xy = ET.SubElement(xml_maneuver, 'tolerance_xy')
        xml_tolerance_xy.text = str(step.tolerance_xy)
        xml_no_altitude_goes_up = ET.SubElement(xml_maneuver, 'no_altitude_goes_up')
        xml_no_altitude_goes_up.text = str(step.no_altitude_goes_up).lower()

    def write_section_maneuver(self, root, step):
        """
        Write a section maneuver in a XML.

        :param root: Parent tag.
        :type root: str
        :param step: Mission section step.
        :type step: MissionSection
        """
        xml_maneuver = ET.SubElement(root, 'maneuver', {'type': 'section'})
        xml_initial_latitude = ET.SubElement(xml_maneuver, 'initial_latitude')
        xml_initial_latitude.text = str(step.initial_latitude)
        xml_initial_longitude = ET.SubElement(xml_maneuver, 'initial_longitude')
        xml_initial_longitude.text = str(step.initial_longitude)
        xml_initial_depth = ET.SubElement(xml_maneuver, 'initial_depth')
        xml_initial_depth.text = str(step.initial_depth)
        xml_final_latitude = ET.SubElement(xml_maneuver, 'final_latitude')
        xml_final_latitude.text = str(step.final_latitude)
        xml_final_longitude = ET.SubElement(xml_maneuver, 'final_longitude')
        xml_final_longitude.text = str(step.final_longitude)
        xml_final_depth = ET.SubElement(xml_maneuver, 'final_depth')
        xml_final_depth.text = str(step.final_depth)
        xml_final_altitude = ET.SubElement(xml_maneuver, 'final_altitude')
        xml_final_altitude.text = str(step.final_altitude)
        xml_heave_mode = ET.SubElement(xml_maneuver, 'heave_mode')
        xml_heave_mode.text = str(step.heave_mode)
        xml_surge_velocity = ET.SubElement(xml_maneuver, 'surge_velocity')
        xml_surge_velocity.text = str(step.surge_velocity)
        xml_tolerance_xy = ET.SubElement(xml_maneuver, 'tolerance_xy')
        xml_tolerance_xy.text = str(step.tolerance_xy)
        xml_no_altitude_goes_up = ET.SubElement(xml_maneuver, 'no_altitude_goes_up')
        xml_no_altitude_goes_up.text = str(step.no_altitude_goes_up).lower()

    def write_park_maneuver(self, root, step):
        """
        Write a park maneuver in a XML.

        :param root: Parent tag.
        :type root: str
        :param step: Mission park step.
        :type step: MissionPark
        """
        xml_maneuver = ET.SubElement(root, 'maneuver', {'type': 'park'})
        xml_final_latitude = ET.SubElement(xml_maneuver, 'final_latitude')
        xml_final_latitude.text = str(step.final_latitude)
        xml_final_longitude = ET.SubElement(xml_maneuver, 'final_longitude')
        xml_final_longitude.text = str(step.final_longitude)
        xml_final_depth = ET.SubElement(xml_maneuver, 'final_depth')
        xml_final_depth.text = str(step.final_depth)
        xml_final_altitude = ET.SubElement(xml_maneuver, 'final_altitude')
        xml_final_altitude.text = str(step.final_altitude)
        xml_final_yaw = ET.SubElement(xml_maneuver, 'final_yaw')
        xml_final_yaw.text = str(step.final_yaw)
        xml_use_yaw = ET.SubElement(xml_maneuver, 'use_yaw')
        xml_use_yaw.text = str(step.use_yaw).lower()
        xml_heave_mode = ET.SubElement(xml_maneuver, 'heave_mode')
        xml_heave_mode.text = str(step.heave_mode)
        xml_surge_velocity = ET.SubElement(xml_maneuver, 'surge_velocity')
        xml_surge_velocity.text = str(step.surge_velocity)
        xml_time = ET.SubElement(xml_maneuver, 'time')
        xml_time.text = str(step.time)
        xml_no_altitude_goes_up = ET.SubElement(xml_maneuver, 'no_altitude_goes_up')
        xml_no_altitude_goes_up.text = str(step.no_altitude_goes_up).lower()


class MissionStep(object):
    """
    A mission step implements a part of a mission that consist of a maneuver and actions to be done at the end of the maneuver.
    """
    def __init__(self):
        """Consructor."""
        self.maneuver = None
        self.actions = list()

    def __str__(self):
        ret = "Mission step\n"
        ret = ret + "\t" + str(self.maneuver) + "\n"
        for a in self.actions:
            ret = ret + "\t" + a.action_id + "\n"
        return ret

    def add_action(self, action):
        """
        Add action to mission step.

        :param action: Action.
        :type action: MissionAction
        """
        self.actions.append(action)

    def remove_action(self, id_action):
        """
        Remove action to mission step.

        :param id_action:  Action id.
        :type id_action: int
        """
        if id_action >= 0:
            del self.actions[id_action]

    def add_maneuver(self, maneuver):
        """
        Set maneuver to current step.

        :param maneuver: Mission maneuver.
        :type maneuver: MissionManeuver
        """
        self.maneuver = maneuver

    def get_maneuver(self):
        """
        Get the maneuver.

        :return: Current maneuver.
        :rtype: MissionManeuver
        """
        return self.maneuver

    def get_actions(self):
        """
        Get the actions to do after the maneuver.

        :return: List of actions.
        :rtype: list[MissionAction]
        """
        return self.actions

    def load_action(self, action_element):
        """
        Load an action element from XML.

        :param action_element: Action xml element.
        :type action_element: str
        """
        action_id = action_element.find('action_id').text
        parameters = action_element.find('parameters')
        params = list()
        if parameters is not None:
            for param in parameters:
                value = param.text
                p = Parameter(value)
                params.append(p)
        action = MissionAction(action_id, params)
        self.add_action(action)

    def load_goto_maneuver(self, goto_maneuver):
        """
        Load a goto maneuver from XML element.

        :param goto_maneuver: Goto maneuver xml element.
        :type goto_maneuver: str
        """
        goto = MissionGoto()
        goto.final_latitude = float(goto_maneuver.find('final_latitude').text)
        goto.final_longitude = float(goto_maneuver.find('final_longitude').text)
        goto.final_depth = float(goto_maneuver.find('final_depth').text)
        goto.final_altitude = float(goto_maneuver.find('final_altitude').text)
        goto.heave_mode = int(goto_maneuver.find('heave_mode').text)
        goto.surge_velocity = float(goto_maneuver.find('surge_velocity').text)
        goto.tolerance_xy = float(goto_maneuver.find('tolerance_xy').text)
        goto.no_altitude_goes_up = "true" in goto_maneuver.find('no_altitude_goes_up').text
        self.add_maneuver(goto)

    def load_section_maneuver(self, section_maneuver):
        """
        Load a section maneuver from XML element.

        :param section_maneuver: Section maneuver xml element.
        :type section_maneuver: str
        """
        section = MissionSection()
        section.initial_latitude = float(section_maneuver.find('initial_latitude').text)
        section.initial_longitude = float(section_maneuver.find('initial_longitude').text)
        section.initial_depth = float(section_maneuver.find('initial_depth').text)
        section.final_latitude = float(section_maneuver.find('final_latitude').text)
        section.final_longitude = float(section_maneuver.find('final_longitude').text)
        section.final_depth = float(section_maneuver.find('final_depth').text)
        section.final_altitude = float(section_maneuver.find('final_altitude').text)
        section.heave_mode = int(section_maneuver.find('heave_mode').text)
        section.surge_velocity = float(section_maneuver.find('surge_velocity').text)
        section.tolerance_xy = float(section_maneuver.find('tolerance_xy').text)
        section.no_altitude_goes_up = "true" in section_maneuver.find('no_altitude_goes_up').text
        self.add_maneuver(section)

    def load_park_maneuver(self, park_maneuver):
        """
        Load a park maneuver from XML element.

        :param park_maneuver: Park maneuver xml element.
        :type park_maneuver: str
        """
        park = MissionPark()
        park.final_latitude = float(park_maneuver.find('final_latitude').text)
        park.final_longitude = float(park_maneuver.find('final_longitude').text)
        park.final_depth = float(park_maneuver.find('final_depth').text)
        park.final_altitude = float(park_maneuver.find('final_altitude').text)
        park.final_yaw = float(park_maneuver.find('final_yaw').text)
        park.use_yaw = "true" in park_maneuver.find('use_yaw').text
        park.heave_mode = int(park_maneuver.find('heave_mode').text)
        park.surge_velocity = float(park_maneuver.find('surge_velocity').text)
        park.time = float(park_maneuver.find('time').text)
        park.no_altitude_goes_up = "true" in park_maneuver.find('no_altitude_goes_up').text
        self.add_maneuver(park)


def test_write_xml():
    """
    Test to write a xml file.
    """
    mission = Mission()

    mission_step = MissionStep()
    parameters = list()
    param = Parameter("abcd")
    param_2 = Parameter("2")
    parameters.append(param)
    parameters.append(param_2)
    action = MissionAction("action1", parameters)
    mission_step.add_action(action)
    goto = MissionGoto()
    goto.final_latitude = 41.777
    goto.final_longitude = 3.030
    goto.final_depth = 15.0
    goto.surge_velocity = 0.5
    goto.tolerance_xy = 2.0
    mission_step.add_maneuver(goto)
    mission.add_step(mission_step)
    mission_step2 = MissionStep()
    sec = MissionSection()
    sec.initial_latitude = 41.777
    sec.initial_longitude = 3.030
    sec.initial_depth = 15.0
    sec.final_latitude = 41.787
    sec.final_longitude = 3.034
    sec.final_depth = 15.0
    sec.final_altitude = 12.0
    sec.heave_mode = HEAVE_MODE_BOTH
    sec.surge_velocity = 0.5
    sec.tolerance_xy = 2.0
    mission_step2.add_maneuver(sec)
    mission.add_step(mission_step2)
    mission_step3 = MissionStep()
    park = MissionPark()
    park.final_latitude = 41.777
    park.final_longitude = 3.030
    park.final_depth = 15.0
    park.final_altitude = 12.0
    park.final_yaw = 15.0
    park.use_yaw = True
    park.heave_mode = HEAVE_MODE_ALTITUDE
    park.surge_velocity = 0.5
    park.time = 100.0
    mission_step3.add_maneuver(park)
    mission.add_step(mission_step3)
    mission.write_mission('temp.xml')


def test_load_xml():
    """
    Test to load and write xml file.
    """
    mission = Mission()

    # load
    mission.load_mission('temp.xml')

    # write
    mission.write_mission('temp2.xml')


if __name__ == "__main__":
    test_write_xml()
    test_load_xml()

