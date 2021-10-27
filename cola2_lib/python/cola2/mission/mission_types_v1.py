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

WAYPOINT_MANEUVER = 0
SECTION_MANEUVER = 1
PARK_MANEUVER = 2
MISSION_MANEUVER = 0
MISSION_CONFIGURATION = 1
MISSION_ACTION = 2

logger = logging.getLogger(__name__)


class Parameter(object):
    def __init__(self, value=""):
        self.value = value


class MissionManeuver(object):
    def __init__(self, m_type):
        self.maneuver_type = m_type

    def __str__(self):
        logger.info("MissionManeuver To be overrided\n")

    def get_maneuver_type(self):
        """
        WAYPOINT_MANEUVER = 0
        SECTION_MANEUVER = 1
        PARK_MANEUVER = 2
        :return: return maneuver type
        :rtype: int
        """
        return self.maneuver_type


class MissionPosition(object):
    def __init__(self, lat=0.0, lon=0.0, z=0.0, mode=False):
        self.latitude = lat
        self.longitude = lon
        self.z = z
        self.altitude_mode = mode

    def __str__(self):
        ret = "[" + str(self.latitude) + ", " + str(self.longitude) + ", " + str(self.z)
        if self.altitude_mode:
            ret = ret + " (altitude)]"
        else:
            ret = ret + " (depth)]"
        return ret

    def copy(self, position):
        """
        Copy the MissionPosition as position to current
        :param position: the MissionPosition to copy
        :type position: MissionPosition
        """
        self.latitude = position.latitude
        self.longitude = position.longitude
        self.z = position.z
        self.altitude_mode = position.altitude_mode

    def set(self, latitude, longitude, z, altitude_mode):
        """
        Set values
        :param latitude: latitude
        :type latitude: float
        :param longitude: longitude
        :type longitude: float
        :param z: z
        :type z: float
        :param altitude_mode: True if altitude mode is enabled, otherwise False
        :type: bool
        """
        self.latitude = latitude
        self.longitude = longitude
        self.z = z
        self.altitude_mode = altitude_mode

    def set_lat_lon(self, latitude, longitude):
        """
        Set latitude and longitude
        :param latitude: latitude
        :type latitude: float
        :param longitude: longitude
        :type longitude: float
        """
        self.latitude = latitude
        self.longitude = longitude

    def get_latitude(self):
        """ Get latitude"""
        return self.latitude

    def get_longitude(self):
        """ Get longitude"""
        return self.longitude

    def get_z(self):
        """ Get z"""
        return self.z

    def get_altitude_mode(self):
        """ Get altitude mode state"""
        return self.altitude_mode


class MissionTolerance(object):
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x = x
        self.y = y
        self.z = z

    def __str__(self):
        return "[" + str(self.x) + ", " + str(self.y) + ", " + str(self.z) + "]"

    def copy(self, tolerance):
        """
        Copy the MissionTolerance as tolerance to current
        :param tolerance: the MissionTolerance to copy
        :type tolerance: MissionTolerance
        """
        self.x = tolerance.x
        self.y = tolerance.y
        self.z = tolerance.z

    def set(self, x, y, z):
        """
        Set values
        :param x: x
        :type x: float
        :param y: y
        :type y: float
        :param z: z
        :type z: float
        """
        self.x = x
        self.y = y
        self.z = z


class MissionWaypoint(MissionManeuver):
    def __init__(self, position=None, speed=0.0, tolerance=None):
        super(MissionWaypoint, self).__init__(WAYPOINT_MANEUVER)
        self.position = position
        self.speed = speed
        self.tolerance = tolerance

    def __str__(self):
        return "Waypoint -> " + str(self.position) + " at " + str(self.speed) + "m/s with tolerance " + str(
            self.tolerance)

    def set(self, position, speed, tolerance):
        """
        Set values
        :param position: mission position
        :type position: MissionPosition
        :param speed: speed
        :type speed: float
        :param tolerance: mission tolerance
        :type tolerance: MissionTolerance
        """
        self.position = position
        self.speed = speed
        self.tolerance = tolerance

    def set_position(self, position):
        """
        Set position
        :param position: mission position
        :type position: MissionPosition
        """
        self.position = position

    def get_position(self):
        """
        Get the mission position
        :return: return the current position
        :rtype: MissionPosition
        """
        return self.position

    def get_speed(self):
        """
        Get the speed
        :return: speed
        :rtype: float
        """
        return self.speed

    def get_tolerance(self):
        """
        Get the mission tolerance
        :return: return the mission tolerance
        :rtype: MissionTolerance
        """
        return self.tolerance


class MissionSection(MissionManeuver):
    def __init__(self, initial_position=None, final_position=None, speed=0.0, tolerance=None):
        super(MissionSection, self).__init__(SECTION_MANEUVER)
        self.initial_position = initial_position
        self.final_position = final_position
        self.speed = speed
        self.tolerance = tolerance

    def __str__(self):
        return "Section -> " + str(self.initial_position) + " to " + str(self.final_position)

    def set(self, initial_position, final_position, speed, tolerance):
        """
        Set values
        :param initial_position: mission position
        :type initial_position: MissionPosition
        :param final_position: mission position
        :type final_position: MissionPosition
        :param speed: speed
        :type speed: float
        :param tolerance: mission tolerance
        :type tolerance: MissionTolerance
        """
        self.initial_position = initial_position
        self.final_position = final_position
        self.speed = speed
        self.tolerance = tolerance

    def get_final_position(self):
        """
        Get the final mission position
        :return: return the current final position
        :rtype: MissionPosition
        """
        return self.final_position

    def get_initial_position(self):
        """
        Get the initial mission position
        :return: return the current initial position
        :rtype: MissionPosition
        """
        return self.initial_position

    def get_position(self):
        """
        Get the final mission position
        :return: return the current final position
        :rtype: MissionPosition
        """
        return self.final_position

    def get_speed(self):
        """
        Get the speed
        :return: speed
        :rtype: float
        """
        return self.speed

    def get_tolerance(self):
        """
        Get the mission tolerance
        :return: return the mission tolerance
        :rtype: MissionTolerance
        """
        return self.tolerance


class MissionPark(MissionManeuver):
    def __init__(self, position=None, speed=0.0, time=0.0, tolerance=None):
        super(MissionPark, self).__init__(PARK_MANEUVER)
        self.position = position
        self.speed = speed
        self.time = time
        self.tolerance = tolerance

    def __str__(self):
        return "Park -> " + str(self.position) + " for " + str(self.time) + "s with tolerance " + str(self.tolerance)

    def set(self, position, speed, time, tolerance):
        """
        Set values
        :param position: mission position
        :type position: MissionPosition
        :param speed: speed
        :type speed: float
        :param time: time
        :type time: float
        :param tolerance: mission tolerance
        :type tolerance: MissionTolerance
        """
        self.position = position
        self.speed = speed
        self.time = time
        self.tolerance = tolerance

    def get_position(self):
        """
        Get the mission position
        :return: return the current position
        :rtype: MissionPosition
        """
        return self.position

    def get_time(self):
        """
        Get park time
        :return: park time
        :rtype: float
        """
        return self.time

    def get_speed(self):
        """
        Get the speed
        :return: speed
        :rtype: float
        """
        return self.speed

    def get_tolerance(self):
        """
        Get the mission tolerance
        :return: return the mission tolerance
        :rtype: MissionTolerance
        """
        return self.tolerance


class MissionConfiguration(object):
    def __init__(self, key="", value=""):
        self.key = key
        self.value = value

    def __str__(self):
        return "configuration -> " + self.key + ": " + self.value

    def set(self, key, value):
        """
        Set the mission configuration values
        :param key: key of the configuration parameter
        :type key: str
        :param value: value of the configuration parameter
        :type value: str
        """
        self.key = key
        self.value = value


class MissionAction(object):
    def __init__(self, action_id="", parameters=None):
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
        Set the mission action values
        :param action_id: action service id
        :type action_id: str
        :param parameters: list of parameters
        :type parameters: list
        """
        self.action_id = action_id
        self.parameters = parameters

    def get_action_id(self):
        """
        Get Action id value
        :return: return action id
        :rtype: str
        """
        return self.action_id

    def get_parameters(self):
        """
        Get list of parameters
        :return: list of parameters
        :rtype: list
        """
        return self.parameters


class Mission(object):
    def __init__(self):
        self.mission = list()
        self.num_steps = 0

    def __str__(self):
        ret = "MISSION: \n"
        for s in self.mission:
            ret = ret + str(s) + "\n\n"
        return ret

    def get_length(self):
        """
        get the mission length
        :return: return the number of steps in mission
        :rtype: int
        """
        return self.num_steps

    def copy(self, mission):
        """
        Copy Mission mission to current mission
        :param mission: mission to copy
        :type mission: Mission
        """
        self.mission = list(mission.mission)
        self.num_steps = mission.num_steps

    def add_step(self, step):
        """
        Add step to current mission
        :param step: step is a mission maneuver (MissionWaypoint, MissionSection or MissionPark)
        :type step: MissionManeuver
        """
        self.num_steps += 1
        self.mission.append(step)

    def get_step(self, step_id):
        """
        Get a mission step (maneuver) of current mission
        :param step_id: step id
        :type step_id: int
        :return:
        """
        if 0 <= step_id < len(self.mission):
            return self.mission[step_id]
        else:
            return None

    def insert_step(self, step_id, step):
        """
        Insert a mission step in current mission
        :param step_id: id of step
        :type step_id: id
        :param step: step maneuver to insert
        :type step: MissionManeuver
        """
        self.mission.insert(step_id, step)
        self.num_steps += 1

    def update_step(self, step_id, step):
        """
        Update a mission step in current mission
        :param step_id: id of step
        :type step_id: id
        :param step: new data to update
        :type step: MissionManeuver
        """
        self.remove_step(step_id)
        self.insert_step(step_id, step)

    def remove_step(self, step_id):
        """
        Remove a step of the mission
        :param step_id: id of step
        :type step_id: int
        """
        del self.mission[step_id]
        self.num_steps -= 1

    def size(self):
        """

        :return: return the size of the mission
        :rtype: int
        """
        return len(self.mission)

    def load_mission(self, mission_file_name):
        """
        Load a mission from XML
        :param mission_file_name: name of the mission file
        :type mission_file_name: str
        """
        tree = ET.parse(mission_file_name)
        root = tree.getroot()
        for mStep in root:
            mission_step = MissionStep()
            for child in mStep:
                if child.tag == 'configuration':
                    mission_step.load_configuration(child)
                elif child.tag == 'actions_list':
                    for action in child:
                        mission_step.load_action(action)
                elif child.tag == 'maneuver':

                    if child.get('type') == 'waypoint':
                        mission_step.load_waypoint_maneuver(child)
                    if child.get('type') == 'section':
                        mission_step.load_section_maneuver(child)
                    if child.get('type') == 'park':
                        mission_step.load_park_maneuver(child)
            self.add_step(mission_step)

    def write_mission(self, mission_file_name):
        """
        Write a mission in a XML file
        :param mission_file_name: name of the mission file
        """
        xml_mission = ET.Element('mission')
        for mission_step in self.mission:
            xml_mission_step = ET.SubElement(xml_mission, 'mission_step')

            if mission_step.maneuver.maneuver_type == WAYPOINT_MANEUVER:
                self.write_waypoint_maneuver(xml_mission_step, mission_step.maneuver)
            elif mission_step.maneuver.maneuver_type == SECTION_MANEUVER:
                self.write_section_maneuver(xml_mission_step, mission_step.maneuver)
            elif mission_step.maneuver.maneuver_type == PARK_MANEUVER:
                self.write_park_maneuver(xml_mission_step, mission_step.maneuver)

            if mission_step.actions:
                xml_actions = ET.SubElement(xml_mission_step, 'actions_list')
                for action in mission_step.actions:
                    self.write_action(xml_actions, action)

                    # TODO for configuration in mission_step.configurations:

        tree = ET.ElementTree(xml_mission)
        tree.write(mission_file_name, pretty_print=True)

    def write_configuration(self, root, step):
        """
        Write configuration in a XML
        :param root: parent tag
        :type root: str
        :param step: configuration step
        :type step: MissionConfiguration
        """
        xml_conf = ET.SubElement(root, 'configuration')
        xml_key = ET.SubElement(xml_conf, 'key')
        xml_key.text = step.key
        xml_value = ET.SubElement(xml_conf, 'value')
        xml_value.text = step.value

    def write_action(self, root, step):
        """
        Write Action in a XML
        :param root: parent tag
        :type root: str
        :param step: action step
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

    def write_position(self, root, position, tag):
        """
        Write position in a XML
        :param root: parent tag
        :type root: str
        :param position: mission position
        :type position: MissionPosition
        :param tag: tag of the current position ('position', 'initial_position', 'final_position')
        :type tag: str
        """
        xml_position = ET.SubElement(root, tag)
        xml_lat = ET.SubElement(xml_position, 'latitude')
        xml_lat.text = str(position.latitude)
        xml_lon = ET.SubElement(xml_position, 'longitude')
        xml_lon.text = str(position.longitude)
        xml_z = ET.SubElement(xml_position, 'z')
        xml_z.text = str(position.z)
        xml_mode = ET.SubElement(xml_position, 'altitude_mode')
        if position.altitude_mode:
            xml_mode.text = "True"
        else:
            xml_mode.text = "False"

    def write_tolerance(self, root, tolerance):
        """
        Write tolerance in a XML
        :param root: parent tag
        :type root: str
        :param tolerance: mission tolerance
        :type tolerance: MissionTolerance
        """
        xml_tolerance = ET.SubElement(root, 'tolerance')
        xml_x = ET.SubElement(xml_tolerance, 'x')
        xml_x.text = str(tolerance.x)
        xml_y = ET.SubElement(xml_tolerance, 'y')
        xml_y.text = str(tolerance.y)
        xml_z = ET.SubElement(xml_tolerance, 'z')
        xml_z.text = str(tolerance.z)

    def write_waypoint_maneuver(self, root, step):
        """
        Write a waypoint maneuver  in a XML
        :param root: parent tag
        :type root: str
        :param step: mission waypoint step
        :type step: MissionWaypoint
        """
        xml_maneuver = ET.SubElement(root, 'maneuver', {'type': 'waypoint'})
        self.write_position(xml_maneuver, step.position, 'position')
        xml_speed = ET.SubElement(xml_maneuver, 'speed')
        xml_speed.text = str(step.speed)
        self.write_tolerance(xml_maneuver, step.tolerance)

    def write_section_maneuver(self, root, step):
        """
        Write a section maneuver in a XML
        :param root: parent tag
        :type root: str
        :param step: mission section step
        :type step: MissionSection
        """
        xml_maneuver = ET.SubElement(root, 'maneuver', {'type': 'section'})
        self.write_position(xml_maneuver, step.initial_position, 'initial_position')
        self.write_position(xml_maneuver, step.final_position, 'final_position')
        xml_speed = ET.SubElement(xml_maneuver, 'speed')
        xml_speed.text = str(step.speed)
        self.write_tolerance(xml_maneuver, step.tolerance)

    def write_park_maneuver(self, root, step):
        """
        Write a park maneuver in a XML
        :param root: parent tag
        :type root: str
        :param step: mission park step
        :type step: MissionPark
        """
        xml_maneuver = ET.SubElement(root, 'maneuver', {'type': 'park'})
        self.write_position(xml_maneuver, step.position, 'position')
        xml_speed = ET.SubElement(xml_maneuver, 'speed')
        xml_speed.text = str(step.speed)
        xml_time = ET.SubElement(xml_maneuver, 'time')
        xml_time.text = str(step.time)
        self.write_tolerance(xml_maneuver, step.tolerance)


class MissionStep(object):
    def __init__(self):
        self.maneuver = None
        self.actions = list()
        self.configuration = list()

    def __str__(self):
        ret = "Mission step\n"
        ret = ret + "\t" + str(self.maneuver) + "\n"
        for a in self.actions:
            ret = ret + "\t" + a.action_id + "\n"
        return ret

    def add_action(self, action):
        """
        Add action to mission step
        :param action: action
        :type action: MissionAction
        """
        self.actions.append(action)

    def remove_action(self, id_action):
        """
        Remove action to mission step
        :param id_action:  action id
        :type id_action: int
        """
        if id_action >= 0:
            del self.actions[id_action]

    def add_configuration(self, config):
        """
        Add a configuration to step
        :param config: mission configuration
        :type config: MissionConfiguration
        """
        self.actions.append(config)

    def add_maneuver(self, maneuver):
        """
        Set maneuver to current step
        :param maneuver: mission maneuver
        :type maneuver: MissionManeuver
        """
        self.maneuver = maneuver

    def get_maneuver(self):
        """
        :return: return current maneuver
        :rtype: MissionManeuver
        """
        return self.maneuver

    def get_actions(self):
        """
        :return: return list of actions
        :rtype: list
        """
        return self.actions

    def load_configuration(self, config_element):
        """
        load a configuration from XML element
        :param config_element: configuration xml element
        :type config_element: str
        """
        key = config_element.find('key').text
        value = config_element.find('value').text
        config = MissionConfiguration(key, value)
        self.add_configuration(config)
        logger.info(config)

    def load_action(self, action_element):
        """
        load an action element from XML
        :param action_element: action xml element
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

    def load_position(self, xml_position):
        """
        load a position from XML element
        :param xml_position:  position xml element
        :type xml_position: str
        """
        latitude = xml_position.find('latitude').text
        longitude = xml_position.find('longitude').text
        z = xml_position.find('z').text
        altitude_mode = xml_position.find('altitude_mode').text
        if altitude_mode == "False":
            altitude_mode = False
        else:
            altitude_mode = True
        pose = MissionPosition(latitude, longitude, z, altitude_mode)
        return pose

    def load_tolerance(self, xml_tolerance):
        """
        load a tolerance from XML element
        :param xml_tolerance: tolerance xml element
        :type xml_tolerance: str
        """
        x = xml_tolerance.find('x').text
        y = xml_tolerance.find('y').text
        z = xml_tolerance.find('z').text
        tolerance = MissionTolerance(x, y, z)
        return tolerance

    def load_waypoint_maneuver(self, waypoint_maneuver):
        """
        load a waypoint maneuver from XML element
        :param waypoint_maneuver: waypoint maneuver xml element
        :type waypoint_maneuver: str
        """
        xml_position = waypoint_maneuver.find('position')
        xml_tolerance = waypoint_maneuver.find('tolerance')

        pose = self.load_position(xml_position)
        speed = waypoint_maneuver.find('speed').text
        tolerance = self.load_tolerance(xml_tolerance)

        wp = MissionWaypoint(pose, speed, tolerance)
        self.add_maneuver(wp)

    def load_section_maneuver(self, sector_maneuver):
        """
        load a section maneuver from XML element
        :param sector_maneuver: section maneuver xml element
        :type sector_maneuver: str
        """
        xml_initial_position = sector_maneuver.find('initial_position')
        xml_final_position = sector_maneuver.find('final_position')
        xml_tolerance = sector_maneuver.find('tolerance')

        initial_position = self.load_position(xml_initial_position)
        final_position = self.load_position(xml_final_position)
        speed = sector_maneuver.find('speed').text
        tolerance = self.load_tolerance(xml_tolerance)

        section = MissionSection(initial_position, final_position, speed, tolerance)
        self.add_maneuver(section)

    def load_park_maneuver(self, park_maneuver):
        """
        load a park maneuver from XML element
        :param park_maneuver: park maneuver xml element
        :type park_maneuver: str
        """
        xml_position = park_maneuver.find('position')
        xml_tolerance = park_maneuver.find('tolerance')

        position = self.load_position(xml_position)
        time = park_maneuver.find('time').text
        speed = park_maneuver.find('speed').text
        tolerance = self.load_tolerance(xml_tolerance)

        park = MissionPark(position, speed, time, tolerance)
        self.add_maneuver(park)


def test_write_xml():
    """Test to write a xml file"""
    mission = Mission()
    mission_step = MissionStep()

    # config = MissionConfiguration("key1", "value1")
    # missionStep.add_subStep(config)
    param = Parameter("abcd")
    param_2 = Parameter("2")
    parameters = list()
    parameters.append(param)
    parameters.append(param_2)
    action = MissionAction("action1", parameters)
    mission_step.add_action(action)
    wp = MissionWaypoint(MissionPosition(41.777, 3.030, 15.0, False),
                         0.5,
                         MissionTolerance(2.0, 2.0, 1.0))
    mission_step.add_maneuver(wp)
    mission.add_step(mission_step)
    mission_step2 = MissionStep()
    sec = MissionSection(MissionPosition(41.777, 3.030, 15.0, False),
                         MissionPosition(41.787, 3.034, 15.0, False),
                         0.5,
                         MissionTolerance(2.0, 2.0, 1.0))
    mission_step2.add_maneuver(sec)
    mission.add_step(mission_step2)
    mission_step3 = MissionStep()
    park = MissionPark(MissionPosition(41.777, 3.030, 15.0, False),
                       0.5,
                       120,
                       MissionTolerance(2.0, 2.0, 1.0))
    mission_step3.add_maneuver(park)
    mission.add_step(mission_step3)
    mission.write_mission('temp.xml')


def test_load_xml():
    """Test to load and write xml file"""
    mission = Mission()
    # load
    mission.load_mission('temp.xml')

    # write
    mission.write_mission('temp2.xml')


if __name__ == "__main__":
    test_write_xml()
    test_load_xml()