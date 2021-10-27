#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
#
# This file is subject to the terms and conditions defined in file
# 'LICENSE.txt', which is part of this source code package.

"""
Module to wrap angles between [0, 2pi] and compute different representations in degrees, minutes and seconds.
"""

from __future__ import print_function
from math import pi, floor


def wrap_angle(angle):
    """
    Wrap angle between 0 and 2 pi.

    :param angle: Angle to wrap around the limits.
    :type angle: float

    :return: Wrapped angle around the limits.
    :rtype: float
    """
    return angle + (2.0 * pi * floor((pi - angle) / (2.0 * pi)))


def degree_minutes_to_degrees(lat, lon):
    """
    Transform latitude and longitude in the format DDDMM.MM (degrees minutes) to the format DDD.DD (decimal degrees).

    :param lat: Latitude in DDDMM.MM format.
    :type lat: float
    :param lon: Longitude in DDDMM.MM format.
    :type lon: float

    :return: Latitude and longitude in DDD.DD format.
    :rtype: float, float
    """
    lat_deg, lat_min = __split_degree_minutes__(lat)
    lon_deg, lon_min = __split_degree_minutes__(lon)
    return lat_deg + lat_min / 60.0, lon_deg + lon_min / 60.0


def degrees_to_degree_minutes(lat, lon):
    """
    Transform latitude and longitude in the format DDD.DD (decimal degrees) to the format DDDMM.MM (degrees minutes).

    :param lat: Latitude in DDD.DD format.
    :type lat: float
    :param lon: Longitude in DDD.DD format.
    :type lon: float

    :return: Latitude and longitude in DDDMM.MM format.
    :rtype: float, float
    """
    lat_degree = __degree_to_degree_minutes_aux__(lat)
    lon_degree = __degree_to_degree_minutes_aux__(lon)
    return lat_degree, lon_degree


def degrees_to_degree_minute_seconds(lat, lon):
    """
    Transform coordinates from DDD.DDDDD format to a string of DDDºMM'SS.SSS''.

    :param lat: Latitude in DDD.DDDDD format.
    :type lat: float
    :param lon: Longitude in DDD.DDDDD format.
    :type lon: float

    :return: Latitude and longitude in DDDºMM'SS.SSS format.
    :rtype: str, str
    """
    lat_str = __degree_to_degree_minute_seconds_aux__(lat)
    lon_str = __degree_to_degree_minute_seconds_aux__(lon)
    return [lat_str, lon_str]


def __degree_to_degree_minute_seconds_aux__(value):
    """
    Transform coordinates from DDD.DDDDD format to a string of DDDºMM'SS.SSS''.

    :param value: Angle in DDD.DDDDD format.
    :type value: float

    :return: Angle in DDDºMM'SS.SSS format.
    :rtype: str
    """
    d = int(value)
    t = (value - d) * 60
    m = int(t)
    s = (t - m) * 60
    return "{:03d}º {:02d}' {:06.3f}''".format(d, m, s)


def __degree_to_degree_minutes_aux__(value):
    """
    Transform angle in the format DDD.DD (decimal degrees) to the format DDDMM.MM (degrees minutes).

    :param value: Angle in the format DDD.DD.
    :type value: float

    :return: Angle in the format DDDMM.MM.
    :rtype: float
    """
    val = str(value).split('.')
    minute = float('0.' + val[1]) * 60.0
    if minute < 10.0:
        return float(val[0] + '0' + str(minute))
    else:
        return float(val[0] + str(minute))


def __split_degree_minutes__(value):
    """
    Transform angle DDDMM.MM to degrees DDD and minutes MM.MM.

    :param value: Angle in the format DDDMM.MM.
    :type value: float

    :return: Angle degrees integer and angle minutes float.
    :rtype: int, float
    """
    val = str(value).split('.')
    val_min = val[0][-2] + val[0][-1] + '.' + val[1]
    val_deg = ''
    for i in range(len(val[0]) - 2):
        val_deg = val_deg + val[0][i]
    return int(val_deg), float(val_min)


def test():
    """
    Test the functions in this module.
    """
    print("Normalize angle 7.14 = " + str(wrap_angle(7.14)))
    result = degree_minutes_to_degrees('04156.74', '00002.48')
    print("degree_mintues_to_degrees 4156.74, 0002.48 (DDMM.MM) = " + str(result))
    print("degrees_to_degree_minutes " + str(result) + " (DDDD.DD) = " +
          str(degrees_to_degree_minute_seconds(result[0], result[1])))
    result2 = degrees_to_degree_minute_seconds(result[0], result[1])
    print("degrees_to_degree_minute_seconds " + str(result) + " (DDDD.DD) = " + str(result2[0]) + str(result2[1]))


if __name__ == '__main__':
    test()
