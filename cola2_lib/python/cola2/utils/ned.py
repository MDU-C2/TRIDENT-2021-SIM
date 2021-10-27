#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
#
# This file is subject to the terms and conditions defined in file
# 'LICENSE.txt', which is part of this source code package.

"""
Module to handle transformations from the WGS84 frame (usually GPS data) to the NED (north, east, down) frame where
a vehicle localizes itself.
"""

from __future__ import print_function
import math
import numpy as np


class NED:
    """
    Class to transform to and from geodetic WGS84 and NED coordinates.

    :param lat: Origin latitude of the NED plane.
    :type lat: float
    :param lon: Origin longitude of the NED plane.
    :type lon: float
    :param height: Origin height of the NED plane (0 corresponds to sea level).
    :type height: float
    """

    def __init__(self, lat, lon, height):
        """
        Constructor.
        """
        # Constants defined by the World Geodetic System 1984 (WGS84)
        self.a = 6378137
        self.b = 6356752.3142
        self.esq = 6.69437999014 * 0.001
        self.e1sq = 6.73949674228 * 0.001
        self.f = 1 / 298.257223563

        # Save NED origin
        self.init_lat = math.radians(lat)
        self.init_lon = math.radians(lon)
        self.init_h = height
        self.init_ecef = self.geodetic2ecef([lat, lon, height])
        phiP = math.atan2(self.init_ecef[2],
                          math.sqrt(self.init_ecef[0] ** 2 + self.init_ecef[1] ** 2))
        self.ecef_to_ned_matrix = __nRe__(phiP, self.init_lon)
        self.ned_to_ecef_matrix = __nRe__(self.init_lat, self.init_lon).T

    def geodetic2ecef(self, coord):
        """
        Convert geodetic WGS84 coordinates to ECEF.

        :param coord: Geodetic coordinates (latitude_deg, longitude_deg, height_meters).
        :type coord: list(float)

        :return: ECEF coordinates.
        :rtype: list(float)
        """
        # http://code.google.com/p/pysatel/source/browse/trunk/coord.py?r=22
        # latitude and longitude in radians
        lat = math.radians(coord[0])
        lon = math.radians(coord[1])
        # compute ECEF
        xi = math.sqrt(1 - self.esq * math.sin(lat)**2)
        x = (self.a / xi + coord[2]) * math.cos(lat) * math.cos(lon)
        y = (self.a / xi + coord[2]) * math.cos(lat) * math.sin(lon)
        z = (self.a / xi * (1 - self.esq) + coord[2]) * math.sin(lat)
        return np.array([x, y, z])

    def ecef2geodetic(self, ecef):
        """
        Convert geodetic (WGS84) coordinates to ECEF.

        :param ecef: ECEF coordinates.
        :type ecef: list(float)

        :return: Geodetic WGS84 coordinates.
        :rtype: list(float)
        """
        # J. Zhu, "Conversion of Earth-centered Earth-fixed coordinates
        # to geodetic coordinates," IEEE Transactions on Aerospace and
        # Electronic Systems, vol. 30, pp. 957-961, 1994.
        x = ecef[0]
        y = ecef[1]
        z = ecef[2]

        r = math.sqrt(x * x + y * y)
        Esq = self.a * self.a - self.b * self.b
        F = 54 * self.b * self.b * z * z
        G = r * r + (1 - self.esq) * z * z - self.esq * Esq
        C = (self.esq * self.esq * F * r * r) / (pow(G, 3))
        S = __cbrt__(1 + C + math.sqrt(C * C + 2 * C))
        P = F / (3 * pow((S + 1 / S + 1), 2) * G * G)
        Q = math.sqrt(1 + 2 * self.esq * self.esq * P)
        r_0 = -(P * self.esq * r) / (1 + Q) + math.sqrt(0.5 * self.a * self.a * (1 + 1.0 / Q) -
                                                        P * (1 - self.esq) * z * z / (Q * (1 + Q)) - 0.5 * P * r * r)
        U = math.sqrt(pow((r - self.esq * r_0), 2) + z * z)
        V = math.sqrt(pow((r - self.esq * r_0), 2) + (1 - self.esq) * z * z)
        Z_0 = self.b * self.b * z / (self.a * V)
        h = U * (1 - self.b * self.b / (self.a * V))
        lat = math.atan((z + self.e1sq * Z_0) / r)
        lon = math.atan2(y, x)
        return math.degrees(lat), math.degrees(lon), h

    def ecef2ned(self, ecef):
        """
        Converts ECEF coordinate pos into local-tangent-plane NED coordinates relative to another ECEF coordinate ref.

        :param ecef: ECEF coordinates.
        :type ecef: list(float)

        :return: NED coordinates.
        :rtype: list(float)
        """
        p = ecef - self.init_ecef
        ned = np.dot(self.ecef_to_ned_matrix, p)
        ned[2] = -ned[2]
        return ned

    def ned2ecef(self, ned):
        """
        NED to ECEF coordinate system conversion.

        :param ned: NED coordinates.
        :type ned: list(float)

        :return: ECEF coordinates.
        :rtype: list(float)
        """
        ned = np.array([ned[0], ned[1], -ned[2]])
        res = np.dot(self.ned_to_ecef_matrix, ned) + self.init_ecef
        return res

    def geodetic2ned(self, coord):
        """
        Geodetic position to a local NED system.

        :param coord: Geodetic WGS84 coordinates.
        :type coord: list(float)

        :return: NED coordinates.
        :rtype: list(float)
        """
        ecef = self.geodetic2ecef(coord)
        return self.ecef2ned(ecef)

    def ned2geodetic(self, ned):
        """
        Local NED position to geodetic WGS84.

        :param ned: NED coordinates.
        :type ned: list(float)

        :return: Geodetic WGS84 coordinates.
        :rtype: list(float)
        """
        ecef = self.ned2ecef(ned)
        return self.ecef2geodetic(ecef)


def __cbrt__(x):
    """
    Compute cubic root of a number.

    :param x: Number to compute cubic root.
    :type x: float

    :return: Cubic root of the input number.
    :rtype: float
    """
    if x >= 0:
        return pow(x, 1.0 / 3.0)
    return -pow(abs(x), 1.0 / 3.0)


def __nRe__(lat, lon):
    """
    Matrix used to convert between ECEF and NED frames.

    :param lat: Latitude.
    :type lat: float
    :param lon: Longitude.
    :type lon: float

    :return: Matrix to convert between ECEF and NED frames.
    :rtype: list(float)
    """
    sinLat = math.sin(lat)
    sinLon = math.sin(lon)
    cosLat = math.cos(lat)
    cosLon = math.cos(lon)
    mx = np.array([-sinLat * cosLon, -sinLat * sinLon, cosLat,
                   -sinLon, cosLon, 0.0,
                   cosLat * cosLon, cosLat * sinLon, sinLat]).reshape(3, 3)
    return mx


def test():
    """
    Test functions in this module.
    """
    lat3 = 39.997417  # DDD.DDDDD
    lon3 = 6.007667  # DDD.DDDDD
    lat4 = 40.017417  # DDD.DDDDD
    lon4 = 6.00667  # DDD.DDDDD
    lat5 = 39.9717417  # DDD.DDDDD
    lon5 = 6.01667  # DDD.DDDDD

    ned = NED(lat3, lon3, 0.0)
    ned_1 = ned.geodetic2ned([lat4, lon4, 0.0])
    print(ned_1)
    ned_2 = ned.geodetic2ned([lat5, lon5, 0.0])
    print(ned_2)
    lat, lon, h = ned.ned2geodetic(ned_2)
    print(lat, lon, h)
    print(np.sqrt((ned_1[0] - ned_2[0])**2 + (ned_1[1] - ned_2[1])**2))


if __name__ == '__main__':
    test()
