#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
#
# This file is subject to the terms and conditions defined in file
# 'LICENSE.txt', which is part of this source code package.

"""
Module to saturate values and lists of values.
"""

from __future__ import print_function
import numpy as np


def saturate_vector(values, min_max):
    """
    Saturate values with common limits.

    :param values: Values to saturate.
    :type values: list(float)
    :param min_max: Limits to saturate values.
    :type min_max: list(float)

    :return: Saturated values.
    :rtype: list(float)
    """
    assert len(values) == len(min_max)
    ret = np.zeros(len(values))
    for i, v in enumerate(values):
        if v < -min_max[i]:
            ret[i] = -min_max[i]
        elif v > min_max[i]:
            ret[i] = min_max[i]
        else:
            ret[i] = v
    return ret


def saturate_value(values, min_max):
    """
    Saturate values with common limits.

    :param values: Values to saturate.
    :type values: list(float)
    :param min_max: Limit to saturate values.
    :type min_max: float

    :return: Saturated values.
    :rtype: list(float)
    """
    ret = np.zeros(len(values))
    for i, v in enumerate(values):
        if v < -min_max:
            ret[i] = -min_max
        elif v > min_max:
            ret[i] = min_max
        else:
            ret[i] = v
    return ret


def saturate_value_float(v, min_max):
    """
    Saturate a float value around [-l, +l].

    :param v: Value to saturate.
    :type v: float
    :param min_max: Limit to saturate the value.
    :type min_max: float

    :return: Saturated value.
    :rtype: float
    """
    if v > min_max:
        return min_max
    if v < -min_max:
        return -min_max
    return v


def test():
    """
    Test functions in this module.
    """
    print("Saturate_vector:")
    print(str(saturate_vector([1.8, 0.3, -3.2, -0.7], np.ones(4))))

    print("Saturate_value:")
    print(str(saturate_value([1.8, 0.3, -3.2, -0.7], 1.0)))

    print("Saturate_value float:")
    print(str(saturate_value_float(1.8, 1.0)))


if __name__ == '__main__':
    test()
