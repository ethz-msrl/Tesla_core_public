# coding: utf-8
# Tesla - A ROS-based framework for performing magnetic manipulation

# Software License Agreement (BSD License)

# 2022 ETH Zurich, ​MAVT, Multi Scale Robotics Lab (MSRL) , Prof Bradley J. Nelson
# All rights reserved.

# Redistribution and use of this software in source and binary forms,
# with or without modification, are permitted provided that the following conditions are met:

# * Redistributions of source code must retain the above
#   copyright notice, this list of conditions and the
#   following disclaimer.

# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.

# * All advertising materials mentioning features or use of this software
#   must display the following acknowledgement:
#   “This product includes software developed by the Multi-Scale Robotics Lab,
#   ETH Zurich, Switzerland and its contributors.”

# * Neither the name of MSRL nor the names of its
#   contributors may be used to endorse or promote products
#   derived from this software without specific prior
#   written permission of MSRL.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
# THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
# IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
# PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
# OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
# STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
# OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import numpy as np


def elevation_azimuth_to_axis(elevation, azimuth, pole=1, primary=2):
    """
    Converts the elevation, azimuth to a direction vector

    With the default parametrization, 0 azimuth and 0 elevation is assumed to be
    aligned with the z-axis and the pole is along the y-axis

    Args:
        * elevation (float): the elevation angle in radians
        * azimuth (float): the azimuth angle in radians
        * pole (int): denotes the parametrization or axis of the pole. Can be 0,1 or 2.
        * primary (int): denotes the direction at 0 elevation and 0 azimuth.
        Can be 0, 1, 2. Must be different from pole
    Returns:
        np.ndarray of length 3 with the position
    """
    if pole not in (0, 1, 2):
        raise ValueError("pole should be 0,1 or 2")
    if primary not in (0, 1, 2):
        raise ValueError("primary should be 0,1, or 2")
    if pole == primary:
        raise ValueError("Pole and Primary axis should be different")
    third = ({0, 1, 2} - {pole, primary}).pop()

    ax = np.zeros((3,), np.float64)
    ax[pole] = np.sin(elevation)
    ax[primary] = np.cos(elevation) * np.cos(azimuth)
    ax[third] = np.cos(elevation) * np.sin(azimuth)

    return ax


def axis_to_elevation_azimuth(axis, pole=1, primary=2):
    """
    Converts a direction vector to a elevation azimuth

    With the default parametrization, 0 azimuth and 0 elevation is assumed to be
    aligned with the z-axis and the pole is along the y-axis

    Args:
        * axis (np.ndarray): array of length 3 with a direction. Does not need to be normalized.
        * pole (int): denotes the parametrization or axis of the pole. Can be 0,1 or 2.
        * primary (int): denotes the direction at 0 elevation and 0 azimuth.
        Can be 0, 1, 2. Must be different from pole
    Returns:
        tuple (float, float) with the elevation, azimuth
    """
    axis_l = np.linalg.norm(axis)
    if axis_l < 1e-12:
        raise ValueError("axis is near zero length")
    axis /= axis_l
    if pole not in (0, 1, 2):
        raise ValueError("pole should be 0,1 or 2")
    if primary not in (0, 1, 2):
        raise ValueError("primary should be 0,1, or 2")
    if pole == primary:
        raise ValueError("Pole and Primary axis should be different")
    third = ({0, 1, 2} - {pole, primary}).pop()

    el = np.arcsin(axis[pole])
    az = np.arctan2(axis[third], axis[primary])
    return el, az


def cartesian_to_spherical(x, y, z):
    """
    Converts vectors from cartesian to spherical coordinates

    Args:
        * x (np.ndarray): x coordinates
        * y (np.ndarray): y coordinates
        * z (np.ndarray): z coordinates
    Returns:
        np.ndarrays of shape x, y, z with the radius, inclination, azimuth
        in radian
    """
    if (np.shape(x) != np.shape(y)
            or np.shape(x) != np.shape(z)
            or np.shape(y) != np.shape(z)):
        raise ValueError("x, y, z arrays should have the same shape")

    r = np.sqrt(x**2 + y**2 + z**2)
    incl = np.arctan2(np.sqrt(x**2 + y**2), z)
    az = np.arctan2(y, x)

    return r, incl, az


def spherical_to_cartesian(radius, inclination, azimuth):
    """
    Converts vectors from spherical to cartesian coordinates

    Args:
        * radius (np.ndarray): the radius
        * inclination (np.ndarray): the inclination in radians
        * azimuth (np.ndarray): the azimuth in radians
    Returns:
        np.ndarrays of shape radius, inclination, azimuth with the
        x, y, z cartesian coordinates
    """
    if (np.shape(radius) != np.shape(inclination)
            or np.shape(radius) != np.shape(azimuth)
            or np.shape(inclination) != np.shape(azimuth)):
        raise ValueError(
            "radius, inclination, azimuth arrays should have the same shape")

    x = radius * np.cos(azimuth) * np.sin(inclination)
    y = radius * np.sin(azimuth) * np.sin(inclination)
    z = radius * np.cos(inclination)

    return x, y, z
