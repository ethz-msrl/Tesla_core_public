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
from itertools import permutations

import numpy as np
from numpy.testing import assert_almost_equal, assert_array_almost_equal

import tsc_utils.rotations as rot
from tsc_utils.coordinates import (axis_to_elevation_azimuth,
                                   elevation_azimuth_to_axis,
                                   cartesian_to_spherical,
                                   spherical_to_cartesian)


def test_elevation_azimuth_to_axis():
    assert_almost_equal(elevation_azimuth_to_axis(0, 0, pole=1, primary=2), rot.ez())
    assert_almost_equal(
        elevation_azimuth_to_axis(np.pi / 2, 0, pole=1, primary=2), rot.ey()
    )
    assert_almost_equal(
        elevation_azimuth_to_axis(0, np.pi / 2, pole=1, primary=2), rot.ex()
    )


def test_axis_to_elevation_azimuth():
    for (pole, primary) in permutations((0, 1, 2), 2):
        el_v = np.linspace(0, 2 * np.pi, 10)
        az_v = np.linspace(0, 2 * np.pi, 10)

        for el in el_v:
            for az in az_v:
                ax = elevation_azimuth_to_axis(el, az, pole, primary)
                el_, az_ = axis_to_elevation_azimuth(ax, pole, primary)
                ax_ = elevation_azimuth_to_axis(el_, az_, pole, primary)
                assert_almost_equal(ax_, ax)


def test_cartesian_to_spherical():
    x = np.array([-1., 2.*np.sqrt(3.), 0.])
    y = np.array([1., 6., 0.])
    z = np.array([-np.sqrt(2), -4, 0.])
    r = np.array([2., 8, 0.])
    incl = np.array([3.*np.pi/4., 2.*np.pi/3.,  0.])
    az = np.array([3.*np.pi/4., np.pi/3., 0.])

    r_, incl_, az_ = cartesian_to_spherical(x, y, z)
    assert_array_almost_equal(r, r_)
    assert_array_almost_equal(incl, incl_)
    assert_array_almost_equal(az, az_)
    assert_almost_equal(r[0], r_[0])
    assert_almost_equal(incl[0], incl_[0])
    assert_almost_equal(az[0], az_[0])

    assert np.shape(x) == np.shape(r)
    assert np.shape(r) == np.shape(r_)


def test_spherical_to_cartesian():
    x = np.array([-1., 2.*np.sqrt(3.), 0.])
    y = np.array([1., 6., 0.])
    z = np.array([-np.sqrt(2), -4, 0.])
    r = np.array([2., 8, 0.])
    incl = np.array([3.*np.pi/4., 2.*np.pi/3.,  0.])
    az = np.array([3.*np.pi/4., np.pi/3., 0.])

    x_, y_, z_ = spherical_to_cartesian(r, incl, az)
    assert_array_almost_equal(x, x_)
    assert_array_almost_equal(y, y_)
    assert_array_almost_equal(z, z_)
    assert_almost_equal(x[0], x_[0])
    assert_almost_equal(y[0], y_[0])
    assert_almost_equal(z[0], z_[0])

    assert np.shape(r) == np.shape(x)
    assert np.shape(x) == np.shape(x_)
