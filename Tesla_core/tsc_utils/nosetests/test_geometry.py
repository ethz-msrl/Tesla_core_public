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

from builtins import range

import numpy as np
from numpy.testing import assert_allclose, assert_almost_equal
import numpy.random

from tsc_utils.geometry import (angle_between, angle_between_vectors,
                                get_plane_orthogonal_projection_matrix,
                                get_vector_orthogonal_projection_matrix,
                                project_vector_onto_plane,
                                project_vector_onto_vector)

rs = numpy.random.RandomState(12345)


def test_angle_between():
    a = np.array([1, 0, 0])
    b = np.array([0, 1, 0])
    n = np.array([0, 0, 1])

    assert_almost_equal(angle_between(a, b, n), np.pi / 2)

    a = np.array([1, 0, 0])
    b = np.array([-1, 0, 0])
    assert_almost_equal(angle_between(a, b, n), np.pi)


def test_angle_between_vectors():
    for i in range(100):
        a = rs.rand(
            3,
        )
        na = np.linalg.norm(a)
        b = rs.rand(
            3,
        )
        nb = np.linalg.norm(b)
        th = np.arccos(np.dot(a, b) / (na * nb))
        th_ = angle_between_vectors(a, b)
        assert_almost_equal(th, th_)


def test_get_plane_orthogonal_projection_matrix():
    for i in range(100):
        normal = rs.rand(
            3,
        )

        P = get_plane_orthogonal_projection_matrix(normal)
        assert_allclose(np.dot(P, normal), np.zeros((3,), np.float), atol=1e-8)


def test_project_vector_onto_plane():
    for i in range(100):
        normal = rs.rand(
            3,
        )
        vector = rs.rand(
            3,
        )

        projected = project_vector_onto_plane(normal, vector)
        assert_allclose(np.dot(projected, normal), 0.0, atol=1e-8)

        # another way to calculate the projection
        P = get_plane_orthogonal_projection_matrix(normal)
        projected_ = P.dot(vector)
        assert_allclose(projected, projected_)


def test_project_vector_onto_vector():
    for i in range(100):
        normal = rs.rand(
            3,
        )
        vector = rs.rand(
            3,
        )

        projected = project_vector_onto_vector(normal, vector)
        assert_almost_equal(
            np.dot(projected, normal),
            np.linalg.norm(projected) * np.linalg.norm(normal),
        )

        # another way to calculate the projection
        P = get_vector_orthogonal_projection_matrix(normal)
        projected_ = P.dot(vector)
        assert_allclose(projected, projected_)
