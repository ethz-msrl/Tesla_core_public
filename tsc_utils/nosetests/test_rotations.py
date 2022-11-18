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

from __future__ import division
from builtins import range

import numpy as np

from numpy.testing import assert_allclose, assert_almost_equal
import numpy.random

from tsc_utils import rotations

rs = numpy.random.RandomState(12345)


def test_rotation_matrix_to_axis_angle():
    axis = rs.rand(3)

    axis = axis / np.linalg.norm(axis)

    angle = rs.rand(1)[0] * np.pi

    R = rotations.euler_rodrigues(axis, angle)

    axis_, angle_ = rotations.rotation_matrix_to_axis_angle(R)

    assert_almost_equal(axis_, axis)
    assert_almost_equal(angle_, angle)


def test_axis_angle_to_quaternion():

    axis = rs.rand(3)
    axis = axis / np.linalg.norm(axis)

    angle = rs.rand(1)[0] * np.pi

    q = rotations.axis_angle_to_quaternion(axis, angle)

    axis_, angle_ = rotations.quaternion_to_axis_angle(q)

    assert_almost_equal(axis_, axis)
    assert_almost_equal(angle_, angle)


def test_quaternion_from_two_vectors():

    a = np.array([1, 0, 0], np.float)
    b = np.array([1, 0, 0], np.float)

    q = rotations.quaternion_from_two_vectors(a, b)

    assert_almost_equal(q, np.array([0, 0, 0, 1], np.float))

    a = np.array([1, 0, 0], np.float)
    b = np.array([0, 1, 0], np.float)

    q = rotations.quaternion_from_two_vectors(a, b)

    assert_almost_equal(q, np.array([0, 0, np.sqrt(2) / 2, np.sqrt(2) / 2], np.float))

    a = np.array([-1, 0, 0], np.float)
    b = np.array([1, 0, 0], np.float)
    q = rotations.quaternion_from_two_vectors(a, b)
    assert_almost_equal(q, np.array([0, 0, 1, 0], np.float))

    a = np.array([1, 1, 1], np.float)
    b = np.array([-1, -1, -1], np.float)
    q = rotations.quaternion_from_two_vectors(a, b)
    assert_almost_equal(q, np.array([0, -np.sqrt(2) / 2, np.sqrt(2) / 2, 0], np.float))


def test_quaternion_to_rotation_matrix():

    q = np.array([0.15849365, 0.59150635, 0.15849365, 0.77451905])
    R = rotations.quaternion_to_rotation_matrix(q)
    R_ = np.array(
        [
            [0.25, -0.0580127, 0.96650635],
            [0.4330127, 0.89951905, -0.0580127],
            [-0.8660254, 0.4330127, 0.25],
        ]
    )
    assert_allclose(R, R_)

    for i in range(100):
        # method for generating random quaternions
        v = rs.rand(
            3,
        )
        q = np.array(
            [
                np.sqrt(1 - v[0]) * np.sin(2 * np.pi * v[1]),
                np.sqrt(1 - v[0]) * np.cos(2 * np.pi * v[1]),
                np.sqrt(v[0]) * np.sin(2 * np.pi * v[2]),
                np.sqrt(v[0]) * np.cos(2 * np.pi * v[2]),
            ]
        )
        R = rotations.quaternion_to_rotation_matrix(q)
        q_ = rotations.rotation_matrix_to_quaternion(R)
        assert rotations.quaternion_is_equal(q, q_)


def test_rotation_matrix_to_quaternion():
    R = np.array(
        [
            [0.25, -0.0580127, 0.96650635],
            [0.4330127, 0.89951905, -0.0580127],
            [-0.8660254, 0.4330127, 0.25],
        ]
    )
    q = rotations.rotation_matrix_to_quaternion(R)
    q_ = np.array([0.15849365, 0.59150635, 0.15849365, 0.77451905])
    assert_almost_equal(q, q_)


def test_skew():

    a = rs.rand(
        3,
    )

    S = rotations.skew(a)

    assert_allclose(-S.transpose(), S)


def test_unskew():
    a = rs.rand(
        3,
    )

    S = rotations.skew(a)

    a_ = rotations.unskew(S)

    assert_allclose(a, a_)


def test_exp_log_SO3():
    w = rs.rand(
        3,
    )

    wx = rotations.skew(w)

    R = rotations.exp_SO3(wx)

    assert rotations.is_rotation_matrix(R)

    wx_ = rotations.log_SO3(R)

    assert_allclose(wx, wx_)


def test_pose_6D_to_matrix():
    w = rs.rand(
        3,
    )
    t = rs.rand(
        3,
    )

    T = rotations.pose_6D_to_matrix(w, t)
    assert rotations.is_rotation_matrix(T[0:3, 0:3])
    assert_allclose(T[0:3, 3], t)


def test_quaternion_rotate_vector():
    v = np.array([1.0, 2.0, 3])
    q = np.array([0.15849365, 0.59150635, 0.15849365, 0.77451905])
    R = rotations.quaternion_to_rotation_matrix(q)
    vr = rotations.quaternion_rotate_vector(q, v)
    vr2 = np.dot(R, v)
    assert_almost_equal(vr, vr2)


def test_jac_normalized_quaternion():
    q = np.array([0.15849365, 0.59150635, 0.15849365, 0.77451905])
    dqn_dq = rotations.jac_normalized_quaternion(q)
    eps = 1e-5
    delm = eps * np.eye(4)
    dqn_dq_fd = np.zeros((4, 4), np.float)
    for i in range(4):
        q_ = q + delm[:, i]
        q_ /= np.linalg.norm(q_)
        dqn_dq_fd[:, i] = (q_ - q) / eps
    assert_almost_equal(dqn_dq, dqn_dq_fd, 1e-2)


def test_jac_tangent_quaternion():
    q = np.array([0.15849365, 0.59150635, 0.15849365, 0.77451905])
    dT_dq = rotations.jac_tangent_quaternion(q)

    # R = rotations.quaternion_to_rotation_matrix(q)
    # T = R.dot(rotations.ez())
    T = rotations.quaternion_rotate_vector(q, rotations.ez())
    eps = 1e-4
    del_m = eps * np.eye(4)
    dT_dq_fd = np.zeros((3, 4), np.float)
    for i in range(4):
        q_ = q + del_m[:, i]
        q_ /= np.linalg.norm(q_)
        T_ = rotations.quaternion_rotate_vector(q_, rotations.ez())
        dT_dq_fd[:, i] = (T_ - T) / eps
    assert_allclose(dT_dq, dT_dq_fd, 1e-2)


def test_jac_rotated_vector():

    v = rs.rand(
        3,
    )
    u = rs.rand(
        3,
    )
    th = np.linalg.norm(v)
    ax = v / th

    dRu_dv = rotations.jac_rotated_vector(v, u)

    Ru = rotations.euler_rodrigues(ax, th).dot(u)

    eps = 1e-5
    delm = eps * np.eye(3)
    dRu_dv_fd = np.zeros((3, 3), np.float)
    for i in range(3):
        v_ = v + delm[:, i]
        th_ = np.linalg.norm(v_)
        ax_ = v_ / th_
        Ru_ = rotations.euler_rodrigues(ax_, th_).dot(u)
        dRu_dv_fd[:, i] = (Ru_ - Ru) / eps

    assert_allclose(dRu_dv, dRu_dv_fd, rtol=1e-3)
