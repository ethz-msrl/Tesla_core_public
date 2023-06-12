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
from geometry_msgs.msg import Pose, Vector3
from mag_msgs.msg import Gradient5Vector
from numpy.testing import assert_almost_equal
import numpy.random

from tsc_utils.conversions import (
    gradient5_to_np,
    np_to_gradient5,
    np_to_point_msg,
    np_to_pose_msg,
    np_to_quaternion_msg,
    np_to_vector3_msg,
    point_msg_to_np,
    pose_msg_to_np,
    quaternion_msg_to_np,
    vector3_msg_to_np,
    np_to_colorrgba,
    colorrgba_to_np,
)

rs = numpy.random.RandomState(12345)


def test_point_msg_to_from_np():
    p = rs.rand(3)
    p_msg = np_to_point_msg(p)
    p_ = point_msg_to_np(p_msg)

    assert_almost_equal(p_, p)


def test_quaternion_msg_to_from_np():
    v = rs.rand(3)
    q = np.zeros((4,), np.float64)
    q[0:3] = v
    q[3] = np.linalg.norm(v)

    q_msg = np_to_quaternion_msg(q)
    q_ = quaternion_msg_to_np(q_msg)

    assert_almost_equal(q_, q)


def test_np_to_pose_msg():

    T = np.eye(4)
    T[0:3, 3] = [0, 0, 1]

    msg = np_to_pose_msg(T)

    t_ = point_msg_to_np(msg.position)
    q_ = quaternion_msg_to_np(msg.orientation)

    assert_almost_equal(t_, [0, 0, 1])
    assert_almost_equal(q_, [0, 0, 0, 1])


def test_pose_msg_to_np():

    msg = Pose()
    msg.position.x = 0
    msg.position.y = 0
    msg.position.z = 1

    msg.orientation.x = 0
    msg.orientation.y = 0
    msg.orientation.z = 0
    msg.orientation.w = 1

    T = pose_msg_to_np(msg)

    T_ = np.eye(4)
    T_[0:3, 3] = [0, 0, 1]

    assert_almost_equal(T_, T)


def test_vector3_msg_to_np():
    msg = Vector3()
    msg.x = 0
    msg.y = 0
    msg.z = 1

    V = vector3_msg_to_np(msg)
    V_ = np.array([0, 0, 1.0])
    assert_almost_equal(V, V_)


def test_np_to_vector3_msg():
    v = np.array([0, 0, 1.0])
    msg = np_to_vector3_msg(v)
    v_ = vector3_msg_to_np(msg)
    assert_almost_equal(v, v_)


def test_gradient5_to_np():
    g = Gradient5Vector(0, 0, 0, 0, 0)
    assert_almost_equal(gradient5_to_np(g), np.zeros((5,), np.float64))


def test_np_to_gradient5():
    g = rs.rand(
        5,
    )
    gv = np_to_gradient5(g)
    assert gv.xx == g[0]
    assert gv.xy == g[1]
    assert gv.xz == g[2]
    assert gv.yy == g[3]
    assert gv.yz == g[4]


def test_np_to_colorrgba():
    c = np.array([0, 0, 0, 1])
    msg = np_to_colorrgba(c)
    c_ = colorrgba_to_np(msg)
    assert_almost_equal(c, c_)
