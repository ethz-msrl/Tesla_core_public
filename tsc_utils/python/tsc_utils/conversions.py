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
from geometry_msgs.msg import Point, Pose, Quaternion, Vector3
from std_msgs.msg import ColorRGBA
from mag_msgs.msg import Gradient5Vector
from tsc_utils.rotations import (
    quaternion_to_rotation_matrix,
    rotation_matrix_to_quaternion,
)


def point_msg_to_np(p):
    """
    Converts from a Point message to a numpy array

    Args:
        p(geometry_msgs.msg.Point) : point message to convert
    Returns:
        np.ndarray: version of p
    """

    return np.array([p.x, p.y, p.z], np.float)


def np_to_point_msg(p):
    """
    Converts from numpy array to a Point message

    Args:
        p (np.ndarray): 3D point
    Returns:
        geometry_msgs.msg.Point: the converted point
    """
    return Point(p[0], p[1], p[2])


def np_to_vector3_msg(p):
    """
    Converts from numpy array to a Point message

    Args:
        p (np.ndarray): 3D point
    Returns:
        geometry_msgs.msg.Vector3: the converted point
    """
    return Vector3(p[0], p[1], p[2])


def vector3_msg_to_np(p):
    """
    Converts from a Vector3 message to a numpy array

    Args:
        p(geometry_msgs.msg.Vector3) : point message to convert
    Returns:
        np.ndarray: version of p
    """

    return np.array([p.x, p.y, p.z], np.float)


def quaternion_msg_to_np(q):
    """
    Converts from quaternion msg to numpy array

    Args:
        q (geometry_msgs.msg.Quaternion): the quaternion to convert
    Returns:
        np.ndarray: the quaternion in format [x, y, z, w]
    """
    return np.array([q.x, q.y, q.z, q.w])


def np_to_quaternion_msg(q):
    """
    Converts from numpy array to quaternion msg

    Args:
        q (np.ndarray): the quaternion in format [x, y, z, w] to convert
    Returns:
        geometry_msgs.msg.Quaternion: the converted quaternion
    """
    return Quaternion(w=q[3], x=q[0], y=q[1], z=q[2])


def pose_msg_to_np(P):
    """
    Converts from a pose msg to numpy array

    Args:
        p (geometry_msgs.msg.Pose): pose message to convert
    Returns:
        np.ndarray: converted to a 4x4 transformation matrix
    """
    T = np.eye(4)
    T[0:3, 3] = point_msg_to_np(P.position)
    q = quaternion_msg_to_np(P.orientation)
    T[0:3, 0:3] = quaternion_to_rotation_matrix(q)
    return T


def np_to_pose_msg(T):
    """
    Converts from a 4x4 transformation matrix to a pose msg

    Args:
        p (np.ndarray): a 4x4 transformation matrix
    Returns:
        geometry_msgs.msg.Pose: the converted pose msg
    """
    msg = Pose()
    msg.position = np_to_point_msg(T[0:3, 3])
    q = rotation_matrix_to_quaternion(T[0:3, 0:3])
    msg.orientation = np_to_quaternion_msg(q)
    return msg


def gradient5_to_np(g):
    """
    Converts from a 5D gradient vector to a numpy array

    Args:
        g (mag_msgs.Gradient5Vector)
    Returns:
        np.ndarray: converted vector
    """
    return np.array([g.xx, g.xy, g.xz, g.yy, g.yz])


def np_to_gradient5(g):
    """
    Converts a 5D gradient vector from a numpy array

    Args:
        g (np.ndarray): vector of size 5
    Returns:
        mag_msgs.Gradient5Vector: converted vector
    """
    return Gradient5Vector(g[0], g[1], g[2], g[3], g[4])


def np_to_colorrgba(c):
    """
    Converts to a std_msgs.msg.ColorRGBA from a numpy array

    Args:
        c (np.ndarray): vector of size 4
    Returns:
        std_msgs.msgs.ColorRGBA: converted color
    """
    return ColorRGBA(c[0], c[1], c[2], c[3])


def colorrgba_to_np(c):
    """
    Converts from a std_msgs.msg.ColorRGBA to a numpy array

    Args:
        c (std_msgs.msg.ColorRGBA): color to convert
    Returns:
        np.ndarray of size 4 with R,G,B,A
    """
    return np.array([c.r, c.g, c.b, c.a])


###############################
# Converts to/from homogenous #
###############################
def to_homogeneous(P):
    """
    Converts an array of 3D positions to homogoneous coordinates

    Args:
        P (np.ndarray): a Nx3 numpy array of positions
    Returns:
        np.ndarray: a Nx4 array where positions are in [x, y, z, 1]^T homogenous form
    """
    Ph = np.ones((P.shape[0], 4), np.float)
    Ph[:, 0:3] = P
    return Ph


def from_homogeneous(Ph):
    """
    Converts an array of homogeonous positions to 3D coordinates

    Args:
        P (np.ndarray): a Nx4 numpy array of positions in [x, y, z, 1]^T form
    Returns:
        np.ndarray: a Nx3 numpy array where positions are in [x, y, z]^T
    """
    return Ph[:, 0:3]
