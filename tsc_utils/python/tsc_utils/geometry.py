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


def angle_between(a, b, n):
    """
    Computes the signed angle between a and b with a in b in plane normal to n

    Args:
        a (numpy.ndarray): 3D vector
        b (numpy.ndarray): other 3D vector
        n (numpy.ndarray): 3D vector normal to a and b
    Returns:
        float: signed angle between a and b
    """
    s = np.sign(np.cross(a, b).dot(n))
    # this is to handle the case where a,b are parallel or anti parallel
    if np.linalg.norm(s) < 1e-12:
        return angle_between_vectors(a, b)
    else:
        return s * angle_between_vectors(a, b)


def angle_between_vectors(a, b):
    """
    Numerically stable computation of angle between 3D vectors

    This is more stable than using the old acos(a.b) trick

    From https://www.cs.berkeley.edu/~wkahan/Mindless.pdf

    Args:
        a (numpy.ndarray): 3D vector
        b (numpy.ndarray): other 3D vector
    Returns:
        float: anble between a and b in radians
    """
    na = np.linalg.norm(a)
    nb = np.linalg.norm(b)
    return 2 * np.arctan2(
        np.linalg.norm(nb * a - na * b), np.linalg.norm(nb * a + na * b)
    )


def get_plane_orthogonal_projection_matrix(normal):
    """
    Returns the orthogonal projection matrix P of a plane defined by its normal vector such that

    vp = P * v

    is the projection of vector v onto the plane for any vector v

    Args:
        normal (np.ndarray): an euclidean vector of length n normal to the plane
    Returns:
        np.ndarray of shape n times n with the projection matrix of the plane
    """
    assert normal.ndim == 1
    n = len(normal)
    return np.eye(n) - np.outer(normal, normal) / np.dot(normal, normal)


def get_vector_orthogonal_projection_matrix(normal):
    """
    Returns the orthogonal projection matrix P of a vector such that

    vp = P * v

    is the projection of vector v onto the normal vector for any vector v

    Args:
        normal (np.ndarray): an Euclidean vector of length n
    Returns:
        np.ndarray of shape n times n with the projection matrix onto the vector
    """
    return np.outer(normal, normal) / np.dot(normal, normal)


def project_vector_onto_plane(normal, vector):
    """
    Returns the projection of a vector v onto the plane defined by a normal vector n
    vp = proj_n(v)

    Where vp is the projection of v onto the plane defined by n

    Args:
        * normal (np.ndarray) of length n with the normal vector of the plane
        * vector (np.ndarray) of length n with the vector to be projected

    Returns:
        np.ndarray of length n with the projection of vector onto plane defined by normal
    """
    assert normal.ndim == 1
    assert vector.ndim == 1
    assert len(normal) == len(vector)

    v_cross_n = np.cross(vector, normal)
    return np.cross(normal, v_cross_n) / np.dot(normal, normal)


def project_vector_onto_vector(normal, vector):
    """
    Returns the projection of a vector v onto the vector
    vp = proj_n(v)

    Where vp is the projection of v onto the vector n

    Args:
        * normal (np.ndarray) of length n with the vector on which to project
        * vector (np.ndarray) of length n with the vector to be projected

    Returns:
        np.ndarray of length n with the projection of vector onto normal vector
    """
    assert normal.ndim == 1
    assert vector.ndim == 1
    assert len(normal) == len(vector)

    return np.dot(normal, vector) * normal / np.dot(normal, normal)
