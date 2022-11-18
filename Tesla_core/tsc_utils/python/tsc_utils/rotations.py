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

"""
Some helper functions for 3D Rotations

Notes:
    * We use Hamilton quaternions (same as ROS, and Eigen) and use the same ordering as ROS: q = [x, y, z, w]
"""

import numpy as np


def null(a, rtol=1e-5):
    """
    Returns the rank and null space of a matrix

    Args:
        a (np.ndarray): numpy array to get null space of
        rtol (float): optional param specifying the tolerance for a singular value being considered 0

    Returns:
        (tuple):
            int: the integer rank of matrix a
            np.ndarray: a numpy array containing the null space of a
    """
    u, s, v = np.linalg.svd(a)
    rank = (s > rtol * s[0]).sum()
    return rank, v[rank:].T.copy()


def skew(v):
    """
    Returns the skew-symmetric matrix associated with 3x1 vector v

    Args:
        v (np.ndarray): 3x1 numpy array

    Returns:
        np.ndarray: 3x3 array containing the skew symmetric matrix associated with v

    """
    assert v.shape == (3,)

    return np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])


def unskew(M):
    """
    Returns the vector associated with a 3x3 skew-symmetric matrix M

    Args:
        M (np.ndarray): 3x3 skew-symmetric numpy array

    Returns:
        np.ndarray: 3x1 numpy array corresponding to the vector form of the skew-symmetric matrix M
    """

    assert M.shape == (3, 3)

    if not np.allclose(M.transpose(), -M):
        return ValueError("M is not skew-symmetric")

    return np.array([M[2, 1], M[0, 2], M[1, 0]])


def wrap_to_2pi(angles):
    """
    Converts angles from the [-pi,pi] representation to [0, 2*pi]

    Args:
        angles (np.ndarray) of angles in radians
    Returns
        np.ndarray of angles in range 0 to 2*pi
    """
    return (angles + 2 * np.pi) % (2 * np.pi)


def euler_rodrigues(axis, theta):
    """
    Computes the rotation matrix of rotation of theta radians about 3D rotation axis using the Euler-Rodriguez formula

    Args:
        axis (np.ndarray): 3x1 numpy array (unnormalized) rotation axis
        angle (float): float rotation angle in radians

    Returns:
        np.ndarray: 3x3 numpy array rotation matrix
    """

    axis = np.asarray(axis)
    theta = np.asarray(theta)
    axis = axis / np.sqrt(np.dot(axis, axis))
    a = np.cos(theta / 2.0)
    b, c, d = -axis * np.sin(theta / 2.0)
    aa, bb, cc, dd = a * a, b * b, c * c, d * d
    bc, ad, ac, ab, bd, cd = b * c, a * d, a * c, a * b, b * d, c * d
    return np.array(
        [
            [aa + bb - cc - dd, 2 * (bc + ad), 2 * (bd - ac)],
            [2 * (bc - ad), aa + cc - bb - dd, 2 * (cd + ab)],
            [2 * (bd + ac), 2 * (cd - ab), aa + dd - bb - cc],
        ]
    )


def rotation_matrix_to_axis_angle(R):
    """
    Convert a 3x3 rotation matrix to axis angle representation

    Args:
        R (np.ndarray): rotation matrix (3x3 numpy array)

    Returns:
        tuple: The axis (3x1 numpy array) and angle in radians corresponding
        to rotation matrix R
    """

    angle = np.arccos((np.trace(R) - 1) / 2.0)

    axis = (1 / (2 * np.sin(angle))) * np.array(
        [R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]]
    )
    return axis, angle


def rotation_matrix_to_quaternion(R):
    """
    Convert a 3x3 rotation matrix to quaternion representation

    Args:
        R (np.ndarray): rotation matrix (3x3 numpy array)
    Returns:
        np.ndarray: quaternion as 4d numpy array [x,y,z,w]
    """
    assert is_rotation_matrix(R), "R is not a rotation matrix"

    w2_a = 1 + R[0, 0] + R[1, 1] + R[2, 2]
    w2_b = 1 + R[0, 0] - R[1, 1] - R[2, 2]
    w2_c = 1 - R[0, 0] + R[1, 1] - R[2, 2]
    w2_d = 1 - R[0, 0] - R[1, 1] + R[2, 2]

    w2_m = max([w2_a, w2_b, w2_c, w2_d])

    if w2_m == w2_a:
        w = 0.5 * np.sqrt(w2_a)
        x = (R[2, 1] - R[1, 2]) / (4 * w)
        y = (R[0, 2] - R[2, 0]) / (4 * w)
        z = (R[1, 0] - R[0, 1]) / (4 * w)
    elif w2_m == w2_b:
        x = 0.5 * np.sqrt(w2_b)
        y = (R[0, 1] + R[1, 0]) / (4 * x)
        z = (R[0, 2] + R[2, 0]) / (4 * x)
        w = (R[2, 1] - R[1, 2]) / (4 * x)
    elif w2_m == w2_c:
        y = 0.5 * np.sqrt(w2_c)
        x = (R[1, 0] + R[0, 1]) / (4 * y)
        z = (R[1, 2] + R[2, 1]) / (4 * y)
        w = (R[0, 2] - R[2, 0]) / (4 * y)
    else:
        z = 0.5 * np.sqrt(w2_d)
        x = (R[2, 0] + R[0, 2]) / (4 * z)
        y = (R[2, 1] + R[1, 2]) / (4 * z)
        w = (R[1, 0] - R[0, 1]) / (4 * z)

    return np.array([x, y, z, w])


def exp_SO3(wh):
    """
    Exponential map from so(3) Lie algebra to SO(3) Lie group

    Args:
        wh: skew-symmetric matrix

    Returns:
        rotation matrix corresponding to exp(wh)
    """

    w = unskew(wh)
    th = np.linalg.norm(w)
    ax = w / th

    return euler_rodrigues(ax, th)


def log_SO3(R):
    """
    Logarithm map from SO(3) Lie group to so(3) Lie algebra

    Args:
        R (np.ndarray): rotation matrix (3x3 numpy array)

    Returns:
        np.ndarray: skew symmetric matrix (3x3 numpy array) that is log(R)
    """

    axis, angle = rotation_matrix_to_axis_angle(R)
    w = axis * angle

    return skew(w)


def quaternion_is_equal(q, q_, **kwargs):
    """
    Returns true if both quaternions are equal

    See numpy.allclose for tolerance arguments

    Args:
        q: np.ndarray of length 4
        q_: np.ndarray of length 4
        rtol (float) relative tolerance
        atol (float) absolute tolerance.
    Returns
        true if the quaternions are equal
    """
    return np.allclose(q, q_, **kwargs) or np.allclose(q, -q_, **kwargs)


def quaternion_inverse(q):
    """
    Returns the inverse of q
    """
    return np.array([-q[0], -q[1], -q[2], q[3]])


def quaternion_multiply(q_, q):
    """
    Returns the Hamilton product of q and q_
    """
    return np.array(
        [
            q[3] * q_[0] + q[2] * q_[1] - q[1] * q_[2] + q[0] * q_[3],
            -q[2] * q_[0] + q[3] * q_[1] + q[0] * q_[2] + q[1] * q_[3],
            q[1] * q_[0] - q[0] * q_[1] + q[3] * q_[2] + q[2] * q_[3],
            -q[0] * q_[0] - q[1] * q_[1] - q[2] * q_[2] + q[3] * q_[3],
        ]
    )


def quaternion_to_axis_angle(q):
    """
    converts quaternion q = [x, y, z, w] to angle-axis representation

    Args:
        q (np.ndarray): [x, y, z, w] vector (3x1 numpy array)

    Returns:
        tuple: axis (3x1 numpy array), angle (radians)
    """
    angle = 2 * np.arctan2(np.linalg.norm(q[0:3]), q[3])

    if angle < 1e-6:
        axis = np.zeros((3,), np.float)
    else:
        axis = (1.0 / np.sin(angle / 2)) * q[0:3]

    return axis, angle


def axis_angle_to_quaternion(axis, angle):
    """
    Converts a rotation of angle about axis to quaternion

    Args:
        axis: 3x1 numpy array containing axis (should be normalized)
        angle: rotation angle in radians
    Returns:
        4x1 numpy array: [q.x q.y q.z q.w]
    """

    w = np.atleast_1d(np.cos(angle / 2))
    v = axis * np.sin(angle / 2)

    return np.concatenate((v, w))


def quaternion_from_two_vectors(a, b):
    """
    Calculates the quaternion rotating vector a into vector b

    Args:
        a (np.ndarray): 3x1 numpy array
        b (np.ndarray): 3x1 numpy array
    Returns:
        np.ndarray: 4x1 numpy array: [q.x q.y q.z q.w]
    """

    na = np.linalg.norm(a)
    a_n = a / na
    nb = np.linalg.norm(b)
    b_n = b / nb

    cos_th = np.dot(a_n, b_n)

    # we need to check the case where a and b are opposite
    if np.isclose(cos_th, -1.0, rtol=1e-9):
        A = np.concatenate((a_n[np.newaxis, :], b_n[np.newaxis, :]), axis=0)
        U, S, V = np.linalg.svd(A)
        ax = V[2, :]
        v = ax * 1.0
        return np.append(v, 0.0)
    else:
        half_cos = np.sqrt(0.5 * (1.0 + cos_th))
        v = np.cross(a_n, b_n) / np.sqrt((1 + cos_th) * 2.0)
        return np.append(v, half_cos)


def quaternion_xyzw_to_wxyz(q):
    """
    Converts a quaternion vector from the [x, y, z, w]^T ordering to
    the [w, x, y, z]^T ordering (used in mag_rod_solver)

    Args:
        q (np.ndarray): a quaternion in [x, y , z, w]^T ordering
    Returns:
        np.ndarray: q converted to [w, x, y, z]^T

    """
    qn = np.zeros((4,), np.float)
    qn[0] = q[3]
    qn[1:] = q[0:3]
    return qn


def quaternion_wxyz_to_xyzw(q):
    """
    Converts a quaternion vector from the [w, x, y, z]^T ordering to
    the [x, y, z, w]^T ordering (used here)

    Args:
        q (np.ndarray): a quaternion in [w, x , y, z]^T ordering
    Returns:
        np.ndarray q converted to [x, y, z, w]^T
    """
    qn = np.zeros((4,), np.float)
    qn[3] = q[0]
    qn[0:3] = q[1:]
    return qn


def quaternion_to_rotation_matrix(q):
    """
    Returns a 3x3 rotation matrix associated with quaternion

    Args:
        q (np.ndarray): 4x1 quaternion of form [x, y, z, w]
    returns:
        np.ndarray: 3x3 rotation matrix
    """
    return np.array(
        [
            [
                1 - 2 * (q[1] ** 2 + q[2] ** 2),
                2 * (q[0] * q[1] - q[2] * q[3]),
                2 * (q[0] * q[2] + q[1] * q[3]),
            ],
            [
                2 * (q[0] * q[1] + q[2] * q[3]),
                1 - 2 * (q[0] ** 2 + q[2] ** 2),
                2 * (q[1] * q[2] - q[0] * q[3]),
            ],
            [
                2 * (q[0] * q[2] - q[1] * q[3]),
                2 * (q[1] * q[2] + q[0] * q[3]),
                1 - 2 * (q[0] ** 2 + q[1] ** 2),
            ],
        ]
    )


def pose_6D_to_matrix(w, t):
    """
    Returns a 4x4 transformation matrix from a 3D rotation vector and translation vector

    Args:
        w (np.ndarray): rotation matrix as 3D vector representation of a Lie algebra (axis-angle) representation
        t (np.ndarray): 3D translation matrix
    Returns:
        np.ndarray: 4x4 transformation matrix
    """

    T = np.eye(4)
    th = np.linalg.norm(w)
    ax = w / th

    T[0:3, 0:3] = euler_rodrigues(ax, th)
    T[0:3, 3] = t
    return T


def rotate_vector(axis, angles, v):
    """
    Rotates 3x1 vector v n times to form a nx3 matrix.

    Args:
        axis (np.ndarray): a 3x1 vector representing the axis of the rotations
        angles (np.ndarray): a nx1 vector representing all the rotation angles in radians
        v (np.ndarray): a 3x1 vector to be rotated

    Returns:
        np.ndarray: a nx3 matrix containing all the rotated vectors

    """

    cost = np.cos(angles)
    sint = np.sin(angles)
    a = v
    b = skew(axis).dot(v)
    c = np.outer(axis, axis).dot(v)
    return np.outer(cost, a) + np.outer(sint, b) + np.outer(1 - cost, c)


def is_rotation_matrix(R):
    """
    Returns true if R is a rotation matrix

    Args:
        R (np.ndarray): rotation matrix (3x3 numpy array)

    Returns:
        bool: True if R is a rotation matrix
    """

    is_3by3 = np.shape(R) == (3, 3)

    Rinv = np.linalg.pinv(R)
    Rt = np.transpose(R)

    return is_3by3 and np.allclose(Rinv, Rt)


def rotx(roll):
    """Returns rotation matrix for rotation of roll degrees about x-axis

    Args:
        roll (float): angle about x-axis in degrees
    Returns:
        np.ndarray: 3x3 rotation matrix
    """

    rotX = np.array(
        [
            [1, 0, 0],
            [0, np.cos(np.deg2rad(roll)), -np.sin(np.deg2rad(roll))],
            [0, np.sin(np.deg2rad(roll)), np.cos(np.deg2rad(roll))],
        ]
    )
    return rotX


def roty(pitch):
    """Returns rotation matrix for rotation of pitch degrees about y-axis

    Args:
        pitch (float): angle about y-axis in degrees
    Returns:
        np.ndarray: 3x3 rotation matrix
    """
    rotY = np.array(
        [
            [np.cos(np.deg2rad(pitch)), 0, np.sin(np.deg2rad(pitch))],
            [0, 1, 0],
            [-np.sin(np.deg2rad(pitch)), 0, np.cos(np.deg2rad(pitch))],
        ]
    )
    return rotY


def rotz(yaw):
    """Returns rotation matrix for rotation of yaw degrees about y-axis

    Args:
        yaw (float): angle about y-axis in degrees
    Returns:
        np.ndarray: 3x3 rotation matrix
    """
    rotZ = np.array(
        [
            [np.cos(np.deg2rad(yaw)), -np.sin(np.deg2rad(yaw)), 0],
            [np.sin(np.deg2rad(yaw)), np.cos(np.deg2rad(yaw)), 0],
            [0, 0, 1],
        ]
    )
    return rotZ


def ex():
    """Returns the [1,0,0] vector

    Returns:
        np.ndarray: the [1,0,0] vector
    """
    return np.array([1.0, 0, 0])


def ey():
    """Returns the [0,1,0] vector

    Returns:
        np.ndarray: the [0,1,0] vector
    """
    return np.array([0, 1.0, 0])


def ez():
    """Returns the [0,0,1] vector

    Returns:
        np.ndarray: the [0,0,1] vector
    """
    return np.array([0, 0, 1.0])


def quaternion_rotate_vector(q, v):
    """
    Rotates a vector by a quaternion

    Args:
        * q (np.ndarray): [x, y, z, w]^T unit quaternion as numpy.ndarray
    Returns:
        np.ndarray: rotated vector as 3x1 numpy.pndarray
    """
    w = q[3]
    u = q[0:3]
    return 2 * np.dot(u, v) * u + (w ** 2 - np.dot(u, u)) * v + 2 * w * np.cross(u, v)


def jac_normalized_quaternion(q):
    """Returns the Jacobian associated with the normalization operation
    of the quaternion

    f(q) = q / ||q||

    Args:
        * q: [x, y, z, w]^T unit quaternion as numpy.ndarray
    Returns:
        4x4 numpy.ndarray of df(q)/dq
    """
    qn = np.linalg.norm(q)
    return np.eye(4) / qn - np.outer(q, q) / qn ** 3


def jac_tangent_quaternion(q):
    """
    Returns the Jacobian relating changes in quaternion to tangent vector

    Args:
        * q (np.ndarray): [x, y, z, w]^T unit quaternion as numpy.ndarray
    Returns:
        np.ndarray: 3x4 matrix dT/dq as numpy.ndarray
    """
    w = q[3]
    v = q[0:3]
    z = ez()
    dT_dw = 2 * w * z + 2 * np.cross(v, z)
    dT_dv = (
        2 * np.outer(v, z)
        + 2 * np.eye(3) * np.dot(z, v)
        - 2 * np.outer(z, v)
        - 2 * np.dot(w, skew(z))
    )
    dT_dq = np.concatenate((dT_dv, dT_dw[:, np.newaxis]), axis=1)
    dqn_dq = jac_normalized_quaternion(q)
    return dT_dq.dot(dqn_dq)


def jac_rotated_vector(v, u):
    """
    Returns the Jacobian relating changes in the exponential coordinates v
    to the rotated vector R(v) * u = exp(skew(v)) * u
    d(R(v) * u) / dv

    From "A Compact Formula for the Derivative of a 3-D Rotation
    in Exponential Coordinates" by Gallego et. al

    Args:
        v (np.ndarray): 3D numpy array containing rotation in exponential coordinates
        u (np.ndarray): 3D vector to be rotated
    Returns:
        np.ndarray: 3x3 matrix of d(R(v) * u)/dv
    """
    # angle of rotation
    vn = np.linalg.norm(v)
    # axis of rotation
    ax = v / vn
    R = euler_rodrigues(ax, vn)
    i_vn2 = 1 / (vn * vn)
    I3 = np.eye(3)
    return np.dot(-R, skew(u)).dot((np.outer(v, v) + (R.T - I3).dot(skew(v))) * i_vn2)
