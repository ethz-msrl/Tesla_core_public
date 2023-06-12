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
Some functions for generating magnetic fields, currents and the like
"""

import numpy as np


def grad5_to_grad33(x):
    """
    Converts an array of shape (N, 5) to the equivalent array of shape (N, 3, 3)
    by converting gradient5 vectors to 3x3 gradient matrices

    Notes: also works with array of shape (5,)

    Args:
        x (np.ndarray) of size (N, 5)
    Returns:
        np.ndarray of size (N, 3, 3)
    """
    if np.array(x).ndim > 1:
        N = x.shape[0]
        m = np.zeros((N, 3, 3), np.float64)
        m[:, 0, 0:3] = x[:, 0:3]
        m[:, 1, 1:3] = x[:, 3:5]
        m[:, 2, 2] = -(m[:, 0, 0] + m[:, 1, 1])
        return np.triu(m, 1) + np.tril(m.transpose((0, 2, 1)))
    else:
        m = np.zeros((3, 3), np.float64)
        m[0, 0:3] = x[0:3]
        m[1, 1:3] = x[3:5]
        m[2, 2] = -(m[0, 0] + m[1, 1])
        return np.triu(m, 1) + np.tril(m.T)


def generate_currents_step_changes(current_vals, repeats, ncoils=8):
    """
    Repeat n step changes on each coil.

    Generates a currents list of the form

    [5, 0, ..., 0]
    [0, 0, ..., 0]
    [5, 0, ..., 0]
    ...
    [0, 5, ..., 0]
    [0, 0, ..., 0]
    ...

    where 5 is the current_val and where each step change is repeated a number of times

    Args:
        current_vals (np.ndarray): list of currents to be applied on each coil in Amps
        repeats (int): number of times to repeat the step changes
        ncoils (int): number of coils in the system
    Returns:
        np.ndarray: array of size (ncoils * 2 * repeats, ncoils) containing current sequence
    """

    N = len(current_vals)
    currents = np.zeros((ncoils * 2 * repeats * N, ncoils), np.float64)
    current_nz = np.kron(current_vals, np.eye(ncoils)).reshape((N * ncoils, ncoils))
    currents[::2] = np.repeat(current_nz, repeats, axis=0)
    return currents


def generate_field_step_changes(field_mags, repeats):
    """
    Repeat n step changes in the fields

    Generates a fields list of the form
    [B1 0 0 ]
    [0 0 0 ]
    ...
    [B2 0 0]
    [0 0 0]
    [0 0 Bn]
    [0 0 0]

    Args:
        field_mag (iterable): list of field magnitudes of length N [B1 ... Bn] in Tesla
        repeats (int): number of times to repeat each step change sequence in the x, y, z directions
    Returns:
        np.ndarray: array of the size (6 * repeats * N, 3) containing the field sequence
    """

    N = len(field_mags)
    fields = np.zeros((6 * repeats * N, 3), np.float64)
    fields[::2] = np.repeat(
        np.kron(field_mags, np.eye(3)).reshape((N * 3, 3)), repeats, axis=0
    )
    return fields


def write_step_changes(filename, step_changes, step_duration):
    """
    Writes a text file containing the step changes as rows with the last column being the step duration

    Args:
        filename (str): the file to write to
        step_changes (np.ndarray): generated step changes
        step_duration (float): the duration to apply the step change for in seconds
    """

    N = step_changes.shape[0]

    data = np.concatenate(
        (step_changes, step_duration * np.ones((N, 1), np.float64)), axis=1
    )
    np.savetxt(filename, data)
