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

from __future__ import print_function

from builtins import range

import numpy as np
from numpy.testing import assert_allclose
import numpy.random

from tsc_utils.mag_utils import (generate_field_step_changes, grad5_to_grad33,
                                 write_step_changes)

rs = numpy.random.RandomState(12345)


def test_grad5_to_grad33():
    x = [0.1, 0.2, 0.3, 0.4, 0.5]
    m = grad5_to_grad33(x)
    m_ = np.array([[0.1, 0.2, 0.3], [0.2, 0.4, 0.5], [0.3, 0.5, -0.5]])
    assert np.allclose(m, m_)


def test_grad5_array_to_grad33_array():
    x = rs.rand(10, 5)
    m = np.zeros((x.shape[0], 3, 3), np.float)

    for i in range(x.shape[0]):
        m[i, :, :] = grad5_to_grad33(x[i, :])

    assert_allclose(m, grad5_to_grad33(x))


def test_generate_field_step_changes():
    field_mags = [30e-3, 40e-3, 50e-3]
    print(generate_field_step_changes(field_mags, 3))


def test_write_step_changes():
    field_mags = [30e-3, 40e-3, 50e-3]
    steps = generate_field_step_changes(field_mags, 3)
    write_step_changes("test.txt", steps, 10.0)
