#! /usr/bin/env python
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

import sys

import mpem
import numpy as np
import yaml

if __name__ == "__main__":
    # Run with as a single argument, the path to the MPEM YAML calibration file
    if len(sys.argv) != 2:
        raise ValueError("usage: generate_rbf_from_mpem.py <path to MPEM cal file>")
    cal = mpem.ElectromagnetCalibration(sys.argv[1])

    # the number of grid points per side
    ng = 5
    # the size of the workspace cube in m
    ws_size = 0.005
    xv = np.linspace(-ws_size, ws_size, 5)
    yv = np.linspace(-ws_size, ws_size, 5)
    zv = np.linspace(-ws_size, ws_size, 5)

    xg, yg, zg = np.meshgrid(xv, yv, zv)

    Ne = cal.getNumberOfCoils()
    coil_names = ["coil_{:d}".format(c) for c in range(Ne)]
    d = {"name": cal.getName(), "vfields": {}}

    for c, coil in enumerate(coil_names):
        d_coil = {"kernel": "gaussian", "shape_param": 20.0, "nodes": []}
        for x, y, z in zip(xg.ravel(), yg.ravel(), zg.ravel()):
            p = np.array([x, y, z])
            currents = np.zeros((Ne,), np.float64)
            currents[c] = 1.0
            field = cal.fieldAtPoint(currents, p)
            d_coil["nodes"].append(
                {"position": p.tolist(), "value": np.squeeze(field).tolist()}
            )
        d["vfields"][coil] = d_coil
    with open("rbf_out.yaml", "w") as f:
        f.write(yaml.dump(d))
