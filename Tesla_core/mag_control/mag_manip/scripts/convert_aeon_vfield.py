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

import argparse
import glob
import os
from math import floor

import numpy as np
import yaml


class VFieldGridProperties:
    def __init__(self):
        self.dim_x = 0
        self.dim_y = 0
        self.dim_z = 0
        self.min_x = 0.0
        self.max_x = 0.0
        self.min_y = 0.0
        self.max_y = 0.0
        self.min_z = 0.0
        self.max_z = 0.0

    def get_index(self, x, y, z):
        if (
            x > self.max_x
            or x < self.min_x
            or y > self.max_y
            or y < self.min_y
            or z > self.max_z
            or z < self.min_z
        ):
            raise ValueError("position not in bounds")

        step = (self.max_x - self.min_x) / (self.dim_x - 1)
        xf = int(floor((x - self.min_x) / step))
        yf = int(floor((y - self.min_y) / step))
        zf = int(floor((z - self.min_z) / step))

        return xf * self.dim_y * self.dim_z + yf * self.dim_z + zf


def parse_vfield_file(filename):
    props = VFieldGridProperties()
    # props = namedtuple('VFieldGridProperties', 'dim_x, dim_y, dim_z, \
    #        min_x, max_x, min_y, max_y, min_z, max_z')
    with open(filename, "r") as f:
        line = f.readline()
        line_l = line.split()
        props.dim_x, props.dim_y, props.dim_z = (int(line) for line in line_l[0:3])
        line = f.readline()
        props.min_x, props.max_x = (float(line) for line in line.split()[0:3])
        line = f.readline()
        props.min_y, props.max_y = (float(line) for line in line.split()[0:3])
        line = f.readline()
        props.min_z, props.max_z = (float(line) for line in line.split()[0:3])

        N = props.dim_x * props.dim_y * props.dim_z
        data = np.zeros((N, 3), np.float)
        for line in f.readlines():
            line_l = line.split()
            p = [float(line) for line in line_l[0:3]]
            d = [float(line) for line in line_l[3:6]]
            idx = props.get_index(p[0], p[1], p[2])
            data[idx, :] = d

    return data, props


def vfield_to_dict(data, props):
    node = dict()
    node["dim_x"] = props.dim_x
    node["dim_y"] = props.dim_y
    node["dim_z"] = props.dim_z
    node["min_x"] = props.min_x
    node["max_x"] = props.max_x
    node["min_y"] = props.min_y
    node["max_y"] = props.max_y
    node["min_z"] = props.min_z
    node["max_z"] = props.max_z
    node["data"] = data.tolist()
    return node


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Converts folders of Aeon VField calibrations to YAML files that can be used with mag_manip"
    )
    parser.add_argument(
        "input", type=str, help="path to a folder containing the Aeon calibration"
    )
    parser.add_argument("output", type=str, help="path to output YAML file")
    parser.add_argument("--name", default="vfield", help="name of the system")

    args = parser.parse_args()

    if not os.path.isdir(args.input):
        raise IOError("%s is not a folder" % args.input)

    f = open(args.output, "w")

    root = dict()
    root["name"] = args.name

    vfields = dict()
    for i, vfield_file in enumerate(glob.glob(os.path.join(args.input, "*.txt"))):
        print("opening file: ", vfield_file)
        data, props = parse_vfield_file(vfield_file)
        vfields["coil_%d" % i] = vfield_to_dict(data, props)
    root["vfields"] = vfields

    f.write(yaml.dump(root))
    f.close()
