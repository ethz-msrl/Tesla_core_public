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

from __future__ import print_function

import os.path
from argparse import ArgumentParser
from glob import glob
from os.path import join

import pandas as pd
from catkin_pkg.packages import find_packages

if __name__ == "__main__":

    parser = ArgumentParser(
        description="Updates the version tags the package.xml files of a catkin workspace"
    )
    parser.add_argument(
        "base_path", help="base path in which to search recursively for catkin packages"
    )
    parser.add_argument(
        "-e", "--exclude-paths", nargs="+", help="optional list of paths to exclude"
    )
    parser.add_argument("-o", "--output_path", help="optional path to save a csv to")

    args = parser.parse_args()

    pkgs = find_packages(args.base_path, exclude_paths=args.exclude_paths)
    pkgs_with_python = {
        pkg_path: pkg
        for pkg_path, pkg in list(pkgs.items())
        if glob(join(args.base_path, pkg_path, "**/*.py"))
    }

    maintainers = [pkg.maintainers[0].name for pkg in list(pkgs_with_python.values())]
    pkg_names = [
        os.path.split(pkg_name)[-1] for pkg_name in list(pkgs_with_python.keys())
    ]

    df_python_pkgs = pd.DataFrame.from_dict(
        {"Name": pkg_names, "Maintainer": maintainers}
    )

    print(df_python_pkgs)

    if args.output_path:
        df_python_pkgs.to_csv(args.output_path, encoding="utf8", index=False)
