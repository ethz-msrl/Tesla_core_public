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

import re
from argparse import ArgumentParser
from os.path import join

from catkin_pkg.packages import find_package_paths, find_packages


def update_version(manifest_path, version, output=None):
    """
    Updates the version tag in a package.xml file

    Args:
        manifest_path (str): path to the package.xml file to change
        version (str): new version number (eg. 1.0.0)
        output (str): the path to write the new xml file to. If None,
        does in place
        editing of manifest_path
    """

    regex = re.compile(r"<version>(.+)</version>")
    with open(manifest_path, "r") as file_h:
        data = file_h.read()

    new_manifest = regex.sub("<version>%s</version>" % version, data)

    if not output:
        output = manifest_path

    with open(output, "w") as f:
        f.write(new_manifest)


if __name__ == "__main__":

    parser = ArgumentParser(
        description="Updates the version tags the package.xml files of a catkin workspace"
    )
    parser.add_argument(
        "base_path", help="base path in which to search recursively for catkin packages"
    )
    parser.add_argument("version", help="the version tag to change to (eg. 1.0.0)")
    parser.add_argument(
        "-e", "--exclude-paths", nargs="+", help="optional list of paths to exclude"
    )
    parser.add_argument(
        "-n",
        "--dry-run",
        action="store_true",
        help="does not actually modify the files",
    )

    args = parser.parse_args()

    if args.dry_run:
        packages = find_packages(args.base_path, exclude_paths=args.exclude_paths)
        for key, package in packages.items():
            print("%s would change version %s" % (package.name, package.version))
    else:
        pkg_paths = find_package_paths(args.base_path, exclude_paths=args.exclude_paths)
        for pkg_path in pkg_paths:
            manifest_path = join(args.base_path, pkg_path, "package.xml")
            update_version(manifest_path, args.version)
