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

from argparse import ArgumentParser
from os import chdir, getcwd, listdir, mkdir, rmdir
from os.path import basename, isdir, isfile, join, exists
from shutil import move, rmtree
from subprocess import call

from catkin_pkg.packages import find_package_paths


def get_doc_type(doc_path):
    if isfile(join(doc_path, "Doxyfile")):
        return "Doxygen"
    if isfile(join(doc_path, "conf.py")):
        return "sphinx"


if __name__ == "__main__":
    parser = ArgumentParser(
        description="Finds the autodocs in catkin packages and generates their documentation"
    )
    parser.add_argument(
        "base_path", help="base path in which to search recursively for catkin packages"
    )
    parser.add_argument("-o", "--output", help="folder in which to generate docs")
    parser.add_argument(
        "-e", "--exclude-paths", nargs="+", help="optional list of paths to exclude"
    )
    parser.add_argument(
        "-n",
        "--dry-run",
        action="store_true",
        help="does not actually modify the files",
    )
    parser.add_argument(
            '-x', '--overwrite', action='store_true', help='Will overwrite an existing directory')

    args = parser.parse_args()

    pkg_paths = find_package_paths(args.base_path, exclude_paths=args.exclude_paths)

    print("finding packages with autodocs in %s\n" % args.base_path)

    if args.dry_run:
        template = "{0:32} | {1:8}"
        print(template.format("PATH", "TYPE"))

    for pkg_path in pkg_paths:
        doc_path = join(args.base_path, pkg_path, "doc")
        if isdir(doc_path):
            if args.dry_run:
                print(template.format(pkg_path, get_doc_type(doc_path)))
            else:
                if args.output:
                    if exists(args.output) and args.overwrite:
                        rmtree(args.output)

                    if not isdir(args.output):
                        mkdir(args.output)

                    chdir(args.output)

                output_dir = join(getcwd(), basename(pkg_path))

                if not exists(output_dir):
                    mkdir(output_dir)

                chdir(output_dir)

                if get_doc_type(doc_path) == "Doxygen":
                    doxygen_path = join(doc_path, "Doxyfile")
                    chdir(doc_path)

                    call(["doxygen", "Doxyfile"])

                    # need to move everything in html directory
                    html_dir = join("html")
                    files = listdir(html_dir)
                    for f in files:
                        move(join(html_dir, f), output_dir)

                    rmdir(html_dir)

                elif get_doc_type(doc_path) == "sphinx":
                    if args.output:
                        chdir(args.output)

                    call(["sphinx-build", doc_path, output_dir])

                chdir(args.base_path)
