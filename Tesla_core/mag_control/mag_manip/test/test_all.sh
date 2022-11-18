#! /bin/bash
#
# Tesla - A ROS-based framework for performing magnetic manipulation
#
# Copyright 2018 Multi Scale Robotics Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#

IFS=":" read -ra DIR_ARR <<< "$CMAKE_PREFIX_PATH"

for i in "${DIR_ARR[@]}"; do
    # Check if path contains a devel folder
    if grep -q 'devel' <<< "$i"; then
        # We need to check if the path has a mag_manip folder
        if [ -d "$i/lib/mag_manip" ]; then
            mag_manip_path=$i/lib/mag_manip 
            for j in "$mag_manip_path"/test*; do
            echo running "$j"
            "$j"; 
            done
        fi
    fi 
done

