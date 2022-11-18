#! /usr/bin/env python

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


import sys
from os import path

import numpy as np

if __name__ == "__main__":
    np_path = np.__path__[0]
    np_c_include = path.join(np_path, "core/include")

    if not path.exists(np_c_include):
        sys.exit("Numpy C include directory not found")
    else:
        print(np_c_include)
