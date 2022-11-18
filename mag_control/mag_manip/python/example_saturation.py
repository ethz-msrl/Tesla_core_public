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

import os.path

import numpy as np
import rospkg
from mag_manip import mag_manip

rp = rospkg.RosPack()
cal_path = os.path.join(rp.get_path("mag_manip"), "test", "OctoMag_Calibration.yaml")

f = mag_manip.ForwardModelFactory()
lin_model = f.create("mpem", cal_path)

saturations = [mag_manip.SaturationTanh(np.array([8.0, 1 / 8.0]))] * 8

model = mag_manip.ForwardModelSaturation()

model.setLinearModel(lin_model)
model.setSaturations(saturations)
