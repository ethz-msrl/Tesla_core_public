//
// Tesla - A ROS-based framework for performing magnetic manipulation
//
// Software License Agreement (BSD License)
//
// ©2022 ETH Zurich, D-​MAVT; Multi-Scale Robotics Lab (MSRL) ; Prof Bradley J. Nelson
// All rights reserved.
//
// Redistribution and use of this software in source and binary forms,
// with or without modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above
//   copyright notice, this list of conditions and the
//   following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * All advertising materials mentioning features or use of this software
//   must display the following acknowledgement:
//   “This product includes software developed by the Multi-Scale Robotics Lab, ETH Zurich,
//   Switzerland and its contributors.”
//
// * Neither the name of MSRL nor the names of its
//   contributors may be used to endorse or promote products
//   derived from this software without specific prior
//   written permission of MSRL.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
// IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
// FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
// WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
// WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include "mag_manip/emns_parameters_rosparam.h"
#include <ros/param.h>
#include <vector>
#include "mag_manip/exceptions.h"

using namespace std;
using namespace mag_manip;

EMNSParametersRosparam::EMNSParametersRosparam(ros::NodeHandle& nh) : nh_(nh) {}

string EMNSParametersRosparam::getSystemName() const {
  string system_name;
  if (nh_.getParam("system_name", system_name))
    return system_name;
  else
    throw InvalidCalibration("system_name parameter not found");
}

int EMNSParametersRosparam::getNumElectromagnets() const {
  int num_electromagnets;
  if (!nh_.getParam("num_electromagnets", num_electromagnets))
    throw InvalidCalibration("num_electromagnets parameter not found");

  if (num_electromagnets < 0) {
    throw InvalidCalibration("num_electromagnets cannot be negative");
  }
  return num_electromagnets;
}

const CurrentsVec EMNSParametersRosparam::getMaxCurrents() const {
  vector<double> v_max_currents;
  if (!nh_.getParam("max_currents", v_max_currents))
    throw InvalidCalibration("max_currents parameter not found");

  if (v_max_currents.size() != getNumElectromagnets()) {
    throw InvalidCalibration("Size of max_currents does not match num_electromagnets");
  }

  CurrentsVec max_currents = Eigen::Map<CurrentsVec>(v_max_currents.data(), getNumElectromagnets());
  if ((max_currents.array() < 0).any()) {
    throw InvalidCalibration("max_currents cannot have negative elements");
  }
  return max_currents;
}

double EMNSParametersRosparam::getMaxPower() const {
  double max_power;
  if (!nh_.getParam("max_power", max_power)) {
    throw InvalidCalibration("max_power parameter not found");
  }

  if (max_power < 0) {
    throw InvalidCalibration("max_power cannot be negative");
  }

  return max_power;
}

const ResistancesVec EMNSParametersRosparam::getCoilResistances() const {
  vector<double> v_coil_res;
  if (!nh_.getParam("coil_resistances", v_coil_res))
    throw InvalidCalibration("coil_resistances parameter not found");

  if (v_coil_res.size() != getNumElectromagnets()) {
    throw InvalidCalibration("Size of coil_resistances does not match num_electromagnets");
  }

  ResistancesVec coil_res = Eigen::Map<CurrentsVec>(v_coil_res.data(), getNumElectromagnets());
  if ((coil_res.array() < 0).any()) {
    throw InvalidCalibration("coil_resistances cannot have negative elements");
  }
  return coil_res;
}
