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

#include "mag_manip/emns_parameters_conversions.h"
#include <yaml-cpp/yaml.h>

using namespace std;
namespace mag_manip {
std::string eMNSParametersToYAML(const EMNSParameters& params) {
  YAML::Emitter out;
  out << YAML::BeginMap;
  out << YAML::Key << "system_name";
  out << YAML::Value << params.getSystemName();
  out << YAML::Key << "num_electromagnets";
  out << YAML::Value << params.getNumElectromagnets();
  out << YAML::Key << "max_currents";
  CurrentsVec max_currents = params.getMaxCurrents();
  vector<double> v_max_currents(max_currents.data(),
                                max_currents.data() + max_currents.rows() * max_currents.cols());
  out << YAML::Value << v_max_currents;
  out << YAML::Key << "max_power";
  out << YAML::Value << params.getMaxPower();
  out << YAML::Key << "coil_resistances";
  ResistancesVec coil_res = params.getCoilResistances();
  vector<double> v_coil_res(coil_res.data(), coil_res.data() + coil_res.rows() * coil_res.cols());
  out << YAML::Value << v_coil_res;
  return out.c_str();
}

void eMNSParametersToRosparam(const EMNSParameters& params, ros::NodeHandle& nh) {
  nh.setParam("system_name", params.getSystemName());
  nh.setParam("num_electromagnets", params.getNumElectromagnets());
  CurrentsVec max_currents = params.getMaxCurrents();
  vector<double> v_max_currents(max_currents.data(),
                                max_currents.data() + max_currents.rows() * max_currents.cols());
  nh.setParam("max_currents", v_max_currents);
  nh.setParam("max_power", params.getMaxPower());
  ResistancesVec coil_res = params.getCoilResistances();
  vector<double> v_coil_res(coil_res.data(), coil_res.data() + coil_res.rows() * coil_res.cols());
  nh.setParam("coil_resistances", v_coil_res);
}
}  // namespace mag_manip
