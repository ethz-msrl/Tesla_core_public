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

#include "mag_manip/saturation.h"
#include "mag_manip/exceptions.h"
#include "mag_manip/saturation_function_factory.h"
#include "yaml-cpp/yaml.h"

using namespace std;

namespace mag_manip {
std::vector<SaturationFunction::Ptr> saturationFunctionsFromFile(const std::string& filename) {
  YAML::Node config;

  try {
    config = YAML::LoadFile(filename);
  } catch (YAML::BadFile& e) {
    throw InvalidFile(filename, e.what());
  }

  YAML::Node n_sat_funs;
  try {
    n_sat_funs = config["saturation_functions"];
  } catch (YAML::Exception& e) {
    throw InvalidFile(filename, "saturation_functions field invalid");
  }

  // For some reason yaml-cpp does not sort map keys
  // We do one pass to get the sorted keys and then query the node key by key
  vector<string> coil_keys;
  for (YAML::const_iterator cit = n_sat_funs.begin(); cit != n_sat_funs.end(); cit++) {
    coil_keys.push_back(cit->first.as<string>());
  }

  std::sort(coil_keys.begin(), coil_keys.end());

  vector<SaturationFunction::Ptr> sat_functions;

  SaturationFunctionFactory sat_fact;
  for (string key : coil_keys) {
    YAML::Node n_sat_fun = n_sat_funs[key];
    string sat_type = n_sat_fun["type"].as<string>();
    vector<double> v_params = n_sat_fun["params"].as<vector<double> >();
    auto params = Eigen::Map<Eigen::VectorXd>(v_params.data(), v_params.size());
    sat_functions.push_back(sat_fact.create(sat_type, params));
  }

  return sat_functions;
}
}  // namespace mag_manip
