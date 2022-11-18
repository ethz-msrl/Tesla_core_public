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

#include "mag_manip/emns_parameters_yaml.h"
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <streambuf>
#include <vector>
#include "mag_manip/exceptions.h"

using namespace mag_manip;
using namespace std;

EMNSParametersYAML::EMNSParametersYAML(const std::string& config_s) { load(config_s); }

string EMNSParametersYAML::getSystemName() const { return system_name_; }

int EMNSParametersYAML::getNumElectromagnets() const { return ne_; }

const CurrentsVec EMNSParametersYAML::getMaxCurrents() const { return max_currents_; }

double EMNSParametersYAML::getMaxPower() const { return max_power_; }

const ResistancesVec EMNSParametersYAML::getCoilResistances() const { return coil_res_; }

void EMNSParametersYAML::load(const std::string& config_s) {
  YAML::Node config;
  try {
    config = YAML::Load(config_s);
  } catch (YAML::Exception& e) {
    throw InvalidCalibration("Unable to load calibration YAML");
  }

  try {
    system_name_ = config["system_name"].as<string>();
  } catch (YAML::Exception& e) {
    throw InvalidCalibration("system_name parameter is invalid");
  }

  try {
    ne_ = config["num_electromagnets"].as<int>();
  } catch (YAML::Exception& e) {
    throw InvalidCalibration("num_electromagnets parameter is invalid");
  }

  if (ne_ < 0) {
    throw InvalidCalibration("num_electromagnets cannot be negative");
  }

  vector<double> v_max_currents;
  try {
    v_max_currents = config["max_currents"].as<vector<double> >();
  } catch (YAML::Exception& e) {
    throw InvalidCalibration("max_currents parameters is invalid");
  }

  if (v_max_currents.size() != ne_) {
    throw InvalidCalibration("Size of max_currents does not match num_electromagnets");
  }

  max_currents_ = Eigen::Map<CurrentsVec>(v_max_currents.data(), ne_);
  if ((max_currents_.array() < 0).any()) {
    throw InvalidCalibration("max_currents cannot have negative elements");
  }

  try {
    max_power_ = config["max_power"].as<double>();
  } catch (YAML::Exception& e) {
    throw InvalidCalibration("max_power param is invalid");
  }

  if (max_power_ < 0) {
    throw InvalidCalibration("max_power cannot be negative");
  }

  vector<double> v_coil_res;
  try {
    v_coil_res = config["coil_resistances"].as<vector<double> >();
  } catch (YAML::Exception& e) {
    throw InvalidCalibration("coil_resistances parameter is invalid");
  }

  if (v_coil_res.size() != ne_) {
    throw InvalidCalibration("Size of coil_resistances does not match num_electromagnets");
  }

  coil_res_ = Eigen::Map<CurrentsVec>(v_coil_res.data(), ne_);
  if ((coil_res_.array() < 0).any()) {
    throw InvalidCalibration("coil_resistances cannot have negative elements");
  }
}

EMNSParametersYAML EMNSParametersYAML::fromFile(const string& filename) {
  std::ifstream t(filename);
  if (!t.good()) {
    throw InvalidFile(filename, "Unable to open with ifstream");
  }

  std::string config_s((std::istreambuf_iterator<char>(t)), std::istreambuf_iterator<char>());
  return EMNSParametersYAML(config_s);
}

EMNSParametersYAML::Ptr EMNSParametersYAML::ptrFromFile(const string& filename) {
  std::ifstream t(filename);
  if (!t.good()) {
    throw InvalidFile(filename, "Unable to open with ifstream");
  }

  std::string config_s((std::istreambuf_iterator<char>(t)), std::istreambuf_iterator<char>());
  return make_shared<EMNSParametersYAML>(config_s);
}
