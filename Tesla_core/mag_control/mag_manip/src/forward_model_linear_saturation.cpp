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

#include <yaml-cpp/yaml.h>

#include "mag_manip/forward_model_linear_factory.h"
#include "mag_manip/forward_model_linear_saturation.h"
#include "mag_manip/utils.h"

using namespace std;
using namespace mag_manip;

void ForwardModelLinearSaturation::setCalibrationFile(const std::string& filename) {
  YAML::Node config;
  string parent_dir = getFileDirectory(filename);

  if (parent_dir.empty()) {
    throw std::runtime_error("filename has empty parent directory");
  }

  try {
    config = YAML::LoadFile(filename);
  } catch (YAML::BadFile& e) {
    throw InvalidFile(filename, e.what());
  }

  try {
    cal_name_ = config["name"].as<std::string>();
  } catch (YAML::Exception& e) {
    throw InvalidCalibration("Unable to set name. Reason: " + string(e.what()));
  }

  string cal_type;
  try {
    cal_type = config["type"].as<std::string>();
  } catch (YAML::Exception& e) {
    throw InvalidCalibration("Unable to set cal_type. Reason: " + string(e.what()));
  }

  if (cal_type != "forward_model_linear_saturation") {
    throw InvalidCalibration("Invalid calibration type: " + cal_type +
                             ". Should be forward_model_linear_saturation.");
  }

  YAML::Node n_fml_model = config["forward_model_linear"];

  string fml_model_type;

  try {
    fml_model_type = n_fml_model["type"].as<string>();
  } catch (YAML::Exception& e) {
    throw InvalidCalibration("Unable to set forward model type.");
  }

  string fml_model_fn;

  try {
    fml_model_fn = n_fml_model["filename"].as<string>();
  } catch (YAML::Exception& e) {
    throw InvalidCalibration("Unable to set forward model filename.");
  }

  auto fact = ForwardModelLinearFactory();
  setLinearModel(fact.create(fml_model_type, pathAppend(parent_dir, fml_model_fn)));

  string sat_functions_fn;
  try {
    sat_functions_fn = config["saturations_filename"].as<string>();
  } catch (YAML::Exception& e) {
    throw InvalidCalibration("Unable to set the saturations filename.");
  }

  setSaturationFunctionsFile(pathAppend(parent_dir, sat_functions_fn));
}

void ForwardModelLinearSaturation::setLinearModel(ForwardModelLinear::Ptr p_lin_model) {
  p_lin_model_ = p_lin_model;
  p_for_model_ = p_lin_model;
}

ForwardModelLinear::Ptr ForwardModelLinearSaturation::getLinearModel() const {
  return p_lin_model_;
}

void ForwardModelLinearSaturation::setModel(ForwardModel::Ptr p_model) {
  auto p_lin_model = dynamic_pointer_cast<ForwardModelLinear>(p_model);
  if (!p_lin_model) {
    throw std::runtime_error(
        "Error casting ForwardModel to ForwardModelLinear. Make sure your ForwardModel is indeed a "
        "ForwardModelLinear type");
  }

  p_lin_model_ = p_lin_model;
  p_for_model_ = p_model;
}

void ForwardModelLinearSaturation::setCachedPosition(const PositionVec& position) {
  p_lin_model_->setCachedPosition(position);
}

FieldVec ForwardModelLinearSaturation::computeFieldFromCurrentsCached(
    const CurrentsVec& currents) const {
  assert(sat_functions_.size() == getNumCoils());

  CurrentsVec currents_s(getNumCoils());
  for (int i = 0; i < currents.size(); i++) {
    currents_s(i) = sat_functions_[i]->evaluate(currents(i));
  }

  return p_lin_model_->computeFieldFromCurrentsCached(currents_s);
}
