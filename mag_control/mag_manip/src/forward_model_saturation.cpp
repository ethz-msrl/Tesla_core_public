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

#include "mag_manip/forward_model_saturation.h"
#include "mag_manip/forward_model_factory.h"
#include "mag_manip/saturation.h"
#include "mag_manip/saturation_function_factory.h"
#include "mag_manip/utils.h"

#include "yaml-cpp/yaml.h"

using namespace std;

namespace mag_manip {

bool ForwardModelSaturation::isValid() const {
  if (p_for_model_ == nullptr || sat_functions_.size() == 0)
    return false;
  else
    return p_for_model_->isValid();
}
int ForwardModelSaturation::getNumCoils() const {
  if (p_for_model_ == nullptr) {
    throw InvalidCalibration("Linear model not set");
  } else {
    return p_for_model_->getNumCoils();
  }
}

string ForwardModelSaturation::getName() const { return cal_name_; }

void ForwardModelSaturation::setName(const std::string& name) { cal_name_ = name; }

ForwardModel::Ptr ForwardModelSaturation::getModel() const { return p_for_model_; }

/**
 * @brief Sets the forward model.
 *
 * WARNING: Must be done before computing.
 *
 * @param p_for_model
 */
void ForwardModelSaturation::setModel(ForwardModel::Ptr p_for_model) { p_for_model_ = p_for_model; }

SaturationFunction::Ptr ForwardModelSaturation::getSaturationFunction(const int i) const {
  return sat_functions_[i];
}

vector<SaturationFunction::Ptr> ForwardModelSaturation::getSaturationFunctions() const {
  return sat_functions_;
}

void ForwardModelSaturation::setCalibrationFile(const std::string& filename) {
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

  if (cal_type != "forward_model_saturation") {
    throw InvalidCalibration("Invalid calibration type: " + cal_type +
                             ". Should be forward_model_saturation.");
  }

  YAML::Node n_for_model = config["forward_model"];

  if (!n_for_model.IsMap()) {
    throw InvalidCalibration("Invalid map forward_model");
  }

  string for_model_type;
  try {
    for_model_type = n_for_model["type"].as<string>();
  } catch (YAML::Exception& e) {
    throw InvalidCalibration("Unable to set model type.");
  }

  string for_model_fn;

  try {
    for_model_fn = n_for_model["filename"].as<string>();
  } catch (YAML::Exception& e) {
    throw InvalidCalibration("Unable to set model filename.");
  }

  auto fact = ForwardModelFactory();
  p_for_model_ = fact.create(for_model_type, pathAppend(parent_dir, for_model_fn));

  string sat_functions_fn;
  try {
    sat_functions_fn = config["saturations_filename"].as<string>();
  } catch (YAML::Exception& e) {
    throw InvalidCalibration("Unable to set the saturations filename.");
  }

  setSaturationFunctionsFile(pathAppend(parent_dir, sat_functions_fn));
}

void ForwardModelSaturation::setModelCalibrationFile(const std::string& filename) {
  if (p_for_model_ == nullptr) {
    throw InvalidCalibration("Linear model not set");
  } else {
    p_for_model_->setCalibrationFile(filename);
  }
}

void ForwardModelSaturation::setSaturationFunctionsFile(const std::string& filename) {
  sat_functions_.clear();
  sat_functions_ = saturationFunctionsFromFile(filename);

  if (sat_functions_.size() != getNumCoils()) {
    throw InvalidFile(filename, "number of saturation functions does not match number of coils");
  }
}

void ForwardModelSaturation::setSaturationFunctions(
    std::vector<SaturationFunction::Ptr> sat_functions) {
  sat_functions_.clear();

  for (auto& p_sat_fun : sat_functions) {
    sat_functions_.push_back(p_sat_fun);
  }
}

FieldVec ForwardModelSaturation::computeFieldFromCurrents(const PositionVec& position,
                                                          const CurrentsVec& currents) const {
  assert(sat_functions_.size() == getNumCoils());

  CurrentsVec currents_s(getNumCoils());
  for (int i = 0; i < currents.size(); i++) {
    currents_s(i) = sat_functions_[i]->evaluate(currents(i));
  }

  return p_for_model_->computeFieldFromCurrents(position, currents_s);
}

Gradient5Vec ForwardModelSaturation::computeGradient5FromCurrents(
    const PositionVec& position, const CurrentsVec& currents) const {
  assert(sat_functions_.size() == getNumCoils());

  CurrentsVec currents_s(getNumCoils());
  for (int i = 0; i < currents.size(); i++) {
    currents_s(i) = sat_functions_[i]->evaluate(currents(i));
  }

  return p_for_model_->computeGradient5FromCurrents(position, currents_s);
}

FieldGradient5Vec ForwardModelSaturation::computeFieldGradient5FromCurrents(
    const PositionVec& position, const CurrentsVec& currents) const {
  FieldVec b = computeFieldFromCurrents(position, currents);
  Gradient5Vec g = computeGradient5FromCurrents(position, currents);
  FieldGradient5Vec bg;
  bg << b, g;
  return bg;
}

void ForwardModelSaturation::setCachedPosition(const PositionVec& position) {
  p_for_model_->setCachedPosition(position);
}

PositionVec ForwardModelSaturation::getCachedPosition() const {
  return p_for_model_->getCachedPosition();
}

FieldVec ForwardModelSaturation::computeFieldFromCurrentsCached(const CurrentsVec& currents) const {
  assert(sat_functions_.size() == getNumCoils());

  CurrentsVec currents_s(getNumCoils());
  for (int i = 0; i < currents.size(); i++) {
    currents_s(i) = sat_functions_[i]->evaluate(currents(i));
  }

  return p_for_model_->computeFieldFromCurrentsCached(currents_s);
}

Gradient5Vec ForwardModelSaturation::computeGradient5FromCurrentsCached(
    const CurrentsVec& currents) const {
  assert(sat_functions_.size() == getNumCoils());

  CurrentsVec currents_s(getNumCoils());
  for (int i = 0; i < currents.size(); i++) {
    currents_s(i) = sat_functions_[i]->evaluate(currents(i));
  }

  return p_for_model_->computeGradient5FromCurrentsCached(currents_s);
}

FieldGradient5Vec ForwardModelSaturation::computeFieldGradient5FromCurrentsCached(
    const CurrentsVec& currents) const {
  FieldVec b = computeFieldFromCurrentsCached(currents);
  Gradient5Vec g = computeGradient5FromCurrentsCached(currents);
  FieldGradient5Vec bg;
  bg << b, g;
  return bg;
}
}  // namespace mag_manip
