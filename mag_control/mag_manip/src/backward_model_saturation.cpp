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

#include "mag_manip/backward_model_saturation.h"
#include "mag_manip/backward_model_factory.h"
#include "mag_manip/saturation.h"
#include "mag_manip/saturation_function_factory.h"
#include "mag_manip/utils.h"

#include "yaml-cpp/yaml.h"

using namespace std;

namespace mag_manip {

BackwardModelSaturation::BackwardModelSaturation() : do_check_max_(true) {}

bool BackwardModelSaturation::isValid() const {
  if (p_bac_model_ == nullptr || sat_functions_.size() == 0)
    return false;
  else
    return p_bac_model_->isValid();
}
int BackwardModelSaturation::getNumCoils() const {
  if (p_bac_model_ == nullptr) {
    throw InvalidCalibration("Backward model not set");
  } else {
    return p_bac_model_->getNumCoils();
  }
}

string BackwardModelSaturation::getName() const { return cal_name_; }

void BackwardModelSaturation::setName(const std::string& name) { cal_name_ = name; }

CurrentsVec BackwardModelSaturation::computeCurrentsFromField(const PositionVec& position,
                                                              const FieldVec& field) const {
  assert(sat_functions_.size() == getNumCoils());

  CurrentsVec currents_s = p_bac_model_->computeCurrentsFromField(position, field);
  CurrentsVec currents(getNumCoils());

  // the currents should be higher to compenesate for saturation
  for (int i = 0; i < getNumCoils(); i++) {
    if (do_check_max_) checkMax(sat_functions_[i], currents_s[i]);
    currents[i] = sat_functions_[i]->inverse(currents_s[i]);
  }

  return currents;
}

void BackwardModelSaturation::setDoCheckMax(const bool enable) { do_check_max_ = enable; }

void BackwardModelSaturation::setCalibrationFile(const std::string& filename) {
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
    throw InvalidCalibration("Unable to set calibration type. Reason: " + string(e.what()));
  }

  if (cal_type != "backward_model_saturation") {
    throw InvalidCalibration("Invalid type. Should be backward_model_saturation");
  }

  YAML::Node n_lin_model = config["backward_model"];

  if (!n_lin_model.IsMap()) {
    throw InvalidCalibration("Invalid map backward_model");
  }

  string lin_model_type;

  try {
    lin_model_type = n_lin_model["type"].as<string>();
  } catch (YAML::Exception& e) {
    throw InvalidCalibration("Unable to set backward model type.");
  }

  string lin_model_fn;

  try {
    lin_model_fn = n_lin_model["filename"].as<string>();
  } catch (YAML::Exception& e) {
    throw InvalidCalibration("Unable to set backward model filename.");
  }

  auto lin_fact = BackwardModelFactory();
  p_bac_model_ = lin_fact.create(lin_model_type, pathAppend(parent_dir, lin_model_fn));

  string sat_functions_fn;
  try {
    sat_functions_fn = config["saturations_filename"].as<string>();
  } catch (YAML::Exception& e) {
    throw InvalidCalibration("Unable to set the saturations filename.");
  }

  setSaturationFunctionsFile(pathAppend(parent_dir, sat_functions_fn));
}

void BackwardModelSaturation::setModel(BackwardModel::Ptr p_lin_model) {
  p_bac_model_ = p_lin_model;
}

BackwardModel::Ptr BackwardModelSaturation::getModel() const { return p_bac_model_; }

SaturationFunction::Ptr BackwardModelSaturation::getSaturationFunction(const int i) const {
  return sat_functions_[i];
}

vector<SaturationFunction::Ptr> BackwardModelSaturation::getSaturationFunctions() const {
  return sat_functions_;
}

void BackwardModelSaturation::setModelCalibrationFile(const std::string& filename) {
  if (p_bac_model_ == nullptr) {
    throw InvalidCalibration("Backward model not set");
  } else {
    p_bac_model_->setCalibrationFile(filename);
  }
}

void BackwardModelSaturation::setSaturationFunctionsFile(const std::string& filename) {
  sat_functions_.clear();
  sat_functions_ = saturationFunctionsFromFile(filename);

  if (sat_functions_.size() != getNumCoils()) {
    throw InvalidFile(filename, "number of saturation functions does not match number of coils");
  }
}

void BackwardModelSaturation::setSaturationFunctions(
    std::vector<SaturationFunction::Ptr> sat_functions) {
  sat_functions_.clear();

  for (auto& p_sat_fun : sat_functions) {
    sat_functions_.push_back(p_sat_fun);
  }
}

CurrentsVec BackwardModelSaturation::computeCurrentsFromFieldGradient5(
    const PositionVec& position, const FieldVec& field, const Gradient5Vec& gradient) const {
  assert(sat_functions_.size() == getNumCoils());

  CurrentsVec currents_s =
      p_bac_model_->computeCurrentsFromFieldGradient5(position, field, gradient);
  CurrentsVec currents(getNumCoils());

  // the currents should be higher to compensate for saturation
  for (int i = 0; i < getNumCoils(); i++) {
    if (do_check_max_) checkMax(sat_functions_[i], currents_s[i]);

    currents[i] = sat_functions_[i]->inverse(currents_s[i]);
  }

  return currents;
}

CurrentsVec BackwardModelSaturation::computeCurrentsFromFieldDipoleGradient3(
    const PositionVec& position, const FieldVec& field, const DipoleVec& dipole,
    const Gradient3Vec& gradient) const {
  assert(sat_functions_.size() == getNumCoils());

  CurrentsVec currents_s =
      p_bac_model_->computeCurrentsFromFieldDipoleGradient3(position, field, dipole, gradient);

  CurrentsVec currents(getNumCoils());

  // the currents should be higher to compenesate for saturation
  for (int i = 0; i < getNumCoils(); i++) {
    if (do_check_max_) checkMax(sat_functions_[i], currents_s[i]);
    currents[i] = sat_functions_[i]->inverse(currents_s[i]);
  }

  return currents;
}

void BackwardModelSaturation::checkMax(SaturationFunction::Ptr p_sat, double current) {
  if (abs(current) > (0.99 * p_sat->max())) {
    throw OverSaturationException();
  }
}

void BackwardModelSaturation::setCachedPosition(const PositionVec& position) {
  p_bac_model_->setCachedPosition(position);
}

PositionVec BackwardModelSaturation::getCachedPosition() const {
  return p_bac_model_->getCachedPosition();
}

void BackwardModelSaturation::setCachedPositionDipole(const PositionVec& position,
                                                      const DipoleVec& dipole) {
  p_bac_model_->setCachedPositionDipole(position, dipole);
}

DipoleVec BackwardModelSaturation::getCachedDipole() const {
  return p_bac_model_->getCachedDipole();
}

CurrentsVec BackwardModelSaturation::computeCurrentsFromFieldCached(const FieldVec& field) const {
  assert(sat_functions_.size() == getNumCoils());

  CurrentsVec currents_s = p_bac_model_->computeCurrentsFromFieldCached(field);
  CurrentsVec currents(getNumCoils());

  // the currents should be higher to compenesate for saturation
  for (int i = 0; i < getNumCoils(); i++) {
    if (do_check_max_) checkMax(sat_functions_[i], currents_s[i]);
    currents[i] = sat_functions_[i]->inverse(currents_s[i]);
  }

  return currents;
}

CurrentsVec BackwardModelSaturation::computeCurrentsFromFieldGradient5Cached(
    const FieldVec& field, const Gradient5Vec& gradient) const {
  assert(sat_functions_.size() == getNumCoils());

  CurrentsVec currents_s = p_bac_model_->computeCurrentsFromFieldGradient5Cached(field, gradient);
  CurrentsVec currents(getNumCoils());

  // the currents should be higher to compenesate for saturation
  for (int i = 0; i < getNumCoils(); i++) {
    if (do_check_max_) checkMax(sat_functions_[i], currents_s[i]);
    currents[i] = sat_functions_[i]->inverse(currents_s[i]);
  }

  return currents;
}

CurrentsVec BackwardModelSaturation::computeCurrentsFromFieldDipoleGradient3Cached(
    const FieldVec& field, const Gradient3Vec& gradient) const {
  assert(sat_functions_.size() == getNumCoils());

  CurrentsVec currents_s =
      p_bac_model_->computeCurrentsFromFieldDipoleGradient3Cached(field, gradient);
  CurrentsVec currents(getNumCoils());

  // the currents should be higher to compenesate for saturation
  for (int i = 0; i < getNumCoils(); i++) {
    if (do_check_max_) checkMax(sat_functions_[i], currents_s[i]);
    currents[i] = sat_functions_[i]->inverse(currents_s[i]);
  }

  return currents;
}
}  // namespace mag_manip
