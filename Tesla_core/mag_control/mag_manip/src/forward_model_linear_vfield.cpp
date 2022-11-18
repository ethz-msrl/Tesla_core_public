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

#include "mag_manip/forward_model_linear_vfield.h"
#include <yaml-cpp/yaml.h>
#include "mag_manip/exceptions.h"
#include "mag_manip/helpers.h"
#include "mag_manip/interpolate_regular_factory.h"
#include "mag_manip/vfield.h"

using namespace std;
using namespace Eigen;

namespace mag_manip {

ForwardModelLinearVField::ForwardModelLinearVField()
    : is_valid_(false),
      name_(""),
      interp_type_(InterpolateRegular::Type::TRICUBIC),
      num_coils_(0) {}

void ForwardModelLinearVField::setInterpolationType(InterpolateRegular::Type type) {
  interp_type_ = type;

  // make sure to reload the interpolants
  if (filename_ != "") {
    setCalibrationFile(filename_);
  }
}

vector<VFieldGridProperties> ForwardModelLinearVField::getVFieldGridProperties() const {
  vector<VFieldGridProperties> properties;
  for (auto const& p_interp : interpolants_) {
    properties.push_back(p_interp->getVFieldGridProperties());
  }
  return properties;
}

bool ForwardModelLinearVField::pointInWorkspace(const PositionVec& position) const {
  if (!is_valid_) {
    throw CalibrationNotLoaded();
  }

  vector<VFieldGridProperties> v_props = getVFieldGridProperties();
  return pointInVFieldWorkspace(position, v_props);
}

void ForwardModelLinearVField::setVFieldFiles(const vector<string>& vfield_filenames) {
  interpolants_.clear();

  num_coils_ = vfield_filenames.size();
  InterpolateRegularFactory f;

  for (const string& filename : vfield_filenames) {
    std::tuple<DataMat, VFieldGridProperties> ret = parseVFieldFile(filename);
    interpolants_.push_back(f.create(interp_type_, std::get<0>(ret), std::get<1>(ret)));
  }

  num_coils_ = interpolants_.size();
  is_valid_ = true;
}

void ForwardModelLinearVField::setCalibrationFile(const string& filename) {
  filename_ = filename;
  YAML::Node config;

  try {
    config = YAML::LoadFile(filename);
  } catch (YAML::BadFile& e) {
    throw InvalidFile(filename, e.what());
  }

  name_ = config["name"].as<string>();
  YAML::Node n_vfields = config["vfields"];

  num_coils_ = n_vfields.size();
  if (num_coils_ == 0) {
    throw InvalidCalibration("Number of vfields is zero in " + filename);
  }

  interpolants_.clear();
  InterpolateRegularFactory interp_fact;

  // For some reason yaml-cpp does not sort map keys
  // We do one pass to get the sorted keys and then query the node key by key
  vector<string> coil_keys;
  for (YAML::const_iterator it = n_vfields.begin(); it != n_vfields.end(); it++) {
    coil_keys.push_back(it->first.as<string>());
  }

  std::sort(coil_keys.begin(), coil_keys.end());

  for (const string& key : coil_keys) {
    std::tuple<DataMat, VFieldGridProperties> ret = parseVFieldYAML(n_vfields[key]);
    interpolants_.push_back(interp_fact.create(interp_type_, std::get<0>(ret), std::get<1>(ret)));
  }

  is_valid_ = true;
}

GradientMat ForwardModelLinearVField::computeGradientMatFromCurrents(
    const PositionVec& position, const CurrentsVec& currents) const {
  if (!is_valid_) {
    throw CalibrationNotLoaded();
  }

  if (currents.size() != num_coils_) {
    throw InvalidCurrentsLength();
  }

  GradientMat gradient;
  gradient.setZero();
  for (int i = 0; i < num_coils_; i++) {
    // Interpolation does not guarantee zero curl and zero trace of B
    // mapping betwen gradientMat and Gradient5 is not really defined here
    gradient += currents(i) * interpolants_[i]->getGradient(position);
  }

  return gradient;
}

ActuationMat ForwardModelLinearVField::getActuationMatrix(const PositionVec& p) const {
  ActuationMat act_mat(8, num_coils_);
  act_mat.topRows<3>() = getFieldActuationMatrix(p);
  for (int i = 0; i < num_coils_; i++) {
    // Interpolation does not guarantee zero curl and zero trace of B
    // mapping betwen gradientMat and Gradient5 is not really defined here
    act_mat.block<5, 1>(3, i) = gradientMatToGradient5Vec(interpolants_[i]->getGradient(p));
  }
  return act_mat;
}

ActuationMat ForwardModelLinearVField::getFieldActuationMatrix(const PositionVec& p) const {
  if (!is_valid_) {
    throw CalibrationNotLoaded();
  }

  ActuationMat act_mat(3, num_coils_);
  for (int i = 0; i < num_coils_; i++) {
    act_mat.col(i) = interpolants_[i]->interpolate(p);
  }

  return act_mat;
}

FieldVec ForwardModelLinearVField::computeFieldFromCurrents(const PositionVec& p,
                                                            const CurrentsVec& currents) const {
  if (currents.size() != num_coils_) {
    throw InvalidCurrentsLength();
  }

  return getFieldActuationMatrix(p) * currents;
}

Gradient5Vec ForwardModelLinearVField::computeGradient5FromCurrents(
    const PositionVec& position, const CurrentsVec& currents) const {
  if (!is_valid_) {
    throw CalibrationNotLoaded();
  }
  if (currents.size() != num_coils_) {
    throw InvalidCurrentsLength();
  }

  ActuationMat grad_act_mat(5, num_coils_);
  for (int i = 0; i < num_coils_; i++) {
    // Interpolation does not guarantee zero curl and zero trace of B
    // mapping betwen gradientMat and Gradient5 is not really defined here
    grad_act_mat.row(i) = gradientMatToGradient5Vec(interpolants_[i]->getGradient(position));
  }

  return grad_act_mat * currents;
}

FieldGradient5Vec ForwardModelLinearVField::computeFieldGradient5FromCurrents(
    const PositionVec& position, const CurrentsVec& currents) const {
  ActuationMat act_mat = getActuationMatrix(position);
  return act_mat * currents;
}

}  // namespace mag_manip
