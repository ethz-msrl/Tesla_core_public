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

#include "mag_manip/forward_model_linear_thinplatespline.h"
#include <yaml-cpp/yaml.h>
#include "mag_manip/exceptions.h"
#include "mag_manip/helpers.h"

using namespace mag_manip;
using namespace std;

ForwardModelLinearThinPlateSpline::ForwardModelLinearThinPlateSpline()
    : is_valid_(false), name_(""), num_coils_(0) {}

std::string ForwardModelLinearThinPlateSpline::getName() const { return name_; }

void ForwardModelLinearThinPlateSpline::setCalibrationFile(const string& filename) {
  cal_filename_ = filename;

  YAML::Node config;

  try {
    config = YAML::LoadFile(filename);
  } catch (YAML::BadFile& e) {
    throw InvalidFile(filename, e.what());
  }

  try {
    name_ = config["name"].as<string>();
  } catch (YAML::Exception& e) {
    throw InvalidFile(filename, "unable to get name");
  }

  YAML::Node n_vfields;
  try {
    n_vfields = config["vfields"];
  } catch (YAML::Exception& e) {
    throw InvalidFile(filename, "vfields field invalid");
  }

  num_coils_ = n_vfields.size();
  if (num_coils_ == 0) {
    throw InvalidCalibration("Number of vfields is zero in " + filename);
  }

  // For some reason yaml-cpp does not sort map keys
  // We do one pass to get the sorted keys and then query the node key by key
  vector<string> coil_keys;
  for (YAML::const_iterator it = n_vfields.begin(); it != n_vfields.end(); it++) {
    coil_keys.push_back(it->first.as<string>());
  }

  std::sort(coil_keys.begin(), coil_keys.end());

  for (const string& key : coil_keys) {
    YAML::Node n_vfield = n_vfields[key];

    // string kernel_type = n_vfield["kernel"].as<string>();
    // v_kernel_types_.push_back(kernel_type);

    YAML::Node n_nodes = n_vfield["nodes"];
    const int num_nodes = n_nodes.size();
    if (num_nodes == 0) {
      throw InvalidCalibration("Nodes is empty in coil " + key);
    }

    v_num_nodes_.push_back(num_nodes);

    // we don't make any checks on the shape param
    // for example, negative values are allowed
    // const double shape_param = n_vfield["shape_param"].as<double>();
    // v_shape_params_.push_back(shape_param);

    Eigen::MatrixXd nodes(3, num_nodes);
    Eigen::MatrixXd values(3, num_nodes);

    for (int i = 0; i < num_nodes; i++) {
      YAML::Node n_node = n_nodes[i];
      vector<double> v_position = n_node["position"].as<vector<double> >();
      if (v_position.size() != 3) {
        throw InvalidFile(filename, "A node position does not have length 3 in coil " + key);
      }
      nodes(0, i) = v_position[0];
      nodes(1, i) = v_position[1];
      nodes(2, i) = v_position[2];

      vector<double> v_value = n_node["value"].as<vector<double> >();
      if (v_value.size() != 3) {
        throw InvalidFile(filename, "A node value does not have length 3 in coil " + key);
      }
      values(0, i) = v_value[0];
      values(1, i) = v_value[1];
      values(2, i) = v_value[2];
    }

    // create an interpolant for that coil
    auto p_interp = std::make_shared<ThinPlateSplineInterpolator>(nodes, values);
    v_p_interpolants_.push_back(p_interp);
  }

  is_valid_ = true;
}

std::vector<ThinPlateSplineInterpolator::Ptr> ForwardModelLinearThinPlateSpline::getInterpolants()
    const {
  return v_p_interpolants_;
}

ActuationMat ForwardModelLinearThinPlateSpline::getActuationMatrix(
    const PositionVec& position) const {
  ActuationMat act_mat(8, num_coils_);
  for (int i = 0; i < num_coils_; i++) {
    FieldVec field_unit_current = (*v_p_interpolants_[i])(position);
    act_mat.block<3, 1>(0, i) = field_unit_current;
    Eigen::Tensor<double, 3> t_gradients = v_p_interpolants_[i]->getGradients(position);
    GradientMat gradient_unit_current = Eigen::Map<GradientMat>(t_gradients.data(), 3, 3);

    act_mat.block<5, 1>(3, i) = gradientMatToGradient5Vec(gradient_unit_current);
  }

  return act_mat;
}

ActuationMat ForwardModelLinearThinPlateSpline::getFieldActuationMatrix(
    const PositionVec& position) const {
  ActuationMat act_mat(3, num_coils_);
  for (int i = 0; i < num_coils_; i++) {
    FieldVec field_unit_current = (*v_p_interpolants_[i])(position);
    act_mat.col(i) = field_unit_current;
  }
  return act_mat;
}

int ForwardModelLinearThinPlateSpline::getNumCoils() const { return num_coils_; }

bool ForwardModelLinearThinPlateSpline::isValid() const { return is_valid_; }
