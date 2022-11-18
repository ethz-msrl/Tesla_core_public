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

#include "mag_manip/forward_model_tensorflow.h"
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <unsupported/Eigen/CXX11/Tensor>
#include "mag_manip/exceptions.h"
#include "mag_manip/helpers.h"
#include "mag_manip/utils.h"
#include "mag_manip/vfield.h"

using namespace mag_manip;
using namespace std;

ForwardModelTensorFlow::ForwardModelTensorFlow() : is_valid_(false) {}

string ForwardModelTensorFlow::getName() const { return name_; }

void ForwardModelTensorFlow::setCalibrationFile(const std::string& filename) {
  YAML::Node config;
  try {
    config = YAML::LoadFile(filename);
  } catch (YAML::BadFile& e) {
    throw InvalidFile(filename, e.what());
  }

  try {
    name_ = config["name"].as<string>();
  } catch (YAML::Exception& e) {
    throw InvalidFile(filename, "Unable to read name");
  }

  try {
    num_coils_ = config["num_coils"].as<int>();
  } catch (YAML::Exception& e) {
    throw InvalidFile(filename, "Unable to read num_coils");
  }

  // Assume that the model is in the same folder as this params file
  /*try {
    model_dir_ = config["model_dir"].as<string>();
  } catch (YAML::Exception& e) {
    throw InvalidFile(filename, "Unable to read model_dir");
  }*/

  try {
    signature_tag_ = config["signature_tag"].as<string>();
  } catch (YAML::Exception& e) {
    throw InvalidFile(filename, "Unable to read signature_tag");
  }

  try {
    input_op_name_ = config["input_op_name"].as<string>();
  } catch (YAML::Exception& e) {
    throw InvalidFile(filename, "Unable to read input_op_name");
  }

  try {
    output_op_name_ = config["output_op_name"].as<string>();
  } catch (YAML::Exception& e) {
    throw InvalidFile(filename, "Unable to read output_op_name");
  }

  try {
    p_output_grid_props_.reset(
        new VFieldGridProperties(parseVFieldGridPropertiesYAML(config["output_grid"])));
  } catch (YAML::Exception& e) {
    throw InvalidFile(filename, "Unable to parse output_grid");
  }

  const std::vector<int64_t> input_dims = {1, num_coils_};
  // try {
  p_tf_model_.reset(new mag_tensorflow::Model(getFileDirectory(filename), signature_tag_,
                                              input_op_name_, input_dims, output_op_name_));
  /* } catch (std::runtime_error& e) {
    throw InvalidFile(filename, "Unable to load tensorflow savedmodel");
  } */

  try {
    scaling_field_ = config["scaling_field"].as<double>();
  } catch (YAML::Exception& e) {
    throw InvalidFile(filename, "Unable to read scaling_field");
  }

  try {
    min_current_ = config["min_current"].as<double>();
  } catch (YAML::Exception& e) {
    throw InvalidFile(filename, "Unable to read min_current");
  }

  try {
    max_current_ = config["max_current"].as<double>();
  } catch (YAML::Exception& e) {
    throw InvalidFile(filename, "Unable to read max_current");
  }

  if (min_current_ > max_current_) {
    throw InvalidFile(filename, "min_current can't be larger than max_current");
  }

  is_valid_ = true;
}

bool ForwardModelTensorFlow::isValid() const { return is_valid_; }

int ForwardModelTensorFlow::getNumCoils() const { return num_coils_; }

VFieldGridProperties ForwardModelTensorFlow::getOutputGridProperties() const {
  return *p_output_grid_props_;
}

FieldVec ForwardModelTensorFlow::computeFieldFromCurrents(const PositionVec& position,
                                                          const CurrentsVec& currents) const {
  return getInterpolateRegular(currents)->interpolate(position);
}

FieldVecs ForwardModelTensorFlow::computeFieldsFromCurrents(const PositionVecs& positions,
                                                            const CurrentsVec& currents) const {
  FieldVecs fields(3, positions.cols());
  InterpolateRegular::Ptr p_interpolant = getInterpolateRegular(currents);
  for (int i = 0; i < positions.cols(); i++) {
    fields.col(i) = p_interpolant->interpolate(positions.col(i));
  }
  return fields;
}

FieldVec ForwardModelTensorFlow::computeFieldFromCurrentsRBF(const PositionVec& position,
                                                             const CurrentsVec& currents) const {
  return getRBF3DFieldInterpolator(currents)->getField(position);
}

Gradient5Vec ForwardModelTensorFlow::computeGradient5FromCurrents(
    const PositionVec& position, const CurrentsVec& currents) const {
  GradientMat grad_mat = getInterpolateRegular(currents)->getGradient(position);
  return gradientMatToGradient5Vec(grad_mat);
}

Gradient5Vecs ForwardModelTensorFlow::computeGradient5sFromCurrents(
    const PositionVecs& positions, const CurrentsVec& currents) const {
  Gradient5Vecs gradients(5, positions.cols());
  InterpolateRegular::Ptr p_interpolant = getInterpolateRegular(currents);
  for (int i = 0; i < positions.cols(); i++) {
    GradientMat grad_mat = p_interpolant->getGradient(positions.col(i));
    gradients.col(i) = gradientMatToGradient5Vec(grad_mat);
  }
  return gradients;
}

FieldGradient5Vec ForwardModelTensorFlow::computeFieldGradient5FromCurrents(
    const PositionVec& position, const CurrentsVec& currents) const {
  InterpolateRegular::Ptr p_interpolant = getInterpolateRegular(currents);
  FieldVec field = p_interpolant->interpolate(position);
  Gradient5Vec gradient = gradientMatToGradient5Vec(p_interpolant->getGradient(position));
  FieldGradient5Vec field_gradient;
  field_gradient << field, gradient;
  return field_gradient;
}

FieldGradient5Vecs ForwardModelTensorFlow::computeFieldGradient5sFromCurrents(
    const PositionVecs& positions, const CurrentsVec& currents) const {
  FieldGradient5Vecs field_gradients(8, positions.cols());
  InterpolateRegular::Ptr p_interpolant = getInterpolateRegular(currents);
  for (int i = 0; i < positions.cols(); i++) {
    FieldVec field = p_interpolant->interpolate(positions.col(i));
    GradientMat grad_mat = p_interpolant->getGradient(positions.col(i));
    Gradient5Vec gradient = gradientMatToGradient5Vec(grad_mat);
    field_gradients.col(i) << field, gradient;
  }
  return field_gradients;
}

CurrentsVec ForwardModelTensorFlow::normalizeCurrents(const CurrentsVec& currents) const {
  return ((currents.array() - min_current_) / (max_current_ - min_current_)) * 2 - 1;
}

InterpolateRegular::Ptr ForwardModelTensorFlow::getInterpolateRegular(
    const CurrentsVec& currents) const {
  if (!is_valid_) {
    throw CalibrationNotLoaded();
  }

  if (currents.size() != num_coils_) {
    throw InvalidCurrentsLength();
  }

  CurrentsVec currents_n = normalizeCurrents(currents);

  vector<float> v_input(currents_n.data(),
                        currents_n.data() + currents_n.rows() * currents_n.cols());

  std::pair<std::vector<int>, std::vector<float> > outputs = p_tf_model_->runInput(v_input);

  const int num_positions =
      p_output_grid_props_->dim_x * p_output_grid_props_->dim_y * p_output_grid_props_->dim_z;

  // tensorflow will output a tensor with a singleton dimension so we we also check that the 1, nx,
  // ny, nz dimensions matches the expected grid parameters
  std::vector<int> expected_dims = {p_output_grid_props_->dim_x, p_output_grid_props_->dim_y,
                                    p_output_grid_props_->dim_z, 3};
  std::vector<int> expected_dims_5 = {1, p_output_grid_props_->dim_x, p_output_grid_props_->dim_y,
                                      p_output_grid_props_->dim_z, 3};
  if (outputs.first.size() != 4 && outputs.first.size() != 5) {
    throw InvalidOutputTensorDimensions(expected_dims, outputs.first);
  }
  if (outputs.first.size() == 5 && outputs.first != expected_dims_5) {
    throw InvalidOutputTensorDimensions(expected_dims_5, outputs.first);
  }
  if (outputs.first.size() == 4 && outputs.first != expected_dims) {
    throw InvalidOutputTensorDimensions(expected_dims, outputs.first);
  }

  // Deep fluids uses the Z,Y,X,3 axis convention
  // we need to flip the axes to X,Y,Z,3
  // so we use Eigen's shuffle function

  // Eigen stores tensors in column major form as opposed
  // to tensorflow in Python which stores in row-major
  // We need to load the Tensor in Eigen::RowMajor form to do some index shuffling
  Eigen::TensorMap<Eigen::Tensor<float, 4, Eigen::RowMajor> > t_field_output_(
      outputs.second.data(), p_output_grid_props_->dim_z, p_output_grid_props_->dim_y,
      p_output_grid_props_->dim_x, 3);

  Eigen::array<int, 4> dims({3, 2, 1, 0});
  Eigen::Tensor<float, 4, Eigen::RowMajor> t_field_output = t_field_output_.shuffle(dims);
  t_field_output = t_field_output * static_cast<float>(scaling_field_);

  // Here we remap to a matrix assuming that the dimensions
  // are already correctly formatted keeping the row major format
  // The row major format should be converted to column major implicitly by the next
  // matrix assign
  Eigen::Map<Eigen::Matrix<float, 3, Eigen::Dynamic, Eigen::RowMajor> > field_output(
      t_field_output.data(), 3, num_positions);
  // field_output.array() *= scaling_field_;
  return std::make_shared<InterpolateTricubic>(field_output.cast<double>(), *p_output_grid_props_);
}

RBF3DFieldInterpolator::Ptr ForwardModelTensorFlow::getRBF3DFieldInterpolator(
    const CurrentsVec& currents) const {
  if (!is_valid_) {
    throw CalibrationNotLoaded();
  }

  if (currents.size() != num_coils_) {
    throw InvalidCurrentsLength();
  }

  CurrentsVec currents_n = normalizeCurrents(currents);

  vector<float> v_input(currents_n.data(),
                        currents_n.data() + currents_n.rows() * currents_n.cols());

  std::pair<std::vector<int>, std::vector<float> > outputs = p_tf_model_->runInput(v_input);

  const int num_positions =
      p_output_grid_props_->dim_x * p_output_grid_props_->dim_y * p_output_grid_props_->dim_z;

  // we should check here that the output dimensions match
  // the grid properties

  // Deep fluids uses the Z,Y,X,3 axis convention
  // we need to flip the axes to X,Y,Z,3
  // so we use Eigen's shuffle function

  // Eigen stores tensors in column major form as opposed
  // to tensorflow in Python which stores in row-major
  // We need to load the Tensor in Eigen::RowMajor form to do some index shuffling
  Eigen::TensorMap<Eigen::Tensor<float, 4, Eigen::RowMajor> > t_field_output_(
      outputs.second.data(), p_output_grid_props_->dim_z, p_output_grid_props_->dim_y,
      p_output_grid_props_->dim_x, 3);

  Eigen::array<int, 4> dims({3, 2, 1, 0});
  Eigen::Tensor<float, 4, Eigen::RowMajor> t_field_output = t_field_output_.shuffle(dims);
  t_field_output = t_field_output * static_cast<float>(scaling_field_);

  // Here we remap to a matrix assuming that the dimensions
  // are already correctly formatted keeping the row major format
  // The row major format should be converted to column major implicitly by the next
  // matrix assign
  Eigen::Map<Eigen::Matrix<float, 3, Eigen::Dynamic, Eigen::RowMajor> > field_output(
      t_field_output.data(), 3, num_positions);
  Eigen::MatrixXd field_output_d = field_output.cast<double>();

  // the size of the workspace is approx 20 cm in side
  // and the grid resolution is 16x16x16 so the approximate distance between nodes is 1.25 cm. We
  // use the heuristic of average distance * 0.815
  Eigen::MatrixXd nodes = getGridPositions(*p_output_grid_props_);
  const double shape_param = 10000;
  return std::make_shared<RBF3DFieldMultiquadricInterpolator>(nodes, field_output_d, shape_param);
}

void ForwardModelTensorFlow::runModel(const CurrentsVec& currents) const {
  if (!is_valid_) {
    throw CalibrationNotLoaded();
  }

  if (currents.size() != num_coils_) {
    throw InvalidCurrentsLength();
  }

  CurrentsVec currents_n = normalizeCurrents(currents);

  vector<float> v_input(currents_n.data(),
                        currents_n.data() + currents_n.rows() * currents_n.cols());

  std::pair<std::vector<int>, std::vector<float> > outputs = p_tf_model_->runInput(v_input);
}
