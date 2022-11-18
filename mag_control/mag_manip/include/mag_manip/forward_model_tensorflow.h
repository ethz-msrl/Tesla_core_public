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

#ifndef NO_TENSORFLOW
#pragma once

#include <memory>
#include <string>

#include "mag_manip/forward_model.h"
#include "mag_manip/interpolate_tricubic.h"
#include "mag_manip/interpolate_trilinear.h"
#include "mag_manip/rbf_3d_field_interpolator.h"
#include "mag_manip/vfield_grid_properties.h"
#include "mag_tensorflow/model.h"

namespace mag_manip {

/**
 * @brief A forward model that uses a generative neural network trained in TensorFlow.
 *
 * The generative model creates a nonlinear mapping between the vector of electromagnet currents and
 * a Ng x Ng x Ng x 3 tensor containing the magnetic field vectors on a regular grid of size Ng.
 * This is so far the most accurate model tested on the CardioMag.
 *
 * Because the model outputs a discretized field map, an extra interpolation step is added at the
 * output to compute fields at arbitrary positions in the workspace.
 */
class ForwardModelTensorFlow : public ForwardModel {
 public:
  typedef std::shared_ptr<ForwardModelTensorFlow> Ptr;
  typedef std::unique_ptr<ForwardModelTensorFlow> UPtr;

  virtual ~ForwardModelTensorFlow() {}

  ForwardModelTensorFlow();

  /**
   * @brief Sets the calibration of the model
   *
   * This sets parameters of the tensorflow model. This filename should be that path to a YAML file
   * containing the parameters of the tensorflow model. Note that the actual model is stored using
   * TensorFlow's SavedModel interface. The SavedModel is assumed to live in the same folder as
   * filename.
   *
   * @param filename: path to a params.yaml file containing metadata of the tensorflow model.
   */
  virtual void setCalibrationFile(const std::string& filename) override;

  /**
   * @brief Outputs the name of the system
   *
   * @return the system name
   */
  std::string getName() const;

  virtual bool isValid() const override;

  virtual int getNumCoils() const override;

  VFieldGridProperties getOutputGridProperties() const;

  virtual FieldVec computeFieldFromCurrents(const PositionVec& position,
                                            const CurrentsVec& currents) const;

  /**
   * @brief Computes the field at several positions at a time
   *
   * This function is faster than repeatedly calling computeFieldFromCurrents since it only
   * requires evaluating the tensorflow model once for the currents vector.
   *
   * @param positions: a 3xN matrix containing the positions at which to evaluate the field
   * @param currents: the vector of electromagnet currents
   *
   * @return a 3xN matrix containing the magnetic fields at the given positions
   */
  virtual FieldVecs computeFieldsFromCurrents(const PositionVecs& positions,
                                              const CurrentsVec& currents) const;

  /**
   * @brief This function uses an RBF interpolant instead of a regular grid interpolant
   *
   * This is used for testing purposes only
   *
   * @param position
   * @param currents
   *
   * @return
   */
  FieldVec computeFieldFromCurrentsRBF(const PositionVec& position,
                                       const CurrentsVec& currents) const;

  virtual Gradient5Vec computeGradient5FromCurrents(const PositionVec& position,
                                                    const CurrentsVec& currents) const override;

  /**
   * @brief Computes the gradient vector at several positions at a time
   *
   * This function is faster than repeatedly calling computeGradient5FromCurrents since it only
   * requires evaluating the tensorflow model once for the currents vector.
   *
   * @param positions: a 3xN matrix containing the positions at which to evaluate the field
   * @param currents: the vector of electromagnet currents
   *
   * @return a 5xN matrix containing the magnetic field gradient vectors at the given positions
   */
  virtual Gradient5Vecs computeGradient5sFromCurrents(const PositionVecs& positions,
                                                      const CurrentsVec& currents) const;

  virtual FieldGradient5Vec computeFieldGradient5FromCurrents(
      const PositionVec& position, const CurrentsVec& currents) const override;

  /**
   * @brief Computes the field and gradient vector at several positions at a time
   *
   * This function is faster than repeatedly calling computeFieldGradient5FromCurrents since it only
   * requires evaluating the tensorflow model once for the currents vector.
   *
   * @param positions: a 3xN matrix containing the positions at which to evaluate the field
   * @param currents: the vector of electromagnet currents
   *
   * @return a 8xN matrix containing the concatenation of the field and magnetic field gradient
   * vectors at the given positions
   */
  virtual FieldGradient5Vecs computeFieldGradient5sFromCurrents(const PositionVecs& positions,
                                                                const CurrentsVec& currents) const;

  // This function is just for benchmarking
  void runModel(const CurrentsVec& currents) const;

 private:
  /**
   * @brief Normalizes the currents to be in the range [-1,1]
   *
   * @param currents: the currents vector to normalize
   *
   * @return the normalized currents vector
   */
  CurrentsVec normalizeCurrents(const CurrentsVec& currents) const;

  /**
   * @brief Computes the field map output of the tensorflow model
   * and creates polynomial based interpolant based on the generated data.
   *
   * @param currents: the currents at which to compute the field
   *
   * @return a local polynomial interpoland that can be used to interpolate the fields and gradients
   * at any position
   */
  InterpolateRegular::Ptr getInterpolateRegular(const CurrentsVec& currents) const;

  /**
   * @brief Computes the field map output of the tensorflow model
   * and creates an RBFGaussian based interpolant based on the generated data.
   *
   * Note that this version is only used for comparison with the Tricubic interpolant which is much
   * faster
   *
   * @param currents: the currents at which to compute the field
   *
   * @return an RBF interpoland that can be used to interpolate the fields and gradients at any
   * position
   */
  RBF3DFieldInterpolator::Ptr getRBF3DFieldInterpolator(const CurrentsVec& currents) const;

  std::unique_ptr<mag_tensorflow::Model> p_tf_model_;
  std::string name_;
  int num_coils_;
  bool is_valid_;
  std::string model_dir_;      /** the path to the SavedModel */
  std::string signature_tag_;  /** the signature_tag in the SavedModel */
  std::string input_op_name_;  /** the name of the input operation in the neural network */
  std::string output_op_name_; /** the name of the output operation in the neural network */
  std::unique_ptr<VFieldGridProperties>
      p_output_grid_props_; /** the properties of the regular grid on which the magnetic field is
                               computed */
  Eigen::MatrixXd node_positions_; /** the positions of the regular grid */
  double scaling_field_; /** the outputs of the neural network are scaled by this parameter to help
                            with training */
  double max_current_; /** this is used to scale the input currents which was used for training the
                          model */
  double min_current_;
};
}  // namespace mag_manip
#endif  // NO_TENSORFLOW
