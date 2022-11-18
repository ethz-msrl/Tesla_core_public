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

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "mag_manip/rbf.h"
#include "mag_manip/types.h"

namespace mag_manip {

class RBF3DFieldInterpolator {
 public:
  typedef std::shared_ptr<RBF3DFieldInterpolator> Ptr;

  virtual FieldVec getField(const PositionVec& position) const = 0;
  virtual GradientMat getGradient(const PositionVec& position) const = 0;

  static Ptr create(const std::string& kernel, const Eigen::MatrixXd& nodes,
                    const Eigen::MatrixXd& values, double shape_param);
};

template <typename RBFPolicy>
class RBF3DFieldInterpolatorImpl : public RBF3DFieldInterpolator {
 public:
  /**
   * @brief Creates an RBF based interpolator for 3D vector fields
   *
   * @param nodes: a Nx3 matrix containing the node positions
   * @param values: a Nx3 matrix containing the vector data at the node positions
   * @param shape_param: the scalar shape parameter that is used in some kernels
   */
  RBF3DFieldInterpolatorImpl(const Eigen::MatrixXd& nodes, const Eigen::MatrixXd& values,
                             double shape_param)
      : interp_(nodes, values, shape_param) {}

  /**
   * @brief Interpolates the field value at the given position
   *
   * @param position: the position at which to interpolate the field
   *
   * @return the interpolated field
   */
  virtual FieldVec getField(const PositionVec& position) const override {
    return interp_(position);
  }

  /**
   * @brief Calculates the gradient of the interpolant at the given position
   *
   * This uses an analytical expression of the interpolant to calculate the estimated derivatives
   *
   * @param position: the position at which to calculate the gradient
   *
   * @return the 3x3 gradient matrix
   */
  virtual GradientMat getGradient(const PositionVec& position) const override {
    Eigen::Tensor<double, 3> t_gradients = interp_.getGradients(position);
    return Eigen::Map<GradientMat>(t_gradients.data(), 3, 3);
  }

 private:
  RBFInterpolator<double, 3, 3, RBFPolicy> interp_;
};

using RBF3DFieldGaussianInterpolator = RBF3DFieldInterpolatorImpl<RBFGaussianPolicy>;
using RBF3DFieldMultiquadricInterpolator = RBF3DFieldInterpolatorImpl<RBFMultiquadricPolicy>;
using RBF3DFieldInverseMultiquadricInterpolator =
    RBF3DFieldInterpolatorImpl<RBFInverseMultiquadricPolicy>;
using RBF3DFieldCubicInterpolator = RBF3DFieldInterpolatorImpl<RBFCubicPolicy>;
using RBF3DFieldThinPlateSplineInterpolator = RBF3DFieldInterpolatorImpl<RBFThinPlatePolicy>;

using RBF3DFieldInterpolatorVec = std::vector<RBF3DFieldInterpolator::Ptr>;

}  // namespace mag_manip
