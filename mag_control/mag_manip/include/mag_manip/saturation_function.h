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

#include <Eigen/Dense>
#include <memory>

namespace mag_manip {

class SaturationFunction {
 public:
  typedef std::unique_ptr<SaturationFunction> UPtr;
  typedef std::shared_ptr<SaturationFunction> Ptr;

  /**
   * @brief Models a 1D saturation behaviour
   *
   * @param params: vector containing the parameters of the saturation function
   */
  explicit SaturationFunction(const Eigen::VectorXd& params) : params_(params) {}

  virtual ~SaturationFunction() {}

  /**
   * @brief Evaluates the saturation function
   *
   * y = h(x)
   *
   * @param x: value to evaluate
   *
   * @return evaluated function
   */
  virtual double evaluate(const double x) const = 0;

  /**
   * @brief Evaluates the Jacobian of the saturation function to the parameter values and to the
   * x value
   *
   * J = [dy/da, dy/db] for params [a, b] for example
   *
   * @param x: the value at which to evaluate the Jacobian
   *
   * @return the evaluated Jacobian
   */
  virtual Eigen::VectorXd jacobian(const double x) const {
    throw std::logic_error("Jacobian not implemented");
  }

  /**
   * @brief Evaluates the inverse of the saturation function
   *
   * hi(y) = x
   *
   * @param y: the value at which to evaluate the inverse
   *
   * @return the evaluated inverse
   */
  virtual double inverse(const double y) const = 0;

  /**
   * @brief Returns the asymptotic max of the saturation function
   *
   * @return the asymptotic max of teh error function
   */
  virtual double max() const = 0;

 protected:
  Eigen::VectorXd params_;
};  // namespace mag_manip

}  // namespace mag_manip
