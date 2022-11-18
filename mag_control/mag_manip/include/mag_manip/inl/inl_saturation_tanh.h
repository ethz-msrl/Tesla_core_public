#pragma once

#include <math.h>

/* * Tesla - A ROS-based framework for performing magnetic manipulation
 *
 * Copyright 2018 Multi Scale Robotics Lab
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

namespace mag_manip {
inline SaturationTanh::SaturationTanh(const Eigen::VectorXd& params) : SaturationFunction(params) {
  if (params_.size() != 2) {
    throw std::runtime_error("Tanh saturation function needs 2 parameters");
  }
}

inline double SaturationTanh::evaluate(const double x) const {
  return params_(0) * tanh(params_(1) * x);
}

inline Eigen::VectorXd SaturationTanh::jacobian(const double x) const {
  Eigen::Vector3d jacobian;
  const double a_b = params_(0) * params_(1);
  const double cosh_bx2 = cosh(params_(1) * x) * cosh(params_(1) * x);
  jacobian << a_b / cosh_bx2, tanh(params_(1) * x), params_(0) * x / cosh_bx2;

  return jacobian;
}
inline double SaturationTanh::inverse(const double y) const {
  return atanh(y / params_(0)) / params_(1);
}

inline double SaturationTanh::max() const { return params_(0); }

}  // namespace mag_manip
