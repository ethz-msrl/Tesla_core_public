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

#pragma once

#include <cmath>
#include <exception>

namespace mag_manip {

inline SaturationRational::SaturationRational(const Eigen::VectorXd& params)
    : SaturationFunction(params) {
  if (params_.size() != 2) {
    throw std::runtime_error("Rational saturation function needs 2 parameters");
  }
}

inline double SaturationRational::evaluate(const double x) const {
  return params_(0) * x / (std::abs(x) + params_(1));
}

inline Eigen::VectorXd SaturationRational::jacobian(const double x) const {
  const double d = std::abs(x) + params_(1);
  Eigen::Vector3d jacobian;
  jacobian << params_(0) * params_(1) / (d * d), x / d, -params_(0) * x / (d * d);
  return jacobian;
}

inline double SaturationRational::inverse(const double y) const {
  if (y < 0)
    return params_(1) * y / (params_(0) + y);
  else
    return params_(1) * y / (params_(0) - y);
}

inline double SaturationRational::max() const { return params_(0); }

}  // namespace mag_manip
