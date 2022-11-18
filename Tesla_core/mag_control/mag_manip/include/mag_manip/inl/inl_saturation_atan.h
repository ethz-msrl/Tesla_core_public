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

#include <math.h>
#include <exception>

namespace mag_manip {

inline SaturationAtan::SaturationAtan(const Eigen::VectorXd& params) : SaturationFunction(params) {
  if (params_.size() != 2) {
    throw std::runtime_error("Atan saturation function needs 2 parameters");
  }
}

inline double SaturationAtan::evaluate(const double x) const {
  return params_(0) * atan(params_(1) * x);
}

inline Eigen::VectorXd SaturationAtan::jacobian(const double x) const {
  Eigen::Vector3d jacobian;
  const double a_b2_x2_1 = params_(0) / (params_(1) * params_(1) * x * x + 1);
  jacobian << params_(1) * a_b2_x2_1, atan(params_(1) * x), x * a_b2_x2_1;

  return jacobian;
}

inline double SaturationAtan::inverse(const double y) const {
  return tan(y / params_(0)) / params_(1);
}

inline double SaturationAtan::max() const { return params_(0) * M_PI_2; }

}  // namespace mag_manip
