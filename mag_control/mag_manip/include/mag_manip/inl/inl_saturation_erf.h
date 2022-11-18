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

#define _USE_MATH_DEFINES

#include <math.h>
#include <exception>

namespace mag_manip {

inline SaturationErf::SaturationErf(const Eigen::VectorXd& params) : SaturationFunction(params) {
  if (params_.size() != 2) {
    throw std::runtime_error("Error function saturation function needs 2 parameters");
  }
}

inline double SaturationErf::evaluate(const double x) const {
  return params_(0) * erf(params_(1) * x);
}

inline Eigen::VectorXd SaturationErf::jacobian(const double x) const {
  Eigen::Vector3d jacobian;
  const double ab2_sqrtpi = M_2_SQRTPI * params_(0) * params_(1);
  const double gau = exp(-params_(1) * params_(1) * x * x);
  jacobian << M_2_SQRTPI * params_(0) * params_(1) * gau, erf(params_(1) * x),
      M_2_SQRTPI * params_(0) * x * gau;
  return jacobian;
}

inline double SaturationErf::inverse(const double y) const {
  // Based on "A handy approximation for the error function and its
  // inverse" by Sergei Winitzki.
  // from
  // https://stackoverflow.com/questions/27229371/inverse-error-function-in-c
  float y_ = y / params_(0);
  float tt1, tt2, lny, sgn;
  sgn = (y_ < 0) ? -1.0f : 1.0f;

  y_ = (1 - y_) * (1 + y_);  // x = 1 - x*x;
  lny = logf(y_);

  tt1 = 2 / (M_PI * 0.147) + 0.5f * lny;
  tt2 = 1 / (0.147) * lny;

  return (sgn * sqrtf(-tt1 + sqrtf(tt1 * tt1 - tt2))) / params_(1);
}

inline double SaturationErf::max() const { return params_(0); }
}  // namespace mag_manip
