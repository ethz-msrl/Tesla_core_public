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

#include "mag_manip/types.h"

namespace mag_manip {

/**
 * @brief Converts a 3D gradient matrix to a 5D gradient vector
 *
 * @param grad_mat: the 3D gradient matrix
 *
 * @return the 5D gradient vector
 */
inline Gradient5Vec gradientMatToGradient5Vec(const GradientMat& grad_mat) {
  Gradient5Vec vec;
  vec << grad_mat(0, 0), grad_mat(0, 1), grad_mat(0, 2), grad_mat(1, 1), grad_mat(1, 2);
  return vec;
}

/**
 * @brief Converts a 5D gradient vector into a 3x3 gradient matrix
 *
 * @param grad5_vec: the gradient vector
 *
 * @return the gradient matrix
 */
inline GradientMat gradient5VecToGradientMat(const Gradient5Vec& grad5_vec) {
  GradientMat mat;
  mat << grad5_vec(0), grad5_vec(1), grad5_vec(2), grad5_vec(1), grad5_vec(3), grad5_vec(4),
      grad5_vec(2), grad5_vec(4), -grad5_vec(0) - grad5_vec(3);
  return mat;
}

/**
 * @brief Gets the matrix representation of a directed gradient
 *
 * @param v: 3D direction vector
 *
 * @return a 3x5 matrix converting a Gradient5Vec into a directed Gradient3Vec
 */
inline Grad35Mat directedGradient3Mat(const DipoleVec& v) {
  Grad35Mat M;
  M << v(0), v(1), v(2), 0, 0, 0, v(0), 0, v(1), v(2), -v(2), 0, v(0), -v(2), v(1);
  return M;
}

/**
 * @brief converts a vector to a skew-symmetric matrix
 *
 * The skew-symmetric matrix is useful for representing a cross-product as a matrix multiplication
 * a x b = S(a) * b
 *
 * @param v: a 3D vector
 *
 * @return the skew-symmetric matrix associated with v
 */
inline Eigen::Matrix3d skewSymmetric(const Eigen::Vector3d& v) {
  Eigen::Matrix3d m;
  m << 0, -v(2), v(1), v(2), 0, -v(0), -v(1), v(0), 0;
  return m;
}

/**
 * @brief Checks if a 3D matrix is skew-symmetric
 *
 * @param m: a 3D matrix
 *
 * @return true if m is skew symmetric
 */
inline bool isSkewSymmetric(Eigen::Matrix3d m) { return m.transpose().isApprox(-m); }

/**
 * @brief Takes a skew-symmetric matrix and converts it to a vector form
 *
 * WARNING: This does not check that m is skew-symmetric. You can use isSkewSymmetric to check.
 *
 * @param m: a 3D skew-symmetric matrix
 *
 * @return the vector associated with m
 */
inline Eigen::Vector3d fromSkewSymmetric(const Eigen::Matrix3d& m) {
  Eigen::Vector3d v;
  v << -m(1, 2), m(0, 2), m(1, 0);
  return v;
}

/**
 * @brief Returns a 3D gradient vector aligned in the direction of a dipole vector
 *
 * Warning dipole need not be a unit vector and is normalized
 *
 * @param dipole: a vector representing the direction in which the gradient is calculated
 * @param gradient: a 3D matrix containing the gradient
 *
 * @return a 3D vector with the dipole aligned gradient
 */
inline Gradient3Vec alignedGradientFromMat(const DipoleVec& dipole, const GradientMat& gradient) {
  DipoleVec dipole_n = dipole / dipole.norm();
  return gradient * dipole_n;
}

/**
 * @brief Returns a 3D gradient vector aligned in the direction of a dipole vector
 *
 * Warning dipole need not be a unit vector and is normalized
 *
 * @param dipole: a vector representing the direction in which the gradient is calculated
 * @param gradient: a 5D vector containing the gradient
 *
 * @return a 3D vector with the dipole aligned gradient
 */
inline Gradient3Vec alignedGradientFromGrad5(const DipoleVec& dipole,
                                             const Gradient5Vec& gradient) {
  DipoleVec dipole_n = dipole / dipole.norm();
  return directedGradient3Mat(dipole_n) * gradient;
}

/**
 * @brief Returns a 3D gradient vector aligned in the direction of a dipole vector
 *
 * This function is depracated and aligendGradientFromGrad5 should be used instead.
 * It will be removed in a future version
 *
 * Warning dipole need not be a unit vector and is normalized
 *
 * @param dipole: a vector representing the direction in which the gradient is calculated
 * @param gradient: a 5D vector containing the gradient
 *
 * @return a 3D vector with the dipole aligned gradient
 */
inline Gradient3Vec alignedGradient(const DipoleVec& dipole, const Gradient5Vec& gradient) {
  return alignedGradientFromGrad5(dipole, gradient);
}

}  // namespace mag_manip
