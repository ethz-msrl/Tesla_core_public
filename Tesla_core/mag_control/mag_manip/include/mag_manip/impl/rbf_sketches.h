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
#include "mag_manip/rbf.h"

/**
 * @brief This duplicates the PolyharmonicSplineInterpolator in rbf.h but was used to compare an
 * implementation with fixed size matrices and one with heap allocated matrices. There doesn't
 * appear to be a performance difference so we are reverting to the dynamic implementation
 *
 */
namespace mag_manip {
template <typename Scalar, int D, int N, int Dv, typename RBFPolicy>
class PolyharmonicSplineInterpolatorFixed : private RBFPolicy {
  using Nodes = Eigen::Matrix<Scalar, D, N>;
  using Values = Eigen::Matrix<Scalar, Dv, N>;
  using ValuesT = Eigen::Matrix<Scalar, N, Dv>;
  using CoeffsRBF = Eigen::Matrix<Scalar, N, Dv>;
  // how to deal with this when there D is Eigen::Dynamic?
  using CoeffsPoly = Eigen::Matrix<Scalar, D + 1, Dv>;
  using LType = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>;
  using WType = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>;

 public:
  PolyharmonicSplineInterpolatorFixed(const Nodes& nodes, const Values& values)
      : nodes_(nodes), values_t_(values.transpose()), L_(N + D + 1, N + D + 1), W_(N + D + 1, Dv) {
    getCoeffs();
  }

  template <typename PositionsType>
  Eigen::Matrix<Scalar, Dv, PositionsType::ColsAtCompileTime> operator()(
      const Eigen::MatrixBase<PositionsType>& positions) const {
    const int Np = PositionsType::ColsAtCompileTime;
    Eigen::Matrix<Scalar, Np, N> psi = getKernels(positions);
    Eigen::Matrix<Scalar, D + 1, Np> w;
    w << Eigen::Matrix<Scalar, 1, Np>::Ones(), positions;
    return (psi * coeffs_rbf_).transpose() + (coeffs_poly_.transpose() * w);
  }

 private:
  void setupLinearProblem() {
    Eigen::Matrix<Scalar, N, N> Psi = getKernels(nodes_);
    Eigen::Matrix<Scalar, D + 1, N> B;
    B << Eigen::Matrix<Scalar, 1, N>::Ones(), nodes_;
    L_ = Eigen::MatrixXd::Zero(N + D + 1, N + D + 1);
    L_.block(N, N, 0, 0) = Psi;
    L_.block(N, D + 1, 0, N) = B.transpose();
    L_.block(D + 1, N, N, 0) = B;
    W_.resize(N + D + 1, Dv);
    W_ << values_t_, Eigen::Matrix<Scalar, D + 1, Dv>::Zero();
  }

  void getCoeffs(std::integral_constant<bool, true>) {
    setupLinearProblem();
    Eigen::Matrix<Scalar, N + D + 1, Dv> coeffs_grouped = L_.llt().solve(W_);
    coeffs_rbf_ = coeffs_grouped.topRows(N);
    coeffs_poly_ = coeffs_grouped.bottomRows(D + 1);
  }

  void getCoeffs(std::integral_constant<bool, false>) {
    setupLinearProblem();
    Eigen::Matrix<Scalar, D + 1, Dv> coeffs_grouped = L_.lu().solve(W_);
    coeffs_rbf_ = coeffs_grouped.topRows(N);
    coeffs_poly_ = coeffs_grouped.bottomRows(D + 1);
  }

  void getCoeffs() { getCoeffs(IsPositiveDefinite<RBFPolicy>()); }

  template <typename PositionsType>
  Eigen::Matrix<Scalar, PositionsType::ColsAtCompileTime, N> getKernels(
      const Eigen::MatrixBase<PositionsType>& positions) const {
    return RBFPolicy::getKernels(positions, nodes_, 1.0);
  }

  Nodes nodes_;
  ValuesT values_t_;
  CoeffsRBF coeffs_rbf_;
  CoeffsPoly coeffs_poly_;
  LType L_;
  WType W_;
};
}  // namespace mag_manip
