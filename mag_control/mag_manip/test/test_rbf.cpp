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

#include <gtest/gtest.h>
#include <ros/package.h>
#include <tsc_utils/eigen_matrix_compare.h>
#include <tsc_utils/eigen_tensor_compare.h>

#include <unsupported/Eigen/CXX11/Tensor>

#include "mag_manip/rbf.h"

using namespace mag_manip;
using namespace std;
using namespace Eigen;

TEST(Eigen, Tensors) {
  // This test was used during the development of RBFInterpolator::getGradients
  const int Np = 100;
  const int D = 3;
  const int N = 125;
  const int Dv = 3;
  const double eps = 1.;
  Eigen::MatrixXd positions(D, Np);
  positions.setRandom();
  Eigen::MatrixXd nodes(D, N);
  nodes.setRandom();
  Eigen::MatrixXd coeffs(N, Dv);
  coeffs.setRandom();

  Eigen::MatrixXd distances = RBFGaussianPolicy::getDistanceToNodes(positions, nodes);
  auto f = [eps](double x) { return RBFGaussianPolicy::getKernelDerivativeUnary(x, eps); };
  MatrixXd psi_dot = distances.unaryExpr(f);

  Eigen::TensorMap<Eigen::Tensor<double, 2> > t_psi_dot(psi_dot.data(), Np, N);

  // expend t_psi_dot_e
  Eigen::Tensor<double, 3> t_psi_dot_e(D, Np, N);
  for (int d = 0; d < D; d++) {
    Eigen::array<int, 3> offsets = {d, 0, 0};
    Eigen::array<int, 3> extents = {1, Np, N};
    t_psi_dot_e.slice(offsets, extents) = t_psi_dot;
  }

  EXPECT_EQ(t_psi_dot_e.dimension(0), D);
  EXPECT_EQ(t_psi_dot_e.dimension(1), Np);
  EXPECT_EQ(t_psi_dot_e.dimension(2), N);

  Eigen::Tensor<double, 3> t_dvectors(D, Np, N);
  for (int d = 0; d < D; d++) {
    for (int i = 0; i < Np; i++) {
      for (int j = 0; j < N; j++) {
        t_dvectors(d, i, j) = positions(d, i) - nodes(d, j);
      }
    }
  }

  Eigen::Tensor<double, 2> t_dvectors_norm =
      t_dvectors.square().sum(Eigen::array<int, 1>({0})).sqrt();
  EXPECT_EQ(t_dvectors_norm.dimension(0), Np);
  EXPECT_EQ(t_dvectors_norm.dimension(1), N);

  Eigen::Tensor<double, 3> t_dvectors_norm_e_(D, Np, N);
  for (int d = 0; d < D; d++) {
    Eigen::array<int, 3> offsets = {d, 0, 0};
    Eigen::array<int, 3> extents = {1, Np, N};
    t_dvectors_norm_e_.slice(offsets, extents) = t_dvectors_norm;
  }

  Eigen::Tensor<double, 3> lhs = t_psi_dot_e * (t_dvectors / t_dvectors_norm_e_);

  // EXPECT_EQ(lhs.NumDimensions, 3);
  EXPECT_EQ(lhs.dimension(0), D);
  EXPECT_EQ(lhs.dimension(1), Np);
  EXPECT_EQ(lhs.dimension(2), N);

  Eigen::TensorMap<Eigen::Tensor<double, 2> > t_coeffs(coeffs.data(), N, Dv);
  Eigen::array<Eigen::IndexPair<int>, 1> product_dims = {Eigen::IndexPair<int>(2, 0)};
  Eigen::Tensor<double, 3> derivatives = lhs.contract(t_coeffs, product_dims);

  EXPECT_EQ(derivatives.dimension(0), D);
  EXPECT_EQ(derivatives.dimension(1), Np);
  EXPECT_EQ(derivatives.dimension(2), Dv);
}

TEST(Eigen, getKernelsTensor) {
  const int N = 125;
  const int D = 3;
  const double eps = 1.0;
  const int Np = 100;
  using VectorN = Eigen::Matrix<double, N, 1>;
  Eigen::MatrixXd nodes(D, N);
  nodes.setRandom();
  Eigen::MatrixXd positions(D, Np);
  positions.setRandom();

  using Tensor3d = Eigen::Tensor<double, 3>;
  using Tensor2d = Eigen::Tensor<double, 2>;

  Tensor3d diffs = RBFPolicyBase<void>::getPositionToNodes(positions, nodes);
  Eigen::array<int, 1> dims({0});
  Tensor2d temp = diffs.square().sum(dims) * (-eps);
  Tensor2d out = temp.exp();

  EXPECT_EQ(out.dimension(0), Np);
  EXPECT_EQ(out.dimension(1), N);

  Eigen::Map<Eigen::MatrixXd> out_m(out.data(), Np, N);

  Eigen::MatrixXd kernels(Np, N);
  for (int i = 0; i < Np; i++) {
    VectorN temp = (nodes.colwise() - positions.col(i)).colwise().squaredNorm();
    kernels.row(i) = (-eps * temp).array().exp();
  }

  EXPECT_TRUE(EIGEN_MATRIX_NEAR_ABS(kernels, out_m, 1e-4));
}

TEST(RBFPolicyBase, getDistanceToNodes) {
  struct DummyPolicy;
  const int D = 3;
  const int N = 125;
  const int Np = 500;
  Eigen::MatrixXd nodes(D, N);
  nodes.setRandom();

  Eigen::MatrixXd positions(D, Np);
  positions.setRandom();
  positions.col(0) = nodes.col(0) + Eigen::Vector3d(1, 0, 0);
  Eigen::MatrixXd distances = RBFPolicyBase<DummyPolicy>::getDistanceToNodes(positions, nodes);
  EXPECT_EQ(distances(0, 0), 1.);
}

TEST(RBFPolicyBase, getPositionToNodes) {
  struct DummyPolicy;
  const int D = 3;
  const int N = 125;
  const int Np = 500;
  Eigen::MatrixXd nodes(D, N);
  nodes.setRandom();

  Eigen::MatrixXd positions(D, Np);
  positions.setRandom();
  positions.col(0) = nodes.col(0) + Eigen::Vector3d(1, 0, 0);
  Eigen::Tensor<double, 3> t_dvectors =
      RBFPolicyBase<DummyPolicy>::getPositionToNodes(positions, nodes);
  EXPECT_EQ(t_dvectors.dimension(0), D);
  EXPECT_EQ(t_dvectors.dimension(1), Np);
  EXPECT_EQ(t_dvectors.dimension(2), N);
  EXPECT_EQ(t_dvectors(0, 0, 0), 1);
  EXPECT_EQ(t_dvectors(1, 0, 0), 0);
  EXPECT_EQ(t_dvectors(2, 0, 0), 0);
}

TEST(RBFGaussianPolicy, getKernel_dynamicSize) {
  const int d = 3;
  const int N = 125;
  const double eps = 1.;
  Eigen::MatrixXd nodes = Eigen::MatrixXd::Random(d, N);
  Eigen::VectorXd position = nodes.col(0);
  Eigen::VectorXd K = RBFGaussianPolicy::getKernel(position, nodes, eps);
  ASSERT_EQ(K.size(), nodes.cols());
  EXPECT_EQ(K(0), 1);
}

TEST(RBFGaussianPolicy, getKernel_fixedSize) {
  Eigen::Matrix<double, 3, 125> nodes;
  nodes.setRandom();
  const double eps = 1.;
  Eigen::Vector3d position = nodes.col(0);
  Eigen::Matrix<double, 125, 1> K = RBFGaussianPolicy::getKernel(position, nodes, eps);
  ASSERT_EQ(K.size(), nodes.cols());
  EXPECT_EQ(K(0), 1);
}

TEST(RBFGaussianPolicy, getKernels_Dynamic) {
  const int d = 3;
  const int N = 125;
  const int Np = 120;
  const double eps = 1.;
  Eigen::MatrixXd nodes = Eigen::MatrixXd::Random(d, N);
  Eigen::MatrixXd positions = nodes.leftCols<Np>();
  Eigen::MatrixXd K = RBFGaussianPolicy::getKernels(positions, nodes, eps);
  ASSERT_EQ(K.rows(), Np);
  ASSERT_EQ(K.cols(), N);
  for (int i = 0; i < Np; i++) {
    EXPECT_EQ(K(i, i), 1);
  }
}

TEST(RBFGaussianPolicy, multiplePositionsFixedSize) {
  const int D = 3;
  const int N = 125;
  const int Np = 120;
  const double eps = 1.;
  Eigen::Matrix<double, D, N> nodes;
  nodes.setRandom();
  Eigen::Matrix<double, D, Np> positions = nodes.leftCols<Np>();
  Eigen::Matrix<double, Np, N> K = RBFGaussianPolicy::getKernels(positions, nodes, eps);
  for (int i = 0; i < Np; i++) {
    EXPECT_EQ(K(i, i), 1);
  }
}

TEST(RBFGaussianPolicy, unaryDerivative) {
  EXPECT_EQ(RBFGaussianPolicy::getKernelDerivativeUnary(0.0, 1.0), -2.0);
  const double x = 0.5;
  const double eps = 2.0;
  double dk = RBFGaussianPolicy::getKernelDerivativeUnary(x, eps);

  double k = RBFGaussianPolicy::getKernelUnary(x, eps);
  double dk_ = (RBFGaussianPolicy::getKernelUnary(x + 1e-6, eps) - k) / 1e-6;
  EXPECT_NEAR(dk * x, dk_, 1e-4);
}

TEST(RBFGaussianPolicy, getKernelDerivatives) {
  const int D = 3;
  const int N = 125;
  const int Np = 120;
  const double eps = 1.;
  Eigen::MatrixXd nodes = Eigen::MatrixXd::Random(D, N);
  Eigen::MatrixXd positions = Eigen::MatrixXd::Random(D, Np);
  Eigen::MatrixXd K = RBFGaussianPolicy::getKernels(positions, nodes, eps);
  Eigen::MatrixXd dK = RBFGaussianPolicy::getKernelDerivatives(positions, nodes, eps);
  ASSERT_EQ(K.rows(), Np);
  ASSERT_EQ(K.cols(), N);

  Eigen::MatrixXd dist = RBFGaussianPolicy::getDistanceToNodes(positions, nodes);
  auto f = [eps](double x) { return RBFGaussianPolicy::getKernelDerivativeUnary(x, eps); };
  Eigen::MatrixXd dK_ = dist.unaryExpr(f);

  EXPECT_TRUE(EIGEN_MATRIX_NEAR_ABS(dK, dK_, 1e-8));
}

TEST(RBFInterpolatorGaussian, getCoeffsFixed) {
  const int D = 3;
  const int N = 125;
  const double eps = 1.;
  Eigen::Matrix<double, D, N> nodes;
  nodes.setRandom();

  Eigen::Matrix<double, N, 1> values;
  values.setRandom();
  using GaussianInterp = RBFInterpolator<double, D, 1, RBFGaussianPolicy>;
  GaussianInterp interp(nodes, values, eps);

  Eigen::Matrix<double, N, 1> coeffs = interp.getCoeffs();
  Eigen::Matrix<double, N, N> psi = RBFGaussianPolicy::getKernels(nodes, nodes, eps);
  EXPECT_TRUE(EIGEN_MATRIX_NEAR_ABS(psi * coeffs, values, 1e-4));
}

TEST(RBFInterpolatorGaussian, getCoeffsDynamic) {
  const int D = 3;
  const int N = 125;
  const double eps = 1.;
  Eigen::MatrixXd nodes(D, N);
  nodes.setRandom();

  Eigen::VectorXd values(N, 1);
  values.setRandom();
  using GaussianInterp = RBFInterpolator<double, D, 1, RBFGaussianPolicy>;
  GaussianInterp interp(nodes, values, eps);

  Eigen::VectorXd coeffs = interp.getCoeffs();
  Eigen::MatrixXd psi = RBFGaussianPolicy::getKernels(nodes, nodes, eps);
  EXPECT_TRUE(EIGEN_MATRIX_NEAR_ABS(psi * coeffs, values, 1e-4));
}

TEST(RBFInterpolatorGaussian, getCoeffsMultiCoeffsDim) {
  const int D = 3;
  const int Dv = 3;
  const int N = 125;
  const double eps = 1.;
  Eigen::Matrix<double, D, N> nodes;
  nodes.setRandom();

  Eigen::Matrix<double, Dv, N> values;
  values.setRandom();
  using GaussianInterp = RBFInterpolator<double, D, Dv, RBFGaussianPolicy>;
  GaussianInterp interp(nodes, values, eps);

  Eigen::Matrix<double, N, Dv> coeffs = interp.getCoeffs();
  Eigen::Matrix<double, N, N> psi = RBFGaussianPolicy::getKernels(nodes, nodes, eps);
  EXPECT_TRUE(EIGEN_MATRIX_NEAR_ABS(psi * coeffs, values.transpose(), 1e-4));
}

TEST(RBFInterpolatorGaussian, interpolate) {
  const int D = 3;
  const int Dv = 3;
  const int N = 125;
  const int Np = 120;
  const double eps = 1.;
  using GaussianInterp = RBFInterpolator<double, D, Dv, RBFGaussianPolicy>;
  Eigen::Matrix<double, D, N> nodes;
  nodes.setRandom();

  Eigen::Matrix<double, Dv, N> values;
  values.setRandom();

  Eigen::Matrix<double, D, Np> positions = nodes.leftCols<Np>();

  GaussianInterp interp(nodes, values, eps);
  Eigen::Matrix<double, Dv, Np> interpolated = interp(positions);

  EXPECT_TRUE(EIGEN_MATRIX_NEAR_ABS(interpolated, values.leftCols<Np>(), 1e-4));
}

TEST(RBFInterpolatorGaussian, interpolateDynamic) {
  const int D = 3;
  const int Dv = 3;
  const int N = 125;
  const int Np = 120;
  const double eps = 1.;
  using GaussianInterp = RBFInterpolator<double, D, Dv, RBFGaussianPolicy>;
  Eigen::MatrixXd nodes(D, N);
  nodes.setRandom();

  Eigen::MatrixXd values(Dv, N);
  values.setRandom();

  Eigen::MatrixXd positions = nodes.leftCols<Np>();

  GaussianInterp interp(nodes, values, eps);
  Eigen::MatrixXd interpolated = interp(positions);

  EXPECT_TRUE(EIGEN_MATRIX_NEAR_ABS(interpolated, values.leftCols<Np>(), 1e-4));
}

TEST(RBFInterpolatorGaussian, getGradients) {
  const int D = 3;
  const int Dv = 3;
  const int N = 125;
  const int Np = 100;
  const double eps = 1.;
  using GaussianInterp = RBFInterpolator<double, D, Dv, RBFGaussianPolicy>;
  Eigen::MatrixXd nodes(D, N);
  nodes.setRandom();

  Eigen::MatrixXd values(Dv, N);
  values.setRandom();

  // Eigen::Matrix<double, D, Np> positions = nodes.leftCols<Np>();
  Eigen::MatrixXd positions(D, Np);
  positions.setRandom();

  GaussianInterp interp(nodes, values, eps);
  Eigen::MatrixXd y = interp(positions);
  Eigen::Tensor<double, 3> dK = interp.getGradients(positions);

  Eigen::Tensor<double, 3> dK_(Dv, D, Np);
  Eigen::Matrix<double, D, D> ident;
  ident.setIdentity();
  const double deps = 1e-6;
  for (int i = 0; i < Np; i++) {
    for (int d = 0; d < D; d++) {
      Eigen::Matrix<double, D, 1> dp = positions.col(i) + ident.col(d) * deps;
      Eigen::Matrix<double, Dv, 1> K_ = interp(dp);
      Eigen::array<int, 3> offsets({d, 0, i});
      Eigen::array<int, 3> extents({1, Dv, 1});
      Eigen::Matrix<double, Dv, 1> diff = (K_.col(0) - y.col(i)) / deps;
      {
        Eigen::TensorMap<Eigen::Tensor<double, 3> > t_diff(diff.data(), Dv, 1, 1);
        Eigen::array<int, 3> offsets = {0, i, d};
        Eigen::array<int, 3> extents = {Dv, 1, 1};
        dK_.slice(offsets, extents) = t_diff;
      }
    }
  }

  EXPECT_TRUE(EIGEN_TENSOR3_NEAR_REL(dK, dK_, 1e-4));
}

TEST(RBFInterpolatorGaussian, getLhs) {
  const int D = 3;
  const int Dv = 3;
  const int N = 125;
  const int Np = 100;
  const double eps = 1.;
  using GaussianInterp = RBFInterpolator<double, D, Dv, RBFGaussianPolicy>;
  Eigen::MatrixXd nodes(D, N);
  nodes.setRandom();

  using Tensor3d = Eigen::Tensor<double, 3>;
  using Tensor2d = Eigen::Tensor<double, 2>;

  Eigen::MatrixXd values(Dv, N);
  values.setRandom();

  // Eigen::Matrix<double, D, Np> positions = nodes.leftCols<Np>();
  Eigen::MatrixXd positions(D, Np);
  positions.setRandom();

  GaussianInterp interp(nodes, values, eps);
  Eigen::MatrixXd y = interp(positions);
  Eigen::Matrix<double, D, D> ident;
  ident.setIdentity();
  const double deps = 1e-6;

  Tensor3d t_lhs = RBFPolicyBase<RBFGaussianPolicy>::getGradientOperand(positions, nodes, eps);

  Tensor3d t_lhs_(D, Np, N);

  for (int i = 0; i < Np; i++) {
    auto diffs = (nodes.array() * (-1)).matrix().colwise() + positions.col(i);
    ASSERT_EQ(diffs.matrix().cols(), N);
    ASSERT_EQ(diffs.matrix().rows(), D);
    auto dist = diffs.colwise().norm().transpose();
    auto dist2 = diffs.colwise().squaredNorm().transpose();
    ASSERT_EQ(dist.matrix().rows(), N);
    ASSERT_EQ(dist.matrix().cols(), 1);
    // N x 1
    auto xpart = dist.array() * (-2 * eps);
    ASSERT_EQ(xpart.matrix().rows(), N);
    ASSERT_EQ(xpart.matrix().cols(), 1);
    Eigen::VectorXd psi_dot = xpart.array() * (-eps * dist2).array().exp();
    ASSERT_EQ(psi_dot.rows(), N);
    Eigen::VectorXd n_diffs = diffs.colwise().norm();
    auto diffs_n = diffs.matrix().array().rowwise() / n_diffs.transpose().array();
    ASSERT_EQ(diffs_n.matrix().cols(), N);
    ASSERT_EQ(diffs_n.matrix().rows(), D);
    // N x 1 * D x N = D x N
    Eigen::array<int, 3> offsets = {0, i, 0};
    Eigen::array<int, 3> extents = {D, 1, N};
    Eigen::MatrixXd lhs = diffs_n.matrix() * psi_dot.asDiagonal();
    ASSERT_EQ(lhs.rows(), D);
    ASSERT_EQ(lhs.cols(), N);
    t_lhs_.slice(offsets, extents) = Eigen::TensorMap<Tensor2d>(lhs.data(), D, N);
  }

  EXPECT_TRUE(EIGEN_TENSOR3_NEAR_ABS(t_lhs, t_lhs_, 1e-6));
}

TEST(RBFMultiquadricPolicy, getKernel_dynamicSize) {
  const int D = 3;
  const int N = 125;
  const double eps = 1.;
  Eigen::MatrixXd nodes = Eigen::MatrixXd::Random(D, N);
  Eigen::VectorXd position = nodes.col(0);
  Eigen::VectorXd K = RBFMultiquadricPolicy::getKernel(position, nodes, eps);
  ASSERT_EQ(K.size(), nodes.cols());
  EXPECT_EQ(K(0), 1);
}

TEST(RBFMultiquadricPolicy, getKernel_fixedSize) {
  Eigen::Matrix<double, 3, 125> nodes;
  nodes.setRandom();
  const double eps = 1.;
  Eigen::Vector3d position = nodes.col(0);
  Eigen::Matrix<double, 125, 1> K = RBFMultiquadricPolicy::getKernel(position, nodes, eps);
  ASSERT_EQ(K.size(), nodes.cols());
  EXPECT_EQ(K(0), 1);
}

TEST(RBFMultiquadricPolicy, unaryDerivative) {
  EXPECT_EQ(RBFMultiquadricPolicy::getKernelDerivativeUnary(0.0, 1.0), 1.0);
  const double x = 0.5;
  const double eps = 2.0;
  double dk_x = RBFMultiquadricPolicy::getKernelDerivativeUnary(x, eps);

  double k = RBFMultiquadricPolicy::getKernelUnary(x, eps);
  double dk_ = (RBFMultiquadricPolicy::getKernelUnary(x + 1e-6, eps) - k) / 1e-6;
  EXPECT_NEAR(x * dk_x, dk_, 1e-4);
}

TEST(RBFMultiquadricPolicy, getKernelDerivatives) {
  const int D = 3;
  const int N = 125;
  const int Np = 120;
  const double eps = 1.;
  Eigen::MatrixXd nodes = Eigen::MatrixXd::Random(D, N);
  Eigen::MatrixXd positions = Eigen::MatrixXd::Random(D, Np);
  Eigen::MatrixXd K = RBFMultiquadricPolicy::getKernels(positions, nodes, eps);
  Eigen::MatrixXd dK = RBFMultiquadricPolicy::getKernelDerivatives(positions, nodes, eps);
  ASSERT_EQ(K.rows(), Np);
  ASSERT_EQ(K.cols(), N);

  Eigen::MatrixXd dist = RBFMultiquadricPolicy::getDistanceToNodes(positions, nodes);
  auto f = [eps](double x) { return RBFMultiquadricPolicy::getKernelDerivativeUnary(x, eps); };
  Eigen::MatrixXd dK_ = dist.unaryExpr(f);

  EXPECT_TRUE(EIGEN_MATRIX_NEAR_ABS(dK, dK_, 1e-8));
}

TEST(RBFInterpolatorMultiquadric, getCoeffsFixed) {
  const int D = 3;
  const int N = 125;
  const double eps = 1.;
  Eigen::Matrix<double, D, N> nodes;
  nodes.setRandom();

  Eigen::Matrix<double, 1, N> values;
  values.setRandom();
  using MultiquadricInterp = RBFInterpolator<double, D, 1, RBFMultiquadricPolicy>;
  MultiquadricInterp interp(nodes, values, eps);

  Eigen::Matrix<double, N, 1> coeffs = interp.getCoeffs();
  Eigen::Matrix<double, N, N> psi = RBFMultiquadricPolicy::getKernels(nodes, nodes, eps);
  EXPECT_TRUE(EIGEN_MATRIX_NEAR_ABS(psi * coeffs, values.transpose(), 1e-4));
}

TEST(RBFInterpolatorMultiquadric, interpolate) {
  const int D = 3;
  const int Dv = 3;
  const int N = 125;
  const int Np = 120;
  const double eps = 1.;
  using MultiquadricInterp = RBFInterpolator<double, D, Dv, RBFMultiquadricPolicy>;
  Eigen::Matrix<double, D, N> nodes;
  nodes.setRandom();

  Eigen::Matrix<double, Dv, N> values;
  values.setRandom();

  Eigen::Matrix<double, D, Np> positions = nodes.leftCols<Np>();

  MultiquadricInterp interp(nodes, values, eps);
  Eigen::Matrix<double, Dv, Np> interpolated = interp(positions);

  EXPECT_TRUE(EIGEN_MATRIX_NEAR_ABS(interpolated, values.leftCols<Np>(), 1e-4));
}

TEST(RBFInterpolatorMultiquadric, getGradients) {
  const int D = 3;
  const int Dv = 3;
  const int N = 125;
  const int Np = 100;
  const double eps = 1.;
  using MultiquadricInterp = RBFInterpolator<double, D, Dv, RBFMultiquadricPolicy>;
  Eigen::MatrixXd nodes(D, N);
  nodes.setRandom();

  Eigen::MatrixXd values(Dv, N);
  values.setRandom();

  // Eigen::Matrix<double, D, Np> positions = nodes.leftCols<Np>();
  Eigen::MatrixXd positions(D, Np);
  positions.setRandom();

  MultiquadricInterp interp(nodes, values, eps);
  Eigen::MatrixXd y = interp(positions);
  Eigen::Tensor<double, 3> dK = interp.getGradients(positions);

  Eigen::Tensor<double, 3> dK_(Dv, D, Np);
  Eigen::Matrix<double, D, D> ident;
  ident.setIdentity();
  const double deps = 1e-6;
  for (int i = 0; i < Np; i++) {
    for (int d = 0; d < D; d++) {
      Eigen::Matrix<double, D, 1> dp = positions.col(i) + ident.col(d) * deps;
      Eigen::Matrix<double, Dv, 1> K_ = interp(dp);
      Eigen::array<int, 3> offsets({d, 0, i});
      Eigen::array<int, 3> extents({1, Dv, 1});
      Eigen::Matrix<double, Dv, 1> diff = (K_.col(0) - y.col(i)) / deps;
      {
        Eigen::TensorMap<Eigen::Tensor<double, 3> > t_diff(diff.data(), Dv, 1, 1);
        Eigen::array<int, 3> offsets = {0, i, d};
        Eigen::array<int, 3> extents = {Dv, 1, 1};
        dK_.slice(offsets, extents) = t_diff;
      }
    }
  }

  EXPECT_TRUE(EIGEN_TENSOR3_NEAR_REL(dK, dK_, 1e-4));
}

TEST(RBFInverseMultiquadricPolicy, getKernel_dynamicSize) {
  const int d = 3;
  const int N = 125;
  const double eps = 1.;
  Eigen::MatrixXd nodes = Eigen::MatrixXd::Random(d, N);
  Eigen::VectorXd position = nodes.col(0);
  Eigen::VectorXd K = RBFInverseMultiquadricPolicy::getKernel(position, nodes, eps);
  ASSERT_EQ(K.size(), nodes.cols());
  EXPECT_EQ(K(0), 1);
}

TEST(RBFInverseMultiquadricPolicy, getKernel_fixedSize) {
  Eigen::Matrix<double, 3, 125> nodes;
  nodes.setRandom();
  const double eps = 1.;
  Eigen::Vector3d position = nodes.col(0);
  Eigen::Matrix<double, 125, 1> K = RBFInverseMultiquadricPolicy::getKernel(position, nodes, eps);
  ASSERT_EQ(K.size(), nodes.cols());
  EXPECT_EQ(K(0), 1);
}

TEST(RBFInverseMultiquadricPolicy, unaryDerivative) {
  EXPECT_EQ(RBFInverseMultiquadricPolicy::getKernelDerivativeUnary(0.0, 1.0), -1.0);
  const double x = 0.5;
  const double eps = 2.0;
  double dk_x = RBFInverseMultiquadricPolicy::getKernelDerivativeUnary(x, eps);

  double k = RBFInverseMultiquadricPolicy::getKernelUnary(x, eps);
  double dk_ = (RBFInverseMultiquadricPolicy::getKernelUnary(x + 1e-6, eps) - k) / 1e-6;
  EXPECT_NEAR(x * dk_x, dk_, 1e-4);
}

TEST(RBFInverseMultiquadricPolicy, getKernelDerivatives) {
  const int D = 3;
  const int N = 125;
  const int Np = 120;
  const double eps = 1.;
  Eigen::MatrixXd nodes = Eigen::MatrixXd::Random(D, N);
  Eigen::MatrixXd positions = Eigen::MatrixXd::Random(D, Np);
  Eigen::MatrixXd K = RBFInverseMultiquadricPolicy::getKernels(positions, nodes, eps);
  Eigen::MatrixXd dK = RBFInverseMultiquadricPolicy::getKernelDerivatives(positions, nodes, eps);
  ASSERT_EQ(K.rows(), Np);
  ASSERT_EQ(K.cols(), N);

  Eigen::MatrixXd dist = RBFInverseMultiquadricPolicy::getDistanceToNodes(positions, nodes);
  auto f = [eps](double x) {
    return RBFInverseMultiquadricPolicy::getKernelDerivativeUnary(x, eps);
  };
  Eigen::MatrixXd dK_ = dist.unaryExpr(f);

  EXPECT_TRUE(EIGEN_MATRIX_NEAR_ABS(dK, dK_, 1e-8));
}

TEST(RBFInterpolatorInverseMultiquadric, interpolate) {
  const int D = 3;
  const int Dv = 3;
  const int N = 125;
  const int Np = 120;
  const double eps = 1.;
  using InverseMultiquadricInterp = RBFInterpolator<double, D, Dv, RBFInverseMultiquadricPolicy>;
  Eigen::Matrix<double, D, N> nodes;
  nodes.setRandom();

  Eigen::Matrix<double, Dv, N> values;
  values.setRandom();

  Eigen::Matrix<double, D, Np> positions = nodes.leftCols<Np>();

  InverseMultiquadricInterp interp(nodes, values, eps);
  Eigen::Matrix<double, Dv, Np> interpolated = interp(positions);

  EXPECT_TRUE(EIGEN_MATRIX_NEAR_ABS(interpolated, values.leftCols<Np>(), 1e-4));
}

TEST(RBFInterpolatorInverseMultiquadric, getGradients) {
  const int D = 3;
  const int Dv = 3;
  const int N = 125;
  const int Np = 100;
  const double eps = 1.;
  using InverseMultiquadricInterp = RBFInterpolator<double, D, Dv, RBFInverseMultiquadricPolicy>;
  Eigen::MatrixXd nodes(D, N);
  nodes.setRandom();

  Eigen::MatrixXd values(Dv, N);
  values.setRandom();

  // Eigen::Matrix<double, D, Np> positions = nodes.leftCols<Np>();
  Eigen::MatrixXd positions(D, Np);
  positions.setRandom();

  InverseMultiquadricInterp interp(nodes, values, eps);
  Eigen::MatrixXd y = interp(positions);
  Eigen::Tensor<double, 3> dK = interp.getGradients(positions);

  Eigen::Tensor<double, 3> dK_(Dv, D, Np);
  Eigen::Matrix<double, D, D> ident;
  ident.setIdentity();
  const double deps = 1e-6;
  for (int i = 0; i < Np; i++) {
    for (int d = 0; d < D; d++) {
      Eigen::Matrix<double, D, 1> dp = positions.col(i) + ident.col(d) * deps;
      Eigen::Matrix<double, Dv, 1> K_ = interp(dp);
      Eigen::array<int, 3> offsets({d, 0, i});
      Eigen::array<int, 3> extents({1, Dv, 1});
      Eigen::Matrix<double, Dv, 1> diff = (K_.col(0) - y.col(i)) / deps;
      {
        Eigen::TensorMap<Eigen::Tensor<double, 3> > t_diff(diff.data(), Dv, 1, 1);
        Eigen::array<int, 3> offsets = {0, i, d};
        Eigen::array<int, 3> extents = {Dv, 1, 1};
        dK_.slice(offsets, extents) = t_diff;
      }
    }
  }

  EXPECT_TRUE(EIGEN_TENSOR3_NEAR_REL(dK, dK_, 1e-4));
}

TEST(RBFCubicPolicy, getKernel_dynamicSize) {
  const int d = 3;
  const int N = 125;
  const double eps = 1.;
  Eigen::MatrixXd nodes = Eigen::MatrixXd::Random(d, N);
  Eigen::VectorXd position = nodes.col(0);
  Eigen::VectorXd K = RBFCubicPolicy::getKernel(position, nodes, eps);
  ASSERT_EQ(K.size(), nodes.cols());
  EXPECT_EQ(K(0), 0);
}

TEST(RBFCubicPolicy, getKernel_fixedSize) {
  Eigen::Matrix<double, 3, 125> nodes;
  nodes.setRandom();
  const double eps = 1.;
  Eigen::Vector3d position = nodes.col(0);
  Eigen::Matrix<double, 125, 1> K = RBFCubicPolicy::getKernel(position, nodes, eps);
  ASSERT_EQ(K.size(), nodes.cols());
  EXPECT_EQ(K(0), 0);
}

TEST(RBFCubicPolicy, unaryDerivative) {
  EXPECT_EQ(RBFCubicPolicy::getKernelDerivativeUnary(0.0, 1.0), 0.0);
  const double x = 0.5;
  const double eps = 2.0;
  double dk_x = RBFCubicPolicy::getKernelDerivativeUnary(x, eps);

  double k = RBFCubicPolicy::getKernelUnary(x, eps);
  double dk_ = (RBFCubicPolicy::getKernelUnary(x + 1e-6, eps) - k) / 1e-6;
  EXPECT_NEAR(x * dk_x, dk_, 1e-4);
}

TEST(RBFCubicPolicy, getKernelDerivatives) {
  const int D = 3;
  const int N = 125;
  const int Np = 120;
  const double eps = 1.;
  Eigen::MatrixXd nodes = Eigen::MatrixXd::Random(D, N);
  Eigen::MatrixXd positions = Eigen::MatrixXd::Random(D, Np);
  Eigen::MatrixXd K = RBFCubicPolicy::getKernels(positions, nodes, eps);
  Eigen::MatrixXd dK = RBFCubicPolicy::getKernelDerivatives(positions, nodes, eps);
  ASSERT_EQ(K.rows(), Np);
  ASSERT_EQ(K.cols(), N);

  Eigen::MatrixXd dist = RBFCubicPolicy::getDistanceToNodes(positions, nodes);
  auto f = [eps](double x) { return RBFCubicPolicy::getKernelDerivativeUnary(x, eps); };
  Eigen::MatrixXd dK_ = dist.unaryExpr(f);

  EXPECT_TRUE(EIGEN_MATRIX_NEAR_ABS(dK, dK_, 1e-8));
}

TEST(RBFInterpolatorCubic, interpolate) {
  const int D = 3;
  const int Dv = 3;
  const int N = 125;
  const int Np = 120;
  const double eps = 1.;
  using RBFCubicInterpolator = RBFInterpolator<double, D, Dv, RBFCubicPolicy>;
  Eigen::Matrix<double, D, N> nodes;
  nodes.setRandom();

  Eigen::Matrix<double, Dv, N> values;
  values.setRandom();

  Eigen::Matrix<double, D, Np> positions = nodes.leftCols<Np>();

  RBFCubicInterpolator interp(nodes, values, eps);
  Eigen::Matrix<double, Dv, Np> interpolated = interp(positions);

  EXPECT_TRUE(EIGEN_MATRIX_NEAR_ABS(interpolated, values.leftCols<Np>(), 1e-4));
}

TEST(RBFInterpolatorCubic, getGradients) {
  const int D = 3;
  const int Dv = 3;
  const int N = 125;
  const int Np = 100;
  const double eps = 1.;
  using RBFCubicInterpolator = RBFInterpolator<double, D, Dv, RBFCubicPolicy>;
  Eigen::MatrixXd nodes(D, N);
  nodes.setRandom();

  Eigen::MatrixXd values(Dv, N);
  values.setRandom();

  // Eigen::Matrix<double, D, Np> positions = nodes.leftCols<Np>();
  Eigen::MatrixXd positions(D, Np);
  positions.setRandom();

  RBFCubicInterpolator interp(nodes, values, eps);
  Eigen::MatrixXd y = interp(positions);
  Eigen::Tensor<double, 3> dK = interp.getGradients(positions);

  Eigen::Tensor<double, 3> dK_(Dv, D, Np);
  Eigen::Matrix<double, D, D> ident;
  ident.setIdentity();
  const double deps = 1e-6;
  for (int i = 0; i < Np; i++) {
    for (int d = 0; d < D; d++) {
      Eigen::Matrix<double, D, 1> dp = positions.col(i) + ident.col(d) * deps;
      Eigen::Matrix<double, Dv, 1> K_ = interp(dp);
      Eigen::array<int, 3> offsets({d, 0, i});
      Eigen::array<int, 3> extents({1, Dv, 1});
      Eigen::Matrix<double, Dv, 1> diff = (K_.col(0) - y.col(i)) / deps;
      {
        Eigen::TensorMap<Eigen::Tensor<double, 3> > t_diff(diff.data(), Dv, 1, 1);
        Eigen::array<int, 3> offsets = {0, i, d};
        Eigen::array<int, 3> extents = {Dv, 1, 1};
        dK_.slice(offsets, extents) = t_diff;
      }
    }
  }

  EXPECT_TRUE(EIGEN_TENSOR3_NEAR_REL(dK, dK_, 1e-4));
}

TEST(RBFThinPlatePolicy, getKernel_dynamicSize) {
  const int d = 3;
  const int N = 125;
  const double eps = 1.;
  Eigen::MatrixXd nodes = Eigen::MatrixXd::Random(d, N);
  Eigen::VectorXd position = nodes.col(0);
  Eigen::VectorXd K = RBFThinPlatePolicy::getKernel(position, nodes, eps);
  ASSERT_EQ(K.size(), nodes.cols());
  EXPECT_EQ(K(0), 0);
}

TEST(RBFThinPlatePolicy, getKernel_fixedSize) {
  Eigen::Matrix<double, 3, 125> nodes;
  nodes.setRandom();
  const double eps = 1.;
  Eigen::Vector3d position = nodes.col(0);
  Eigen::Matrix<double, 125, 1> K = RBFThinPlatePolicy::getKernel(position, nodes, eps);
  ASSERT_EQ(K.size(), nodes.cols());
  EXPECT_FALSE(K.array().isNaN().any());
  EXPECT_EQ(K(0), 0);
}

TEST(RBFThinPlatePolicy, unaryDerivative) {
  EXPECT_EQ(RBFThinPlatePolicy::getKernelDerivativeUnary(0.0, 1.0), 0.0);
  const double x = 0.5;
  const double eps = 2.0;
  double dk_x = RBFThinPlatePolicy::getKernelDerivativeUnary(x, eps);

  double k = RBFThinPlatePolicy::getKernelUnary(x, eps);
  double dk_ = (RBFThinPlatePolicy::getKernelUnary(x + 1e-6, eps) - k) / 1e-6;
  EXPECT_NEAR(dk_x * x, dk_, 1e-4);
}

TEST(RBFThinPlatePolicy, getKernelDerivatives) {
  const int D = 3;
  const int N = 125;
  const int Np = 120;
  const double eps = 1.;
  Eigen::MatrixXd nodes = Eigen::MatrixXd::Random(D, N);
  Eigen::MatrixXd positions = Eigen::MatrixXd::Random(D, Np);
  Eigen::MatrixXd K = RBFThinPlatePolicy::getKernels(positions, nodes, eps);
  Eigen::MatrixXd dK = RBFThinPlatePolicy::getKernelDerivatives(positions, nodes, eps);
  ASSERT_EQ(K.rows(), Np);
  ASSERT_EQ(K.cols(), N);

  Eigen::MatrixXd dist = RBFThinPlatePolicy::getDistanceToNodes(positions, nodes);
  auto f = [eps](double x) { return RBFThinPlatePolicy::getKernelDerivativeUnary(x, eps); };
  Eigen::MatrixXd dK_ = dist.unaryExpr(f);

  EXPECT_TRUE(EIGEN_MATRIX_NEAR_ABS(dK, dK_, 1e-8));
}

TEST(RBFInterpolatorThinPlate, interpolate) {
  const int D = 3;
  const int Dv = 3;
  const int N = 125;
  const int Np = 120;
  const double eps = 1.;
  using RBFThinplateInterpolator = RBFInterpolator<double, D, Dv, RBFThinPlatePolicy>;
  Eigen::Matrix<double, D, N> nodes;
  nodes.setRandom();

  Eigen::Matrix<double, Dv, N> values;
  values.setRandom();

  Eigen::Matrix<double, D, Np> positions = nodes.leftCols<Np>();

  RBFThinplateInterpolator interp(nodes, values, eps);
  Eigen::Matrix<double, Dv, Np> interpolated = interp(positions);

  EXPECT_TRUE(EIGEN_MATRIX_NEAR_ABS(interpolated, values.leftCols<Np>(), 1e-4));
}

TEST(InterpolatorThinPlateSpline, interpolate) {
  const int D = 3;
  const int Dv = 3;
  const int N = 125;
  const int Np = 120;
  Eigen::Matrix<double, D, N> nodes;
  nodes.setRandom();

  Eigen::Matrix<double, Dv, N> values;
  values.setRandom();

  Eigen::Matrix<double, D, Np> positions = nodes.leftCols<Np>();

  ThinPlateSplineInterpolator interp(nodes, values);

  Eigen::Matrix<double, Dv, Np> interpolated = interp(positions);

  EXPECT_TRUE(EIGEN_MATRIX_NEAR_ABS(interpolated, values.leftCols<Np>(), 1e-4));
}

TEST(InterpolatorThinPlateSpline, interpolateDynamic) {
  const int D = 3;
  const int Dv = 3;
  const int N = 125;
  const int Np = 120;
  Eigen::MatrixXd nodes(D, N);
  nodes.setRandom();

  Eigen::MatrixXd values(Dv, N);
  values.setRandom();

  Eigen::MatrixXd positions = nodes.leftCols<Np>();

  ThinPlateSplineInterpolator interp(nodes, values);
  Eigen::MatrixXd interpolated = interp(positions);

  EXPECT_TRUE(EIGEN_MATRIX_NEAR_ABS(interpolated, values.leftCols<Np>(), 1e-4));
}

TEST(RBFInterpolatorThinPlateSpline, getGradients) {
  const int D = 3;
  const int Dv = 3;
  const int N = 125;
  const int Np = 100;
  const double eps = 1.;
  Eigen::MatrixXd nodes(D, N);
  nodes.setRandom();

  Eigen::MatrixXd values(Dv, N);
  values.setRandom();

  // Eigen::Matrix<double, D, Np> positions = nodes.leftCols<Np>();
  Eigen::MatrixXd positions(D, Np);
  positions.setRandom();

  ThinPlateSplineInterpolator interp(nodes, values);
  Eigen::MatrixXd y = interp(positions);
  Eigen::Tensor<double, 3> dK = interp.getGradients(positions);

  Eigen::Tensor<double, 3> dK_(Dv, D, Np);
  Eigen::Matrix<double, D, D> ident;
  ident.setIdentity();
  const double deps = 1e-6;
  for (int i = 0; i < Np; i++) {
    for (int d = 0; d < D; d++) {
      Eigen::Matrix<double, D, 1> dp = positions.col(i) + ident.col(d) * deps;
      Eigen::Matrix<double, Dv, 1> K_ = interp(dp);
      Eigen::array<int, 3> offsets({d, 0, i});
      Eigen::array<int, 3> extents({1, Dv, 1});
      Eigen::Matrix<double, Dv, 1> diff = (K_.col(0) - y.col(i)) / deps;
      {
        Eigen::TensorMap<Eigen::Tensor<double, 3> > t_diff(diff.data(), Dv, 1, 1);
        Eigen::array<int, 3> offsets = {0, i, d};
        Eigen::array<int, 3> extents = {Dv, 1, 1};
        dK_.slice(offsets, extents) = t_diff;
      }
    }
  }

  EXPECT_TRUE(EIGEN_TENSOR3_NEAR_REL(dK, dK_, 1e-4));
}

TEST(RBFInterpolatorThinPlate, getGradients) {
  const int D = 3;
  const int Dv = 3;
  const int N = 125;
  const int Np = 100;
  const double eps = 1.;
  using RBFThinplateInterpolator = RBFInterpolator<double, D, Dv, RBFThinPlatePolicy>;
  Eigen::MatrixXd nodes(D, N);
  nodes.setRandom();

  Eigen::MatrixXd values(Dv, N);
  values.setRandom();

  // Eigen::Matrix<double, D, Np> positions = nodes.leftCols<Np>();
  Eigen::MatrixXd positions(D, Np);
  positions.setRandom();

  RBFThinplateInterpolator interp(nodes, values, eps);
  Eigen::MatrixXd y = interp(positions);
  Eigen::Tensor<double, 3> dK = interp.getGradients(positions);

  Eigen::Tensor<double, 3> dK_(Dv, D, Np);
  Eigen::Matrix<double, D, D> ident;
  ident.setIdentity();
  const double deps = 1e-6;
  for (int i = 0; i < Np; i++) {
    for (int d = 0; d < D; d++) {
      Eigen::Matrix<double, D, 1> dp = positions.col(i) + ident.col(d) * deps;
      Eigen::Matrix<double, Dv, 1> K_ = interp(dp);
      Eigen::array<int, 3> offsets({d, 0, i});
      Eigen::array<int, 3> extents({1, Dv, 1});
      Eigen::Matrix<double, Dv, 1> diff = (K_.col(0) - y.col(i)) / deps;
      {
        Eigen::TensorMap<Eigen::Tensor<double, 3> > t_diff(diff.data(), Dv, 1, 1);
        Eigen::array<int, 3> offsets = {0, i, d};
        Eigen::array<int, 3> extents = {Dv, 1, 1};
        dK_.slice(offsets, extents) = t_diff;
      }
    }
  }

  EXPECT_TRUE(EIGEN_TENSOR3_NEAR_REL(dK, dK_, 1e-4));
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
