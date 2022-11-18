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

#include <benchmark/benchmark.h>
#include <Eigen/Cholesky>
#include <Eigen/LU>
#include <Eigen/QR>
#include <Eigen/SVD>
#include <unsupported/Eigen/CXX11/Tensor>
#include "mag_manip/impl/rbf_sketches.h"
#include "mag_manip/rbf.h"

using namespace mag_manip;

static void bm_coeffOpKernel(benchmark::State& state) {
  const int N = 125;
  const int D = 3;
  const double eps = 1.0;
  const int Np = 100;
  using VectorN = Eigen::Matrix<double, N, 1>;
  Eigen::MatrixXd nodes(D, N);
  nodes.setRandom();

  Eigen::MatrixXd position(D, Np);
  position.setRandom();

  for (auto _ : state) {
    auto temp = RBFGaussianPolicy::getDistanceToNodes(position, nodes);
    Eigen::MatrixXd out = (-eps * temp.array().square()).array().exp();
  }
}

BENCHMARK(bm_coeffOpKernel);

static void bm_unaryOpKernel(benchmark::State& state) {
  const int N = 125;
  const int D = 3;
  const double eps = 1.0;
  const int Np = 100;
  using VectorN = Eigen::Matrix<double, N, 1>;
  Eigen::MatrixXd nodes(D, N);
  nodes.setRandom();
  Eigen::MatrixXd position(D, Np);
  position.setRandom();

  for (auto _ : state) {
    auto f = [eps](double x) { return RBFGaussianPolicy::getKernelUnary(x, eps); };
    auto temp = RBFGaussianPolicy::getDistanceToNodes(position, nodes);
    Eigen::MatrixXd out = temp.unaryExpr(f);
  }
}

BENCHMARK(bm_unaryOpKernel);

static void bm_getGradientsMatrix(benchmark::State& state) {
  const int N = 125;
  const int D = 3;
  const double eps = 1.0;
  const int Np = 100;
  const int Dv = 3;
  // using VectorN = Eigen::Vector
  Eigen::MatrixXd nodes(D, N);
  nodes.setRandom();
  Eigen::MatrixXd positions(D, Np);
  positions.setRandom();

  Eigen::MatrixXd coeffs(N, Dv);
  coeffs.setRandom();

  Eigen::TensorMap<Eigen::Tensor<double, 2> > t_coeffs(coeffs.data(), N, Dv);

  using Tensor3d = Eigen::Tensor<double, 3>;
  using Tensor2d = Eigen::Tensor<double, 2>;

  Tensor3d t_lhs(D, Np, N);

  for (auto _ : state) {
    std::vector<Eigen::MatrixXd> v_grads;
    for (int i = 0; i < Np; i++) {
      auto diffs = (nodes.colwise() - positions.col(i));
      auto temp = diffs.colwise().squaredNorm();
      // N x 1
      auto xpart = temp.array() * (-2 * eps);
      Eigen::VectorXd psi_dot = xpart.array() * (-eps * temp).array().exp();
      auto diffs_n = diffs.array() / diffs.colwise().norm().array();
      // N x 1 * D x N = D x N
      Eigen::array<int, 3> offsets = {0, i, 0};
      Eigen::array<int, 3> extents = {D, 1, N};
      Eigen::MatrixXd lhs = diffs_n.matrix() * psi_dot.asDiagonal();
      t_lhs.slice(offsets, extents) = Eigen::TensorMap<Tensor2d>(lhs.data(), D, N);
      // auto lhs = diffs_n.matrix() * psi_dot.asDiagonal();
      // D x N * (Dv x N)^T
      // Eigen::MatrixXd out = (lhs * coeffs.transpose()).transpose();
      // v_grads.push_back(out);
    }
    Eigen::array<Eigen::IndexPair<int>, 1> product_dims = {Eigen::IndexPair<int>(2, 0)};
    Tensor3d derivative = t_lhs.contract(t_coeffs, product_dims);
    Tensor3d out = derivative.shuffle(Eigen::array<int, 3>({2, 0, 1}));
  }
}

BENCHMARK(bm_getGradientsMatrix);

static void bm_getGradientsTensor(benchmark::State& state) {
  const int N = 125;
  const int D = 3;
  const double eps = 1.0;
  const int Np = 100;
  const int Dv = 3;
  using VectorN = Eigen::Matrix<double, N, 1>;
  Eigen::MatrixXd nodes(D, N);
  nodes.setRandom();
  Eigen::MatrixXd positions(D, Np);
  positions.setRandom();

  using Tensor3d = Eigen::Tensor<double, 3>;
  using Tensor2d = Eigen::Tensor<double, 2>;

  Tensor2d t_coeffs(N, Dv);
  t_coeffs.setRandom();

  for (auto _ : state) {
    Tensor3d lhs = RBFPolicyBase<RBFGaussianPolicy>::getGradientOperand(positions, nodes, eps);
    Eigen::array<Eigen::IndexPair<int>, 1> product_dims = {Eigen::IndexPair<int>(2, 0)};
    Tensor3d derivative = lhs.contract(t_coeffs, product_dims);
    Tensor3d out = derivative.shuffle(Eigen::array<int, 3>({2, 0, 1}));
  }
}

BENCHMARK(bm_getGradientsTensor);

static void bm_getKernelsMatrix(benchmark::State& state) {
  const int N = 125;
  const int D = 3;
  const double eps = 1.0;
  const int Np = 100;
  using VectorN = Eigen::Matrix<double, N, 1>;
  Eigen::MatrixXd nodes(D, N);
  nodes.setRandom();
  Eigen::MatrixXd positions(D, Np);
  positions.setRandom();

  Eigen::MatrixXd kernels(Np, N);
  for (auto _ : state) {
    for (int i = 0; i < Np; i++) {
      VectorN temp = (nodes.colwise() - positions.col(i)).colwise().squaredNorm();
      // kernels.row(i) = temp;
      kernels.row(i) = (-eps * temp).array().exp();
    }
  }
}

BENCHMARK(bm_getKernelsMatrix);

static void bm_getKernelsTensor(benchmark::State& state) {
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

  for (auto _ : state) {
    Tensor3d diffs = RBFPolicyBase<void>::getPositionToNodes(positions, nodes);
    Eigen::array<ptrdiff_t, 1> dims({0});
    Tensor2d out = (diffs.square().sum(dims) * (-eps)).exp();
    // Tensor2d out = (diffs.square().sum(dims));
  }
}

BENCHMARK(bm_getKernelsTensor);

static void bm_getRBFGaussianKernel_dynamicSize(benchmark::State& state) {
  const int d = 3;
  const int N = 125;
  const double eps = 1.;
  Eigen::MatrixXd nodes = Eigen::MatrixXd::Random(d, N);
  Eigen::VectorXd position = nodes.col(0);
  for (auto _ : state) {
    Eigen::VectorXd K = RBFGaussianPolicy::getKernels(position, nodes, eps);
  }
}

BENCHMARK(bm_getRBFGaussianKernel_dynamicSize);

static void bm_getRBFGaussianKernel_fixedSize(benchmark::State& state) {
  Eigen::Matrix<double, 3, 125> nodes;
  nodes.setRandom();
  const double eps = 1.;
  Eigen::Vector3d position = nodes.col(0);
  for (auto _ : state) {
    Eigen::Matrix<double, 125, 1> K = RBFGaussianPolicy::getKernels(position, nodes, eps);
  }
}

BENCHMARK(bm_getRBFGaussianKernel_fixedSize);

static void bm_getRBFGaussianKernels_dynamicSize(benchmark::State& state) {
  const int d = 3;
  const int N = 125;
  const int Np = 120;
  const double eps = 1.;
  Eigen::MatrixXd nodes = Eigen::MatrixXd::Random(d, N);
  Eigen::MatrixXd positions = nodes.leftCols<Np>();
  for (auto _ : state) {
    Eigen::MatrixXd K = RBFGaussianPolicy::getKernels(positions, nodes, eps);
  }
}

BENCHMARK(bm_getRBFGaussianKernels_dynamicSize);

static void bm_getRBFGaussianKernels_fixedSize(benchmark::State& state) {
  const int D = 3;
  const int N = 125;
  const int Np = 120;
  const double eps = 1.;
  Eigen::Matrix<double, D, N> nodes;
  nodes.setRandom();
  Eigen::Matrix<double, D, Np> positions = nodes.leftCols<Np>();
  for (auto _ : state) {
    Eigen::Matrix<double, Np, N> K = RBFGaussianPolicy::getKernels(positions, nodes, eps);
  }
}

BENCHMARK(bm_getRBFGaussianKernels_fixedSize);

static void bm_getRBFGaussianCoeffs_ldlt(benchmark::State& state) {
  const int D = 3;
  const int N = 125;
  const double eps = 1.;
  Eigen::Matrix<double, D, N> nodes;
  nodes.setRandom();

  Eigen::Matrix<double, N, 1> values;
  Eigen::Matrix<double, N, N> psi = RBFGaussianPolicy::getKernels(nodes, nodes, eps);
  for (auto _ : state) psi.ldlt().solve(values);
}

BENCHMARK(bm_getRBFGaussianCoeffs_ldlt);

static void bm_getRBFGaussianCoeffs_llt(benchmark::State& state) {
  const int D = 3;
  const int N = 125;
  const double eps = 1.;
  Eigen::Matrix<double, D, N> nodes;
  nodes.setRandom();

  Eigen::Matrix<double, N, 1> values;
  Eigen::Matrix<double, N, N> psi = RBFGaussianPolicy::getKernels(nodes, nodes, eps);
  for (auto _ : state) psi.llt().solve(values);
}

BENCHMARK(bm_getRBFGaussianCoeffs_llt);

static void bm_getRBFGaussianCoeffs_lu(benchmark::State& state) {
  const int D = 3;
  const int N = 125;
  const double eps = 1.;
  Eigen::Matrix<double, D, N> nodes;
  nodes.setRandom();

  Eigen::Matrix<double, N, 1> values;
  Eigen::Matrix<double, N, N> psi = RBFGaussianPolicy::getKernels(nodes, nodes, eps);
  for (auto _ : state) psi.lu().solve(values);
}

BENCHMARK(bm_getRBFGaussianCoeffs_lu);

static void bm_getRBFGaussianCoeffs_svd(benchmark::State& state) {
  const int D = 3;
  const int N = 125;
  const double eps = 1.;
  using MatrixNN = Eigen::Matrix<double, N, N>;
  Eigen::Matrix<double, D, N> nodes;
  nodes.setRandom();

  Eigen::Matrix<double, N, 1> values;
  MatrixNN psi = RBFGaussianPolicy::getKernels(nodes, nodes, eps);
  for (auto _ : state) {
    Eigen::JacobiSVD<MatrixNN> svd(psi);
    svd.solve(values);
  }
}

BENCHMARK(bm_getRBFGaussianCoeffs_svd);

static void bm_getRBFGaussianCoeffs_qr(benchmark::State& state) {
  const int D = 3;
  const int N = 125;
  const double eps = 1.;
  using MatrixNN = Eigen::Matrix<double, N, N>;
  Eigen::Matrix<double, D, N> nodes;
  nodes.setRandom();

  Eigen::Matrix<double, N, 1> values;
  MatrixNN psi = RBFGaussianPolicy::getKernels(nodes, nodes, eps);
  for (auto _ : state) {
    Eigen::HouseholderQR<MatrixNN> qr(psi);
    qr.solve(values);
  }
}

BENCHMARK(bm_getRBFGaussianCoeffs_qr);

static void bm_PolyharmonicInterpolator_fixed(benchmark::State& state) {
  const int D = 3;
  const int Dv = 3;
  const int N = 125;
  const int Np = 120;
  // using ThinplateSplineInterpolator = PolyharmonicSplineInterpolator<double, RBFThinPlatePolicy>;
  using ThinplateSplineInterpolator =
      PolyharmonicSplineInterpolatorFixed<double, D, N, Dv, RBFThinPlatePolicy>;
  Eigen::Matrix<double, D, N> nodes;
  nodes.setRandom();

  Eigen::Matrix<double, Dv, N> values;
  values.setRandom();

  Eigen::Matrix<double, D, Np> positions = nodes.leftCols<Np>();
  for (auto _ : state) {
    ThinplateSplineInterpolator interp(nodes, values);
    Eigen::Matrix<double, Dv, Np> interpolated = interp(positions);
  }
}

BENCHMARK(bm_PolyharmonicInterpolator_fixed);

static void bm_PolyharmonicInterpolator_dynamic(benchmark::State& state) {
  const int D = 3;
  const int Dv = 3;
  const int N = 125;
  const int Np = 120;
  using ThinplateSplineInterpolator = PolyharmonicSplineInterpolator<double, RBFThinPlatePolicy>;
  Eigen::MatrixXd nodes(D, N);
  nodes.setRandom();

  Eigen::MatrixXd values(Dv, N);
  values.setRandom();

  Eigen::MatrixXd positions = nodes.leftCols<Np>();

  for (auto _ : state) {
    ThinplateSplineInterpolator interp(nodes, values);
    Eigen::MatrixXd interpolated = interp(positions);
  }
}

BENCHMARK(bm_PolyharmonicInterpolator_dynamic);

BENCHMARK_MAIN();
