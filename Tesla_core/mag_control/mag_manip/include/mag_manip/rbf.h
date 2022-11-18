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

#include <functional>
#include <memory>

#include <Eigen/Cholesky>
#include <Eigen/Dense>
#include <Eigen/QR>
#include <unsupported/Eigen/CXX11/Tensor>

namespace Eigen {
template <typename Scalar>
struct XLogYOp {
  EIGEN_EMPTY_STRUCT_CTOR(XLogYOp)
  /**
   * @brief A custom binary operator to calculate the following
   * \f[
   * a log(b)
   * \f]
   *
   * This is needed for the case where \$a=0\$ and \$b=0\$ where the output is undefined
   * by calculating the log separately
   *
   * To use this custom operation on two arrays of doubles a1 and a2, call
   * a2.binaryExp(a2, Eigen::XLogYOp<double>())
   *
   * @param a: the lhs of the product
   * @param b: the rhs of the product becomes log(b)
   *
   * @return the output of the binary operation
   */
  Scalar operator()(const Scalar& a, const Scalar& b) const {
    if (b == 0.) {
      return 0.;
    } else {
      return a * std::log(b);
    }
  }
};
}  // namespace Eigen

namespace mag_manip {

/**
 * @brief This is used to dispatch the Eigen linear algebra solver based on whether
 * the kernel creates a positive definite interpolation matrix psi
 *
 * @tparam Policy: the kernel policy
 */
template <typename Policy>
struct IsPositiveDefinite;

/**
 * @brief A templated RBF interpolation class that works with different
 * kernel functions on Eigen fixed-size and dynamic matrix data
 *
 * @tparam Scalar: the Eigen scalar type
 * @tparam D: the dimension of the position data (usually 3)
 * @tparam Dv: the dimension of the values. For 3D vector fields this is 3. For scalar fields this
 * is 1.
 * @tparam RBFPolicy: the policy that implements the kernel (ex RBFGaussianPolicy)
 */
template <typename Scalar, int D, int Dv, typename RBFPolicy>
class RBFInterpolator : private RBFPolicy {
 public:
  using Nodes = Eigen::Matrix<Scalar, D, Eigen::Dynamic>;
  using Values = Eigen::Matrix<Scalar, Dv, Eigen::Dynamic>;
  using ValuesT = Eigen::Matrix<Scalar, Eigen::Dynamic, Dv>;
  using Coeffs = Eigen::Matrix<Scalar, Eigen::Dynamic, Dv>;
  using Tensor3 = Eigen::Tensor<Scalar, 3>;
  using Tensor2 = Eigen::Tensor<Scalar, 2>;
  using Tensor2c = Eigen::Tensor<const Scalar, 2>;

  /**
   * @brief Constructs an interpolator. This also computes the coefficients for the interpolation so
   * you should reuse
   * this instance if you want to perform multiple interpolations on the same data
   *
   * @param nodes: the positions of the N radial bases. Must be of size D x N.
   * @param values: the values of the N data points. Must be of size Dv x N
   * @param shape_param: the value of the scalar shape parameter of the kernel
   */
  RBFInterpolator(const Nodes& nodes, const Values& values, Scalar shape_param)
      : nodes_(nodes),
        values_t_(values.transpose()),
        shape_param_(shape_param),
        coeffs_(getCoeffs()),
        N_(nodes.cols()),
        D_(nodes.rows()),
        Dv_(values.rows()) {
    if (nodes.cols() != values.cols()) {
      std::stringstream ss;
      ss << "node has " << nodes.cols() << " cols while values has " << values.cols();
      throw std::runtime_error(ss.str());
    }
  }

  /**
   * @brief Returns the shape parameter of the basis functions of the interpolant
   *
   * @returns the scalar shape parameter
   */
  Scalar getShapeParam() const { return shape_param_; }

  /**
   * @brief Sets the shape parameter of the basis functions of the interpolant
   *
   * @param shape_param: the scalar shape parameter
   */
  void setShapeParam(Scalar shape_param) { shape_param_ = shape_param; }

  /**
   * @brief Returns the node positions
   *
   * @return a N x Np matrix containing the centers of the nodes
   */
  Nodes getNodes() const { return nodes_; }

  /**
   * @brief Sets the node positions
   *
   * Throws a std::runtime_error if the nodes are invalid
   *
   * @param nodes: a N x Np matrix containing the centers of the nodes
   */
  void setNodes(const Nodes& nodes) {
    if (nodes.cols() != values_t_.rows()) {
      std::stringstream ss;
      ss << "node has " << nodes.cols() << " cols while values has " << values_t_.rows();
      throw std::runtime_error(ss.str());
    }

    D_(nodes.rows());
  }

  /**
   * @brief Returns the function values at the node positions to interpolate between
   *
   * @return a Nv x N matrix containing the function values
   */
  Values getValues() const { return values_t_.transpose(); }

  /**
   * @brief Sets the function values at the node positions to interpolate between
   *
   * Throws a std::runtime_error if the values are invalid
   *
   * @param values: a Nv x N matrix containing the function values
   */
  void setValues(const Values& values) {
    if (nodes_.cols() != values.cols()) {
      std::stringstream ss;
      ss << "node has " << nodes_.cols() << " cols while values has " << values.cols();
      throw std::runtime_error(ss.str());
    }

    Nv_(values.rows());
  }

  /**
   * @brief Computes the RBF interpolation at given positions
   *
   * \f[
   * \matbf{y} = f(\mathbf{p}) = \sum_{i=1}^N \Psi \mathbf{c}_i
   * \f]
   *
   * @tparam PositionsType An Eigen Matrix expression of size DxNp containing the positions
   * at which to compute the interpolation
   *
   * @param positions: An Eigen Matrix expression of size DxNp containing the positions * at which
   * to compute the interpolation
   *
   * @return A Dv x Np sized matrix with the interpolated values
   */
  template <typename PositionsType>
  Eigen::Matrix<Scalar, Dv, PositionsType::ColsAtCompileTime> operator()(
      const Eigen::MatrixBase<PositionsType>& positions) const {
    const int Np = PositionsType::ColsAtCompileTime;
    Eigen::Matrix<Scalar, Np, Eigen::Dynamic> psi = getKernels(positions);
    return (psi * coeffs_).transpose();
  }

  /**
   * @brief Computes the gradient of the RBF at several positions
   *
   * For each position the gradient is
   * \f[
   * \nabla \mathbf{y} = \frac{\partial \mathbf{y}}{\partial \mathbf{x}}
   * \f]
   *
   * And the gradient matrix has size Nv x N where each column represents a varied dimension of the
   * position
   *
   * It is more efficient to compute the interpolation at several positions at once, so we return a
   * tensor which
   * contains all the gradient matrices at the Np positions.
   *
   * @tparam PositionsType: the type of positions
   *
   * @param positions: An Eigen Matrix expression of size DxNp containing the positions
   * at which the gradient is computed
   *
   * @return a tensor of size Dv x D x Np with the gradient values at the Np positions
   */
  template <typename PositionsType>
  Tensor3 getGradients(const Eigen::MatrixBase<PositionsType>& positions) const {
    Tensor3 lhs = RBFPolicy::getGradientOperand(positions, nodes_, shape_param_);
    Eigen::TensorMap<Tensor2c> t_coeffs(coeffs_.data(), N_, Dv_);
    Eigen::array<Eigen::IndexPair<int>, 1> product_dims = {Eigen::IndexPair<int>(2, 0)};
    Tensor3 t_derivative = lhs.contract(t_coeffs, product_dims);

    // this part just rearanges the dimensions of the gradient matrix
    // so that they are Dv x D x Np
    return t_derivative.shuffle(Eigen::array<int, 3>({2, 0, 1}));
  }

  /**
   * @brief Computes the coefficients for a positive definite matrix using tag dispatch
   *
   * We use the LLT decomposition for positive definite matrices
   *
   * @param std::integral_constant: true if the kernel is positive definite
   *
   * @return the coefficients
   */
  Coeffs getCoeffs(std::integral_constant<bool, true>) const {
    Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> psi = getKernels(nodes_);
    return psi.llt().solve(values_t_);
  }

  /**
   * @brief Computes the coefficients for a non positive definite matrix using tag dispatch
   *
   * We use the partial pivot LU decomposition for non positive definite matrices
   *
   * @param std::integral_constant: true if the kernel is positive definite
   *
   * @return the coefficients
   */
  Coeffs getCoeffs(std::integral_constant<bool, false>) const {
    Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> psi = getKernels(nodes_);
    return psi.lu().solve(values_t_);
  }

  /**
   * @brief Computes the coefficients associated with the RBF kernel for interpolation
   *
   * The interpolated data can be scalar over positions of dimension D or vectorial
   * with dimension Dv over positions of dimension D
   *
   * @return a vector of size NxDv with the coefficients of each basis function
   */
  Coeffs getCoeffs() const { return getCoeffs(IsPositiveDefinite<RBFPolicy>()); }

  /**
   * @brief Get the value of the kernel at different positions in D-dimensional data
   *
   * @param positions: a matrix of size D x Np at which to evaluate Np positions
   *
   * @return a matrix of size N x Np containing the values of the different basis functions at the
   * given Np positions
   */
  template <typename PositionsType>
  Eigen::Matrix<Scalar, PositionsType::ColsAtCompileTime, Eigen::Dynamic> getKernels(
      const Eigen::MatrixBase<PositionsType>& positions) const {
    return RBFPolicy::getKernels(positions, nodes_, shape_param_);
  }

  /**
   * @brief Get the value of the kernel derivative at different positions in D-dimensional data
   *
   * @param positions: a matrix of size D x Np at which to evaluate Np positions
   *
   * @return a matrix of size N x Np containing the values of the different basis functions at the
   * given Np positions
   */
  template <typename PositionsType>
  Eigen::Matrix<Scalar, PositionsType::ColsAtCompileTime, Eigen::Dynamic> getKernelDerivatives(
      const Eigen::MatrixBase<PositionsType>& positions) const {
    return RBFPolicy::getKernelDerivatives(positions, nodes_, shape_param_);
  }

 private:
  const int N_;
  const int Dv_;
  const int D_;
  Nodes nodes_;
  ValuesT values_t_;
  Scalar shape_param_;
  Coeffs coeffs_;
};

/**
 * @brief The polyharmonic spline interpolator generalizes the RBF spline with a polyharmonic kernel
 * to contain a
 * polynomial term.
 *
 * \f[
 * f(\mathbf{x}) = \sum_{i=1}^N (\psi(\| \mathbf{x} - \mathbf{r}_i \|) c_i) + \mathbf{d}^T
 * \begin{bmatrix}1 \\
 * \mathbf{x} \end{bmatrix}
 * \f]
 *
 * This polynomical term yields better extrapolation properties
 *
 * @tparam Scalar: the scalar type to be used
 * @tparam RBFPolicy: the RBF policy. Should usually be RBFThinPlateProperty.
 */
template <typename Scalar, typename RBFPolicy>
class PolyharmonicSplineInterpolator : private RBFPolicy {
  using MatrixXs = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>;
  using Tensor3 = Eigen::Tensor<Scalar, 3>;
  using Tensor2 = Eigen::Tensor<Scalar, 2>;
  using Tensor2c = Eigen::Tensor<const Scalar, 2>;

 public:
  using Ptr = std::shared_ptr<PolyharmonicSplineInterpolator<Scalar, RBFPolicy> >;
  /**
   * @brief Constructs the interpolator and calculates the coefficients
   *
   * N is the number of node positions
   * D is the dimension of the positiosn (D=3 for 3D)
   * Dv is the dimension of the values (Dv=1 for scalar data and Dv=3 for
   * 3D fields)
   *
   * @param nodes: a matrix of size DxN containing the node positions
   * @param values: a matrix of size Dv x N containing the data associated with
   * each node position
   */
  PolyharmonicSplineInterpolator(const MatrixXs& nodes, const MatrixXs& values)
      : N_(nodes.cols()),
        D_(nodes.rows()),
        Dv_(values.rows()),
        nodes_(nodes),
        values_t_(values.transpose()),
        L_(N_ + D_ + 1, N_ + D_ + 1),
        W_(N_ + D_ + 1, Dv_) {
    if (nodes.cols() != values.cols()) {
      std::stringstream ss;
      ss << "node has " << nodes.cols() << " cols while values has " << values.cols();
      throw std::runtime_error(ss.str());
    }
    getCoeffs();
  }

  /**
   * @brief Performs the interpolation
   *
   * @tparam PositionsType: The p
   * @param positions: a matrix of size DxNp with containing the positions at
   * which to perform the interpolation
   *
   * @return a matrix of size DvxNp with the interpolated data
   */
  template <typename PositionsType>
  // output is Dv x Np
  MatrixXs operator()(const Eigen::MatrixBase<PositionsType>& positions) const {
    const int Np = positions.cols();
    MatrixXs psi = getKernels(positions);
    MatrixXs w(D_ + 1, Np);
    w << MatrixXs::Ones(1, Np), positions;
    return (psi * coeffs_rbf_).transpose() + (coeffs_poly_.transpose() * w);
  }
  /**
   * @brief Computes the gradient of the Polyharmonic spline at several positions
   *
   * For each position the gradient is
   * \f[
   * \nabla \mathbf{y} = \frac{\partial \mathbf{y}}{\partial \mathbf{x}}
   * \f]
   *
   * And the gradient matrix has size Nv x N where each column represents a varied dimension of the
   * position
   *
   * It is more efficient to compute the interpolation at several positions at once, so we return a
   * tensor which
   * contains all the gradient matrices at the Np positions.
   *
   * @tparam PositionsType: the type of positions
   *
   * @param positions: An Eigen Matrix expression of size DxNp containing the positions
   * at which the gradient is computed
   *
   * @return a tensor of size Dv x D x Np with the gradient values at the Np positions
   */
  template <typename PositionsType>
  Tensor3 getGradients(const Eigen::MatrixBase<PositionsType>& positions) const {
    const int Np = positions.cols();

    // we ignore the shape parameter here so it's set to 1
    Tensor3 lhs = RBFPolicy::getGradientOperand(positions, nodes_, 1.);
    Eigen::TensorMap<Tensor2c> t_coeffs_rbf(coeffs_rbf_.data(), N_, Dv_);
    Eigen::array<Eigen::IndexPair<int>, 1> product_dims = {Eigen::IndexPair<int>(2, 0)};
    Tensor3 t_derivative = lhs.contract(t_coeffs_rbf, product_dims);

    // this part just rearanges the dimensions of the gradient matrix
    // so that they are Dv x D x Np
    return t_derivative.shuffle(Eigen::array<int, 3>({2, 0, 1}));
  }

 private:
  /**
   * @brief Convenience function that generates the matrices needed to calculate
   * the coefficients
   */
  void setupLinearProblem() {
    MatrixXs Psi = getKernels(nodes_);
    MatrixXs B(D_ + 1, N_);
    B << Eigen::Matrix<Scalar, 1, Eigen::Dynamic>::Ones(N_), nodes_;
    L_ = MatrixXs::Zero(N_ + D_ + 1, N_ + D_ + 1);
    L_.block(0, 0, N_, N_) = Psi;
    L_.block(0, N_, N_, D_ + 1) = B.transpose();
    L_.block(N_, 0, D_ + 1, N_) = B;
    W_.resize(N_ + D_ + 1, Dv_);
    W_ << values_t_, MatrixXs::Zero(D_ + 1, Dv_);
  }

  /**
   * @brief This one is run for RBFPolicies that are marked as
   * PositiveDefinite using the IsPositiveDefinite trait
   *
   * @param std::integral_constant
   */
  void getCoeffs(std::integral_constant<bool, true>) {
    setupLinearProblem();
    MatrixXs coeffs_grouped = L_.llt().solve(W_);
    coeffs_rbf_ = coeffs_grouped.topRows(N_);
    coeffs_poly_ = coeffs_grouped.bottomRows(D_ + 1);
  }

  /**
   * @brief This one is run for RBFPolicies that are marked as
   * non PositiveDefinite using the IsPositiveDefinite trait
   *
   * @param std::integral_constant
   */
  void getCoeffs(std::integral_constant<bool, false>) {
    setupLinearProblem();
    MatrixXs coeffs_grouped = L_.lu().solve(W_);
    coeffs_rbf_ = coeffs_grouped.topRows(N_);
    coeffs_poly_ = coeffs_grouped.bottomRows(D_ + 1);
  }

  /**
   * @brief Computes and stores the coefficients
   */
  void getCoeffs() { getCoeffs(IsPositiveDefinite<RBFPolicy>()); }

  template <typename PositionsType>
  MatrixXs getKernels(const Eigen::MatrixBase<PositionsType>& positions) const {
    return RBFPolicy::getKernels(positions, nodes_, 1.0);
  }

  const int N_;
  const int D_;
  const int Dv_;

  MatrixXs nodes_;
  MatrixXs values_t_;
  MatrixXs coeffs_rbf_;
  MatrixXs coeffs_poly_;
  MatrixXs L_;
  MatrixXs W_;
};

/**
 * @brief Implements the common functionality to all the RBF kernel implementations
 *
 * @tparam RBFPolicy the kernel implementation that derives from this base
 */
template <typename RBFPolicy>
class RBFPolicyBase {
 public:
  /**
   * @brief Computes
   *
   * \f[
   * | \mathbf{P} - \mathbf{R} |
   * \f]
   *
   * where \$\mathbf{P}\$ contains a list of positions where each position is a column
   * and \$\mathbf{R}\$ contains a list of node positions where each node is a column
   *
   * @tparam PositionsType the type of the positions matrix
   * @tparam NodesType the type of the nodes matrix
   * @param positions: a matrix of size Np x N
   * @param nodes: a matrix of size D x N
   *
   * @return matrix of size Np x N containing the distances from all positions
   * to all nodes
   */
  template <typename PositionsType, typename NodesType>
  static Eigen::Matrix<typename PositionsType::Scalar, Eigen::Dynamic, Eigen::Dynamic>
  getDistanceToNodes(const Eigen::MatrixBase<PositionsType>& positions,
                     const Eigen::MatrixBase<NodesType>& nodes) {
    const int Np = positions.cols();
    const int N = nodes.cols();
    Eigen::Matrix<typename PositionsType::Scalar, Eigen::Dynamic, Eigen::Dynamic> distances(Np, N);
    for (int i = 0; i < Np; i++) {
      for (int j = 0; j < N; j++) {
        distances(i, j) = (positions.col(i) - nodes.col(j)).norm();
      }
    }
    return distances;
  }

  /**
   *
   * @brief Get the value of the kernel at different positions in D-dimensional data
   *
   * @param positions: a matrix of size D x Np at which to evaluate Np positions
   * @param nodes: the positions of the N radial bases. Must be of size DxN.
   * @param eps: the scale parameter of the RBF
   * @return a matrix of size N x Np containing the values of the different basis functions at the
   * given Np positions
   */
  template <typename PositionsType, typename NodesType>
  static Eigen::Matrix<typename PositionsType::Scalar, PositionsType::ColsAtCompileTime,
                       NodesType::ColsAtCompileTime>
  getKernels(const Eigen::MatrixBase<PositionsType>& positions,
             const Eigen::MatrixBase<NodesType>& nodes,
             typename PositionsType::Scalar shape_param) {
    const int Np = PositionsType::ColsAtCompileTime;
    const int N = NodesType::ColsAtCompileTime;

    // Note that this is determined at run time. A better way would be to
    // determine at compile-time whether the matrices are dynamic using SFINAE
    if (Np == Eigen::Dynamic || N == Eigen::Dynamic) {
      const int Np = positions.cols();
      const int N_ = nodes.cols();
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> K(Np, N_);
      for (int i = 0; i < Np; i++) {
        K.row(i) = RBFPolicy::getKernel(positions.col(i), nodes, shape_param);
      }
      return K;

    } else {
      Eigen::Matrix<typename PositionsType::Scalar, Np, N> K;
      for (int i = 0; i < Np; i++) {
        K.row(i) = RBFPolicy::getKernel(positions.col(i), nodes, shape_param);
      }
      return K;
    }
  }

  /**
   *
   * @brief Get the value of the kernel derivatives at different positions in D-dimensional data
   *
   * @param positions: a matrix of size D x Np at which to evaluate Np positions
   * @param nodes: the positions of the N radial bases. Must be of size DxN.
   * @param eps: the scale parameter of the RBF
   * @return a matrix of size N x Np containing the values of the different basis functions
   * derivatives at the
   * given Np positions
   */
  template <typename PositionsType, typename NodesType>
  static Eigen::Matrix<typename PositionsType::Scalar, PositionsType::ColsAtCompileTime,
                       NodesType::ColsAtCompileTime>
  getKernelDerivatives(const Eigen::MatrixBase<PositionsType>& positions,
                       const Eigen::MatrixBase<NodesType>& nodes,
                       typename PositionsType::Scalar shape_param) {
    const int Np = PositionsType::ColsAtCompileTime;
    const int N = NodesType::ColsAtCompileTime;

    // Note that this is determined at run time. A better way would be to
    // determine at compile-time whether the matrices are dynamic using SFINAE
    if (Np == Eigen::Dynamic || N == Eigen::Dynamic) {
      const int Np = positions.cols();
      const int N_ = nodes.cols();
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> dK(Np, N_);
      for (int i = 0; i < Np; i++) {
        dK.row(i) = RBFPolicy::getKernelDerivative(positions.col(i), nodes, shape_param);
      }
      return dK;

    } else {
      Eigen::Matrix<typename PositionsType::Scalar, Np, N> dK;
      for (int i = 0; i < Np; i++) {
        dK.row(i) = RBFPolicy::getKernelDerivative(positions.col(i), nodes, shape_param);
      }
      return dK;
    }
  }

  /**
   * @brief Computes
   *
   * \f[
   *  \mathbf{P} - \mathbf{R}
   * \f]
   *
   * where \$\mathbf{P}\$ contains a list of positions where each position is a column
   * and \$\mathbf{R}\$ contains a list of node positions where each node is a column
   *
   * @tparam PositionsType the type of the positions matrix
   * @tparam NodesType the type of the nodes matrix
   * @param positions: a matrix of size Np x N
   * @param nodes: a matrix of size D x N
   *
   * @return tensor of size D x Np x N containing the vectors from all nodes to
   * positions
   */
  template <typename PositionsType, typename NodesType>
  static Eigen::Tensor<typename PositionsType::Scalar, 3> getPositionToNodes(
      const Eigen::MatrixBase<PositionsType>& positions,
      const Eigen::MatrixBase<NodesType>& nodes) {
    using Scalar = typename PositionsType::Scalar;
    const int Np = positions.cols();
    const int D = positions.rows();
    const int N = nodes.cols();

    Eigen::Tensor<Scalar, 3> t_dvectors(D, Np, N);
    for (int d = 0; d < D; d++) {
      for (int i = 0; i < Np; i++) {
        for (int j = 0; j < N; j++) {
          t_dvectors(d, i, j) = positions(d, i) - nodes(d, j);
        }
      }
    }

    return t_dvectors;
  }

  template <typename PositionsType, typename NodesType>
  static Eigen::Tensor<typename PositionsType::Scalar, 3> getGradientOperand(
      const Eigen::MatrixBase<PositionsType>& positions, const Eigen::MatrixBase<NodesType>& nodes,
      typename PositionsType::Scalar shape_param) {
    using Scalar = typename PositionsType::Scalar;
    const int Np = positions.cols();
    const int D = positions.rows();
    const int N = nodes.cols();
    using Tensor3 = Eigen::Tensor<Scalar, 3>;
    using Tensor2 = Eigen::Tensor<Scalar, 2>;

    // This part computes psi_dot(| p - r |)
    // and creates a Np x N matrix
    using MatrixXs = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>;
    using namespace std::placeholders;
    const Scalar eps = shape_param;
    auto f = [eps](Scalar x) { return RBFPolicy::getKernelDerivativeUnary(x, eps); };
    MatrixXs distances = RBFPolicy::getDistanceToNodes(positions, nodes);
    MatrixXs psi_dot = distances.unaryExpr(f);

    Eigen::TensorMap<Tensor2> t_psi_dot(psi_dot.data(), Np, N);

    // The psi_dot matrix is then repeated to fit in a
    // D x Np x N tensor so that it can be combined coefficient-wise
    // later and form the gradient tensor
    Tensor3 t_psi_dot_e(D, Np, N);
    for (int d = 0; d < D; d++) {
      Eigen::array<int, 3> offsets = {d, 0, 0};
      Eigen::array<int, 3> extents = {1, Np, N};
      t_psi_dot_e.slice(offsets, extents) = t_psi_dot;
    }

    // This gets the D x Np x N tensor with p - r
    Tensor3 t_dvectors = RBFPolicy::getPositionToNodes(positions, nodes);

    // we also use the | p - r | as the denominator in the gradient computation
    // Eigen::TensorMap<Tensor2> t_distances(distances.data(), Np, N);
    // Tensor3 t_dvectors_norm_e(D, Np, N);
    // for (int d = 0; d < D; d++) {
    //   Eigen::array<int, 3> offsets = {d, 0, 0};
    //   Eigen::array<int, 3> extents = {1, Np, N};
    //   t_dvectors_norm_e.slice(offsets, extents) = t_distances;
    // }

    // on the lhs we have a D x Np x N tensor that forms the generalized
    // gradient tensor for that RBF
    // It is then contracted with the Dv x N coefficients tensor to get the
    // final gradient tensor of size D x Np x Dv
    return t_psi_dot_e * t_dvectors;
  }
};

class RBFGaussianPolicy : public RBFPolicyBase<RBFGaussianPolicy> {
 public:
  /**
   * @brief Gets the value of the Gaussian kernel at a given position in D-dimensional data
   *
   * The Gaussian kernel computes
   * \f[
   * \psi(\mathbf{p}) = \exp(-\epsilon\| \mathbf{p} - \mathbf{r}_i \|^2)
   * ]\f
   *
   * for radial basis centered at \f$\mathbf{r}_i\f$ for \f$i \in [0 \cdots N]\f$ with
   * \f$\mathbf{r}_i} \in \mathbb{R}^d \f$
   *
   * @param position: a vector of size Dx1 at which to evaluate the values of the basis functions
   * @param nodes: the positions of the N radial bases. Must be of size DxN.
   * @param eps: the scale parameter of the RBF
   *
   * Warning this computes the value of the basis functions at a single position. To compute at
   * several positions, use getKernels
   *
   * @return a vector of size Nx1 containing the value of the different basis functions at the
   * given
   * position
   */
  template <typename PositionsType, typename NodesType>
  static Eigen::Matrix<typename PositionsType::Scalar, NodesType::ColsAtCompileTime, 1> getKernel(
      const Eigen::MatrixBase<PositionsType>& position, const Eigen::MatrixBase<NodesType>& nodes,
      typename PositionsType::Scalar eps) {
    const int N = NodesType::ColsAtCompileTime;
    return ((nodes.colwise() - position).colwise().squaredNorm().array() * (-eps)).exp();
  }

  /**
   * @brief Computes the value of the kernel derivative using matrix expressions
   *
   * Note that the derivative is divided once by x, to compensate for the denominator in
   * the interpolation gradient calculation and avoid divide by 0
   *
   * \f[
   * \frac{\dot{\psi}(x)}{x} = -2 \epsilon \exp(-\epsilon x^2)
   *  \f]
   *
   * @tparam PositionsType
   * @tparam NodesType
   * @param position: a vector of length D of position values
   * @param nodes: a matrix of size D x N with the node centers
   * @param eps: the shape parameter
   *
   * @return the value of the kernel derivative for each node center
   */
  template <typename PositionsType, typename NodesType>
  static Eigen::Matrix<typename PositionsType::Scalar, NodesType::ColsAtCompileTime, 1>
  getKernelDerivative(const Eigen::MatrixBase<PositionsType>& position,
                      const Eigen::MatrixBase<NodesType>& nodes,
                      typename PositionsType::Scalar eps) {
    const int N = NodesType::ColsAtCompileTime;
    using VectorN = Eigen::Matrix<typename PositionsType::Scalar, N, 1>;
    auto distances_sq = (nodes.colwise() - position).colwise().squaredNorm();
    auto distances = (nodes.colwise() - position).colwise().norm();
    return (-eps * distances_sq).array().exp().array() * (-2 * eps);
  }

  /**
   * @brief Computes the value of the kernel derivative
   *
   * \f[
   * \dot{\psi}(x) = -2 \epsilon x \exp(-\epsilon x^2)
   *  \f]
   *
   * @tparam Scalar: the scalar type
   * @param x: the value of the radial parameter
   * @param eps: the shape parameter of the Gaussian kernel
   *
   * @return value of the kernel
   */
  template <typename Scalar>
  static Scalar getKernelUnary(Scalar x, Scalar eps) {
    return std::exp(-eps * x * x);
  }

  /**
   * @brief Computes the value of the kernel derivative
   *
   * \f[
   * \frac{\dot{\psi}(x)}{x} = -2 \epsilon \exp(-\epsilon x^2)
   *  \f]
   *
   * @tparam Scalar: the scalar type
   * @param x: the value of the radial parameter
   * @param eps: the shape parameter of the Gaussian kernel
   *
   * @return value of the kernel
   */
  template <typename Scalar>
  static Scalar getKernelDerivativeUnary(Scalar x, Scalar eps) {
    return -2 * eps * std::exp(-eps * x * x);
  }
};

template <>
struct IsPositiveDefinite<RBFGaussianPolicy> : std::integral_constant<bool, false> {};

class RBFMultiquadricPolicy : public RBFPolicyBase<RBFMultiquadricPolicy> {
 public:
  /**
   * @brief Gets the value of the multiquadric kernel at a given position in D-dimensional data
   *
   * The multiquadric kernel computes
   * \f[
   * \psi(\mathbf{p}) = \sqrt{ 1  + \epsilon\| \mathbf{p} - \mathbf{r}_i \|^2}
   * ]\f
   *
   * for radial basis centered at \f$\mathbf{r}_i\f$ for \f$i \in [0 \cdots N]\f$ with
   * \f$\mathbf{r}_i} \in \mathbb{R}^d \f$
   *
   * @param position: a vector of size Dx1 at which to evaluate the values of the basis functions
   * @param nodes: the positions of the N radial bases. Must be of size DxN.
   * @param eps: the scale parameter of the RBF
   *
   * Warning this computes the value of the basis functions at a single position. To compute at
   * several positions, use getKernels
   *
   * @return a vector of size Nx1 containing the value of the different basis functions at the
   * given
   * position
   */
  template <typename PositionsType, typename NodesType>
  static Eigen::Matrix<typename PositionsType::Scalar, NodesType::ColsAtCompileTime, 1> getKernel(
      const Eigen::MatrixBase<PositionsType>& position, const Eigen::MatrixBase<NodesType>& nodes,
      typename PositionsType::Scalar eps) {
    const int N = NodesType::ColsAtCompileTime;
    return ((nodes.colwise() - position).colwise().squaredNorm().array() * eps + 1).sqrt();
  }

  /**
   * @brief Computes the value of the kernel
   *
   * \f[
   * \psi(x) = \sqrt{1 + \epsilon x^2}
   *  \f]
   *
   * @tparam Scalar: the scalar type
   * @param x: the value of the radial parameter
   * @param eps: the shape parameter of the Gaussian kernel
   *
   * @return value of the kernel
   */
  template <typename Scalar>
  static Scalar getKernelUnary(Scalar x, Scalar eps) {
    return std::sqrt(1 + eps * x * x);
  }

  /**
   * @brief Computes the value of the kernel derivative using matrix expressions
   *
   * \f[
   * \frac{\dot(\psi(x)}{x} = \frac{\epsilon}{\sqrt(1 + \epsilon x^2}
   *  \f]
   *
   * @tparam PositionsType
   * @tparam NodesType
   * @param position: a vector of length D of position values
   * @param nodes: a matrix of size D x N with the node centers
   * @param eps: the shape parameter
   *
   * @return the value of the kernel derivative for each node center
   */
  template <typename PositionsType, typename NodesType>
  static Eigen::Matrix<typename PositionsType::Scalar, NodesType::ColsAtCompileTime, 1>
  getKernelDerivative(const Eigen::MatrixBase<PositionsType>& position,
                      const Eigen::MatrixBase<NodesType>& nodes,
                      typename PositionsType::Scalar eps) {
    const int N = NodesType::ColsAtCompileTime;
    auto distances_sq = (nodes.colwise() - position).colwise().squaredNorm();
    return (distances_sq.array() * eps + 1).array().sqrt().inverse() * eps;
  }

  /**
   * @brief Computes the value of the kernel derivative
   *
   * \f[
   * \frac{\dot(\psi(x)}{x} = \frac{\epsilon}{\sqrt(1 + \epsilon x^2}
   *  \f]
   *
   * @tparam Scalar: the scalar type
   * @param x: the value of the radial parameter
   * @param eps: the shape parameter of the Gaussian kernel
   *
   * @return value of the kernel
   */
  template <typename Scalar>
  static Scalar getKernelDerivativeUnary(Scalar x, Scalar eps) {
    return eps / std::sqrt(1 + eps * x * x);
  }
};

template <>
struct IsPositiveDefinite<RBFMultiquadricPolicy> : std::integral_constant<bool, false> {};

class RBFInverseMultiquadricPolicy : public RBFPolicyBase<RBFInverseMultiquadricPolicy> {
 public:
  /**
   * @brief Gets the value of the inverse multiquadric kernel at a given position in D-dimensional
   * data
   *
   * The inverse multiquadric kernel computes
   * \f[
   * \psi(\mathbf{p}) = \frac{1}{\sqrt{ 1  + \epsilon\| \mathbf{p} - \mathbf{r}_i \|^2}}
   * ]\f
   *
   * for radial basis centered at \f$\mathbf{r}_i\f$ for \f$i \in [0 \cdots N]\f$ with
   * \f$\mathbf{r}_i} \in \mathbb{R}^d \f$
   *
   * @param position: a vector of size Dx1 at which to evaluate the values of the basis functions
   * @param nodes: the positions of the N radial bases. Must be of size DxN.
   * @param eps: the scale parameter of the RBF
   *
   * Warning this computes the value of the basis functions at a single position. To compute at
   * several positions, use getKernels
   *
   * @return a vector of size Nx1 containing the value of the different basis functions at the
   * given
   * position
   */
  template <typename PositionsType, typename NodesType>
  static Eigen::Matrix<typename PositionsType::Scalar, NodesType::ColsAtCompileTime, 1> getKernel(
      const Eigen::MatrixBase<PositionsType>& position, const Eigen::MatrixBase<NodesType>& nodes,
      typename PositionsType::Scalar eps) {
    return 1. / ((nodes.colwise() - position).colwise().squaredNorm().array() * eps + 1).sqrt();
  }

  /**
   * @brief Computes the value of the kernel
   *
   * \f[
   * \psi(x) = \frac{1}{\sqrt{1 + \epsilon x^2}}
   *  \f]
   *
   * @tparam Scalar: the scalar type
   * @param x: the value of the radial parameter
   * @param eps: the shape parameter of the Gaussian kernel
   *
   * @return value of the kernel
   */
  template <typename Scalar>
  static Scalar getKernelUnary(Scalar x, Scalar eps) {
    return 1 / std::sqrt(1 + eps * x * x);
  }

  /**
   * @brief Computes the value of the kernel derivative using matrix expressions
   *
   * \f[
   * \frac{\dot(\psi(x)}{x} = -frac{\epsilon}{\sqrt{1 + \epsilon x^2}^3}
   *  \f]
   *
   * @tparam PositionsType
   * @tparam NodesType
   * @param position: a vector of length D of position values
   * @param nodes: a matrix of size D x N with the node centers
   * @param eps: the shape parameter
   *
   * @return the value of the kernel derivative for each node center
   */
  template <typename PositionsType, typename NodesType>
  static Eigen::Matrix<typename PositionsType::Scalar, NodesType::ColsAtCompileTime, 1>
  getKernelDerivative(const Eigen::MatrixBase<PositionsType>& position,
                      const Eigen::MatrixBase<NodesType>& nodes,
                      typename PositionsType::Scalar eps) {
    const int N = NodesType::ColsAtCompileTime;
    using VectorN = Eigen::Matrix<typename PositionsType::Scalar, N, 1>;
    auto distances_sq = (nodes.colwise() - position).colwise().squaredNorm();
    return (distances_sq.array() * eps + 1).array().sqrt().cube().inverse() * (-eps);
  }

  /**
   * @brief Computes the value of the kernel derivative
   *
   * \f[
   * \frac{\dot(\psi(x)}{x} = -frac{\epsilon}{\sqrt{1 + \epsilon x^2}^3}
   *  \f]
   *
   * @tparam Scalar: the scalar type
   * @param x: the value of the radial parameter
   * @param eps: the shape parameter of the Gaussian kernel
   *
   * @return value of the kernel
   */
  template <typename Scalar>
  static Scalar getKernelDerivativeUnary(Scalar x, Scalar eps) {
    return -eps / std::pow(std::sqrt(1 + eps * x * x), 3);
  }
};

template <>
struct IsPositiveDefinite<RBFInverseMultiquadricPolicy> : std::integral_constant<bool, true> {};

class RBFCubicPolicy : public RBFPolicyBase<RBFCubicPolicy> {
 public:
  /**
   * @brief Gets the value of the cubic kernel at a given position in D-dimensional
   * data
   *
   * The cubic kernel computes
   * \f[
   * \psi(\mathbf{p}) =  \epsilon \| \mathbf{p} - \mathbf{r}_i \|^3
   * ]\f
   *
   * for radial basis centered at \f$\mathbf{r}_i\f$ for \f$i \in [0 \cdots N]\f$ with
   * \f$\mathbf{r}_i} \in \mathbb{R}^d \f$
   *
   * @param position: a vector of size Dx1 at which to evaluate the values of the basis functions
   * @param nodes: the positions of the N radial bases. Must be of size DxN.
   * @param eps: the scale parameter of the RBF
   *
   * Warning this computes the value of the basis functions at a single position. To compute at
   * several positions, use getKernels
   *
   * @return a vector of size Nx1 containing the value of the different basis functions at the
   * given
   * position
   */
  template <typename PositionsType, typename NodesType>
  static Eigen::Matrix<typename PositionsType::Scalar, NodesType::ColsAtCompileTime, 1> getKernel(
      const Eigen::MatrixBase<PositionsType>& position, const Eigen::MatrixBase<NodesType>& nodes,
      typename PositionsType::Scalar eps) {
    const int N = NodesType::ColsAtCompileTime;
    return (nodes.colwise() - position).colwise().norm().array().cube();
  }

  /**
   * @brief Computes the value of the kernel
   *
   * \f[
   * \psi(x) = x^3
   *  \f]
   *
   * @tparam Scalar: the scalar type
   * @param x: the value of the radial parameter
   * @param eps: the shape parameter of the Gaussian kernel
   *
   * @return value of the kernel
   */
  template <typename Scalar>
  static Scalar getKernelUnary(Scalar x, Scalar eps) {
    return std::pow(x, 3);
  }

  /**
   * @brief Computes the value of the kernel derivative using matrix expressions
   *
   * \f[
   * \frac{\dot(\psi(x)}{x} = 3 x
   *  \f]
   *
   * @tparam PositionsType
   * @tparam NodesType
   * @param position: a vector of length D of position values
   * @param nodes: a matrix of size D x N with the node centers
   * @param eps: the shape parameter
   *
   * @return the value of the kernel derivative for each node center
   */
  template <typename PositionsType, typename NodesType>
  static Eigen::Matrix<typename PositionsType::Scalar, NodesType::ColsAtCompileTime, 1>
  getKernelDerivative(const Eigen::MatrixBase<PositionsType>& position,
                      const Eigen::MatrixBase<NodesType>& nodes,
                      typename PositionsType::Scalar eps) {
    return (nodes.colwise() - position).colwise().norm().array() * 3;
  }

  /**
   * @brief Computes the value of the kernel derivative
   *
   * \f[
   * \frac{\dot(\psi(x)}{x} = 3 x
   *  \f]
   *
   * @tparam Scalar: the scalar type
   * @param x: the value of the radial parameter
   * @param eps: the shape parameter of the Gaussian kernel
   *
   * @return value of the kernel
   */
  template <typename Scalar>
  static Scalar getKernelDerivativeUnary(Scalar x, Scalar eps) {
    return 3 * x;
  }
};

template <>
struct IsPositiveDefinite<RBFCubicPolicy> : std::integral_constant<bool, false> {};

class RBFThinPlatePolicy : public RBFPolicyBase<RBFThinPlatePolicy> {
 public:
  /**
   * @brief Gets the value of the thin plate kernel at a given position in D-dimensional
   * data
   *
   * The thin plate kernel computes
   * \f[
   * \psi(\mathbf{p}) =  \| \mathbf{p} - \mathbf{r}_i \|^2 \log( \| \mathbf{p} - \mathbf{r}_i \| )
   * ]\f
   *
   * for radial basis centered at \f$\mathbf{r}_i\f$ for \f$i \in [0 \cdots N]\f$ with
   * \f$\mathbf{r}_i} \in \mathbb{R}^d \f$
   *
   * @param position: a vector of size Dx1 at which to evaluate the values of the basis functions
   * @param nodes: the positions of the N radial bases. Must be of size DxN.
   * @param eps: note this parameter is only provided as an interface and is ignored
   *
   * @return
   */
  template <typename PositionsType, typename NodesType>
  static Eigen::Matrix<typename PositionsType::Scalar, NodesType::ColsAtCompileTime, 1> getKernel(
      const Eigen::MatrixBase<PositionsType>& position, const Eigen::MatrixBase<NodesType>& nodes,
      typename PositionsType::Scalar eps) {
    const int N = NodesType::ColsAtCompileTime;
    using Scalar = typename PositionsType::Scalar;
    using VectorN = Eigen::Matrix<typename PositionsType::Scalar, N, 1>;
    VectorN r = (nodes.colwise() - position).colwise().norm();
    VectorN lhs = (nodes.colwise() - position).colwise().squaredNorm();
    return lhs.array().binaryExpr(r.array(), Eigen::XLogYOp<Scalar>());
  }

  /**
   * @brief Computes the value of the kernel
   *
   * \f[
   * \psi(x) = x^2 \log(x)
   *  \f]
   *
   * @tparam Scalar: the scalar type
   * @param x: the value of the radial parameter
   * @param eps: the shape parameter of the Gaussian kernel
   *
   * @return value of the kernel
   */
  template <typename Scalar>
  static Scalar getKernelUnary(Scalar x, Scalar eps) {
    return Eigen::XLogYOp<Scalar>()(x * x, x);
  }

  /**
   * @brief Computes the value of the kernel derivative using matrix expressions
   *
   * \f[
   * \frac{\dot(\psi(x)}{x} = 2 \log(x) + 1
   *  \f]
   *
   * @tparam PositionsType
   * @tparam NodesType
   * @param position: a vector of length D of position values
   * @param nodes: a matrix of size D x N with the node centers
   * @param eps: the shape parameter
   *
   * @return the value of the kernel derivative for each node center
   */
  template <typename PositionsType, typename NodesType>
  static Eigen::Matrix<typename PositionsType::Scalar, NodesType::ColsAtCompileTime, 1>
  getKernelDerivative(const Eigen::MatrixBase<PositionsType>& position,
                      const Eigen::MatrixBase<NodesType>& nodes,
                      typename PositionsType::Scalar eps) {
    const int N = NodesType::ColsAtCompileTime;
    using Scalar = typename PositionsType::Scalar;
    using VectorN = Eigen::Matrix<typename PositionsType::Scalar, N, 1>;
    VectorN r = (nodes.colwise() - position).colwise().norm();
    auto f = [eps](const Scalar x) { return getKernelDerivativeUnary(x, eps); };
    return r.array().unaryExpr(f);
  }

  /**
   * @brief Computes the value of the kernel derivative
   *
   * \f[
   * \frac{\dot(\psi(x)}{x} = 2 \log(x) + 1
   *  \f]
   *
   * @tparam Scalar: the scalar type
   * @param x: the value of the radial parameter
   * @param eps: the shape parameter of the Gaussian kernel
   *
   * @return value of the kernel
   */
  template <typename Scalar>
  static Scalar getKernelDerivativeUnary(Scalar x, Scalar eps) {
    if (x == 0.0) {
      return 0.0;
    } else {
      return 2 * std::log(x) + 1;
    }
  }
};

template <>
struct IsPositiveDefinite<RBFThinPlatePolicy> : std::integral_constant<bool, false> {};

using ThinPlateSplineInterpolator = PolyharmonicSplineInterpolator<double, RBFThinPlatePolicy>;

}  // namespace mag_manip
