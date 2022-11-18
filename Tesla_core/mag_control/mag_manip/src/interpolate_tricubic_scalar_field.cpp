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

#include "mag_manip/interpolate_tricubic_scalar_field.h"
#include "mag_manip/impl/tricubic_scalar_field_coeffs.h"

using namespace std;

namespace mag_manip {

InterpolateTricubicScalarField::InterpolateTricubicScalarField(const DataMat& data,
                                                               const VFieldGridProperties& props)
    : InterpolateTricubicScalarFieldDetail(data, props),
      InterpolateRegular(data_scaled_, props),
      are_coeffs_computed_(false),
      coeff_xi_(0),
      coeff_yi_(0),
      coeff_zi_(0),
      coeffs_(Eigen::VectorXd(64)),
      m_(SpMat(TricubicScalarField::ROWS, TricubicScalarField::COLS)) {
  typedef Eigen::Triplet<double> T;
  std::vector<T> triplet_list;
  triplet_list.reserve(TricubicScalarField::NNZ);
  for (int i = 0; i < TricubicScalarField::NNZ; i++) {
    triplet_list.push_back(T(TricubicScalarField::ROW_INDICES[i],
                             TricubicScalarField::COL_INDICES[i], TricubicScalarField::VALUES[i]));
  }
  m_.setFromTriplets(triplet_list.begin(), triplet_list.end());
  qr_.compute(m_);
}

InterpolateTricubicScalarField::Coeffs InterpolateTricubicScalarField::getCoeffs(
    const PositionVec& p) const {
  PositionVec p_n = getNormalizedPosition(p);
  // normalized position
  PositionVec::Scalar x_ix = p_n(0);
  PositionVec::Scalar y_ix = p_n(1);
  PositionVec::Scalar z_ix = p_n(2);

  // floor
  int xi = std::floor(x_ix);
  int yi = std::floor(y_ix);
  int zi = std::floor(z_ix);

  InterpolateTricubicScalarField::Coeffs X(64);
  X.topRows<8>().setZero();

  // values of f(x,y,z)
  X(8) = getAtIndex(xi, yi, zi)(0);
  X(9) = getAtIndex(xi, yi, zi + 1)(0);
  X(10) = getAtIndex(xi, yi + 1, zi)(0);
  X(11) = getAtIndex(xi, yi + 1, zi + 1)(0);
  X(12) = getAtIndex(xi + 1, yi, zi)(0);
  X(13) = getAtIndex(xi + 1, yi, zi + 1)(0);
  X(14) = getAtIndex(xi + 1, yi + 1, zi)(0);
  X(15) = getAtIndex(xi + 1, yi + 1, zi + 1)(0);
  X(16) = getAtIndex(xi, yi, zi)(1);
  X(17) = getAtIndex(xi, yi, zi + 1)(1);
  X(18) = getAtIndex(xi, yi + 1, zi)(1);
  X(19) = getAtIndex(xi, yi + 1, zi + 1)(1);
  X(20) = getAtIndex(xi + 1, yi, zi)(1);
  X(21) = getAtIndex(xi + 1, yi, zi + 1)(1);
  X(22) = getAtIndex(xi + 1, yi + 1, zi)(1);
  X(23) = getAtIndex(xi + 1, yi + 1, zi + 1)(1);
  X(24) = getAtIndex(xi, yi, zi)(2);
  X(25) = getAtIndex(xi, yi, zi + 1)(2);
  X(26) = getAtIndex(xi, yi + 1, zi)(2);
  X(27) = getAtIndex(xi, yi + 1, zi + 1)(2);
  X(28) = getAtIndex(xi + 1, yi, zi)(2);
  X(29) = getAtIndex(xi + 1, yi, zi + 1)(2);
  X(30) = getAtIndex(xi + 1, yi + 1, zi)(2);
  X(31) = getAtIndex(xi + 1, yi + 1, zi + 1)(2);
  // values of dfx/dy
  X(32) = getDataMatD_dy(xi, yi, zi)(0);
  X(33) = getDataMatD_dy(xi, yi, zi + 1)(0);
  X(34) = getDataMatD_dy(xi, yi + 1, zi)(0);
  X(35) = getDataMatD_dy(xi, yi + 1, zi + 1)(0);
  X(36) = getDataMatD_dy(xi + 1, yi, zi)(0);
  X(37) = getDataMatD_dy(xi + 1, yi, zi + 1)(0);
  X(38) = getDataMatD_dy(xi + 1, yi + 1, zi)(0);
  X(39) = getDataMatD_dy(xi + 1, yi + 1, zi + 1)(0);
  // values of dfx/dz
  X(40) = getDataMatD_dz(xi, yi, zi)(0);
  X(41) = getDataMatD_dz(xi, yi, zi + 1)(0);
  X(42) = getDataMatD_dz(xi, yi + 1, zi)(0);
  X(43) = getDataMatD_dz(xi, yi + 1, zi + 1)(0);
  X(44) = getDataMatD_dz(xi + 1, yi, zi)(0);
  X(45) = getDataMatD_dz(xi + 1, yi, zi + 1)(0);
  X(46) = getDataMatD_dz(xi + 1, yi + 1, zi)(0);
  X(47) = getDataMatD_dz(xi + 1, yi + 1, zi + 1)(0);
  // values of dfy/dz
  X(48) = getDataMatD_dz(xi, yi, zi)(1);
  X(49) = getDataMatD_dz(xi, yi, zi + 1)(1);
  X(50) = getDataMatD_dz(xi, yi + 1, zi)(1);
  X(51) = getDataMatD_dz(xi, yi + 1, zi + 1)(1);
  X(52) = getDataMatD_dz(xi + 1, yi, zi)(1);
  X(53) = getDataMatD_dz(xi + 1, yi, zi + 1)(1);
  X(54) = getDataMatD_dz(xi + 1, yi + 1, zi)(1);
  X(55) = getDataMatD_dz(xi + 1, yi + 1, zi + 1)(1);
  // values of dfx/dydz
  X(56) = getDataMatD2_dydz(xi, yi, zi)(0);
  X(57) = getDataMatD2_dydz(xi, yi, zi + 1)(0);
  X(58) = getDataMatD2_dydz(xi, yi + 1, zi)(0);
  X(59) = getDataMatD2_dydz(xi, yi + 1, zi + 1)(0);
  X(60) = getDataMatD2_dydz(xi + 1, yi, zi)(0);
  X(61) = getDataMatD2_dydz(xi + 1, yi, zi + 1)(0);
  X(62) = getDataMatD2_dydz(xi + 1, yi + 1, zi)(0);
  X(63) = getDataMatD2_dydz(xi + 1, yi + 1, zi + 1)(0);

  Eigen::VectorXd coeffs = qr_.solve(X);

  InterpolateTricubicScalarField::Coeffs aijk(64);
  aijk.tail<63>() = coeffs;
  return aijk;
}

FieldVec InterpolateTricubicScalarField::interpolateImpl(const PositionVec& p) const {
  PositionVec p_n = getNormalizedPosition(p);
  // floor
  int xi = std::floor(p_n(0));
  int yi = std::floor(p_n(1));
  int zi = std::floor(p_n(2));
  PositionVec dp = p_n - PositionVec(xi, yi, zi);

  if (!are_coeffs_computed_ || (xi != coeff_xi_ || yi != coeff_yi_ || zi != coeff_zi_)) {
    coeffs_ = getCoeffs(p);
    are_coeffs_computed_ = true;
    coeff_xi_ = xi;
    coeff_yi_ = yi;
    coeff_zi_ = zi;
  }

  int idx = 0;
  FieldVec field = FieldVec::Zero();
  for (int k = 0; k < 4; k++) {
    PositionVec::Scalar dz_k = pow(dp(2), k);
    PositionVec::Scalar dz_k_1 = 0;

    if (k > 0) dz_k_1 = pow(dp(2), k - 1);

    for (int j = 0; j < 4; j++) {
      PositionVec::Scalar dy_j = pow(dp(1), j);
      PositionVec::Scalar dy_j_1 = 0;

      if (j > 0) dy_j_1 = pow(dp(1), j - 1);

      for (int i = 0; i < 4; i++) {
        PositionVec::Scalar dx_i = pow(dp(0), i);
        PositionVec::Scalar dx_i_1 = 0;

        // dV/dx
        if (i > 0) {
          dx_i_1 = pow(dp(0), i - 1);
          field(0) += coeffs_(idx) * dx_i_1 * dy_j * dz_k * i;
        }

        if (j > 0) {
          field(1) += coeffs_(idx) * dx_i * dy_j_1 * dz_k * j;
        }

        if (k > 0) {
          field(2) += coeffs_(idx) * dx_i * dy_j * dz_k_1 * k;
        }

        idx += 1;
      }
    }
  }

  // we need to scale the gradient by the step size
  field(0) /= grid_props_.step_x;
  field(1) /= grid_props_.step_y;
  field(2) /= grid_props_.step_z;
  // The field is the negative of the gradient of the scalar potential
  return -field;
}

GradientMat InterpolateTricubicScalarField::getGradientImpl(const PositionVec& p) const {
  PositionVec p_n = getNormalizedPosition(p);
  // floor
  int xi = std::floor(p_n(0));
  int yi = std::floor(p_n(1));
  int zi = std::floor(p_n(2));
  PositionVec dp = p_n - PositionVec(xi, yi, zi);

  if (!are_coeffs_computed_ || (xi != coeff_xi_ || yi != coeff_yi_ || zi != coeff_zi_)) {
    coeffs_ = getCoeffs(p);
    are_coeffs_computed_ = true;
    coeff_xi_ = xi;
    coeff_yi_ = yi;
    coeff_zi_ = zi;
  }

  int idx = 0;
  GradientMat grad = GradientMat::Zero();
  for (int k = 0; k < 4; k++) {
    PositionVec::Scalar dz_k = pow(dp(2), k);
    PositionVec::Scalar dz_k_1 = 0;
    PositionVec::Scalar dz_k_2 = 0;
    if (k > 0) dz_k_1 = pow(dp(2), k - 1);
    if (k > 1) dz_k_2 = pow(dp(2), k - 2);

    for (int j = 0; j < 4; j++) {
      PositionVec::Scalar dy_j = pow(dp(1), j);
      PositionVec::Scalar dy_j_1 = 0;
      PositionVec::Scalar dy_j_2 = 0;
      if (j > 0) dy_j_1 = pow(dp(1), j - 1);
      if (j > 1) dy_j_2 = pow(dp(1), j - 2);

      for (int i = 0; i < 4; i++) {
        PositionVec::Scalar dx_i = pow(dp(0), i);
        PositionVec::Scalar dx_i_1 = 0;
        PositionVec::Scalar dx_i_2 = 0;
        if (i > 0) dx_i_1 = pow(dp(0), i - 1);
        if (i > 1) dx_i_2 = pow(dp(0), i - 2);

        if (i > 1) grad(0, 0) += i * (i - 1) * coeffs_(idx) * dx_i_2 * dy_j * dz_k;
        if (i > 0 && j > 0) grad(0, 1) += i * j * coeffs_(idx) * dx_i_1 * dy_j_1 * dz_k;
        if (i > 0 && k > 0) grad(0, 2) += i * k * coeffs_(idx) * dx_i_1 * dy_j * dz_k_1;
        if (j > 1) grad(1, 1) += j * (j - 1) * coeffs_(idx) * dx_i * dy_j_2 * dz_k;
        if (j > 0 && k > 0) grad(1, 2) += j * k * coeffs_(idx) * dx_i * dy_j_1 * dz_k_1;
        if (k > 1) grad(2, 2) += k * (k - 1) * coeffs_(idx) * dx_i * dy_j * dz_k_2;

        // the gradient is symmetric since the field has zero curl
        grad(1, 0) = grad(0, 1);
        grad(2, 0) = grad(0, 2);
        grad(2, 1) = grad(1, 2);

        idx += 1;
      }
    }
  }

  Eigen::Vector3d steps(grid_props_.step_x, grid_props_.step_y, grid_props_.step_z);
  Eigen::Matrix3d scales = steps * steps.transpose();
  return (-grad).array() / scales.array();
}

}  // namespace mag_manip
