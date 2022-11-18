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

#include "mag_manip/interpolate_tricubic.h"
#include <cmath>
#include <iostream>

using namespace std;
using namespace Eigen;
using namespace mag_manip;

InterpolateTricubic::InterpolateTricubic(const DataMat& data, const VFieldGridProperties& props)
    : InterpolateRegular(data, props), are_coeffs_computed_(false) {}

InterpolateTricubic::Coeffs InterpolateTricubic::getCoeffs(const PositionVec& p) const {
  // see http://www.cds.caltech.edu/~marsden/bib/2005/08-LeMa2005/LeMa2005.pdf
  PositionVec p_n = getNormalizedPosition(p);

  // normalized position
  PositionVec::Scalar x_ix = p_n(0);
  PositionVec::Scalar y_ix = p_n(1);
  PositionVec::Scalar z_ix = p_n(2);

  // floor
  int xi = std::floor(x_ix);
  int yi = std::floor(y_ix);
  int zi = std::floor(z_ix);

  InterpolateTricubic::Coeffs X(64, 3);
  // values of f(x,y,z)
  X.row(0) = getAtIndex(xi, yi, zi);
  X.row(1) = getAtIndex(xi + 1, yi, zi);
  X.row(2) = getAtIndex(xi, yi + 1, zi);
  X.row(3) = getAtIndex(xi + 1, yi + 1, zi);
  X.row(4) = getAtIndex(xi, yi, zi + 1);
  X.row(5) = getAtIndex(xi + 1, yi, zi + 1);
  X.row(6) = getAtIndex(xi, yi + 1, zi + 1);
  X.row(7) = getAtIndex(xi + 1, yi + 1, zi + 1);
  // values of df/dx
  X.row(8) = getDataMatD_dx(xi, yi, zi);
  X.row(9) = getDataMatD_dx(xi + 1, yi, zi);
  X.row(10) = getDataMatD_dx(xi, yi + 1, zi);
  X.row(11) = getDataMatD_dx(xi + 1, yi + 1, zi);
  X.row(12) = getDataMatD_dx(xi, yi, zi + 1);
  X.row(13) = getDataMatD_dx(xi + 1, yi, zi + 1);
  X.row(14) = getDataMatD_dx(xi, yi + 1, zi + 1);
  X.row(15) = getDataMatD_dx(xi + 1, yi + 1, zi + 1);
  // values of df/dy
  X.row(16) = getDataMatD_dy(xi, yi, zi);
  X.row(17) = getDataMatD_dy(xi + 1, yi, zi);
  X.row(18) = getDataMatD_dy(xi, yi + 1, zi);
  X.row(19) = getDataMatD_dy(xi + 1, yi + 1, zi);
  X.row(20) = getDataMatD_dy(xi, yi, zi + 1);
  X.row(21) = getDataMatD_dy(xi + 1, yi, zi + 1);
  X.row(22) = getDataMatD_dy(xi, yi + 1, zi + 1);
  X.row(23) = getDataMatD_dy(xi + 1, yi + 1, zi + 1);
  // values of df/dz
  X.row(24) = getDataMatD_dz(xi, yi, zi);
  X.row(25) = getDataMatD_dz(xi + 1, yi, zi);
  X.row(26) = getDataMatD_dz(xi, yi + 1, zi);
  X.row(27) = getDataMatD_dz(xi + 1, yi + 1, zi);
  X.row(28) = getDataMatD_dz(xi, yi, zi + 1);
  X.row(29) = getDataMatD_dz(xi + 1, yi, zi + 1);
  X.row(30) = getDataMatD_dz(xi, yi + 1, zi + 1);
  X.row(31) = getDataMatD_dz(xi + 1, yi + 1, zi + 1);
  // values of d2f/dxdy
  X.row(32) = getDataMatD2_dxdy(xi, yi, zi);
  X.row(33) = getDataMatD2_dxdy(xi + 1, yi, zi);
  X.row(34) = getDataMatD2_dxdy(xi, yi + 1, zi);
  X.row(35) = getDataMatD2_dxdy(xi + 1, yi + 1, zi);
  X.row(36) = getDataMatD2_dxdy(xi, yi, zi + 1);
  X.row(37) = getDataMatD2_dxdy(xi + 1, yi, zi + 1);
  X.row(38) = getDataMatD2_dxdy(xi, yi + 1, zi + 1);
  X.row(39) = getDataMatD2_dxdy(xi + 1, yi + 1, zi + 1);
  // values of d2f/dxdz
  X.row(40) = getDataMatD2_dxdz(xi, yi, zi);
  X.row(41) = getDataMatD2_dxdz(xi + 1, yi, zi);
  X.row(42) = getDataMatD2_dxdz(xi, yi + 1, zi);
  X.row(43) = getDataMatD2_dxdz(xi + 1, yi + 1, zi);
  X.row(44) = getDataMatD2_dxdz(xi, yi, zi + 1);
  X.row(45) = getDataMatD2_dxdz(xi + 1, yi, zi + 1);
  X.row(46) = getDataMatD2_dxdz(xi, yi + 1, zi + 1);
  X.row(47) = getDataMatD2_dxdz(xi + 1, yi + 1, zi + 1);
  // values of d2f/dydz
  X.row(48) = getDataMatD2_dydz(xi, yi, zi);
  X.row(49) = getDataMatD2_dydz(xi + 1, yi, zi);
  X.row(50) = getDataMatD2_dydz(xi, yi + 1, zi);
  X.row(51) = getDataMatD2_dydz(xi + 1, yi + 1, zi);
  X.row(52) = getDataMatD2_dydz(xi, yi, zi + 1);
  X.row(53) = getDataMatD2_dydz(xi + 1, yi, zi + 1);
  X.row(54) = getDataMatD2_dydz(xi, yi + 1, zi + 1);
  X.row(55) = getDataMatD2_dydz(xi + 1, yi + 1, zi + 1);
  // values of d3f/dxdydz
  X.row(56) = getDataMatD3_dxdydz(xi, yi, zi);
  X.row(57) = getDataMatD3_dxdydz(xi + 1, yi, zi);
  X.row(58) = getDataMatD3_dxdydz(xi, yi + 1, zi);
  X.row(59) = getDataMatD3_dxdydz(xi + 1, yi + 1, zi);
  X.row(60) = getDataMatD3_dxdydz(xi, yi, zi + 1);
  X.row(61) = getDataMatD3_dxdydz(xi + 1, yi, zi + 1);
  X.row(62) = getDataMatD3_dxdydz(xi, yi + 1, zi + 1);
  X.row(63) = getDataMatD3_dxdydz(xi + 1, yi + 1, zi + 1);

  Map<const Matrix<int, 64, 64, RowMajor> > m(TRICUBIC_COEFFS);
  Eigen::Matrix<PositionVec::Scalar, 64, 64> M = m.cast<PositionVec::Scalar>();
  InterpolateTricubic::Coeffs aijk = m.cast<PositionVec::Scalar>() * X;
  return aijk;
}

FieldVec InterpolateTricubic::interpolateImpl(const PositionVec& p) const {
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

  FieldVec b1 = interpolateNaive(dp, coeffs_);

  return b1;
}

FieldVec InterpolateTricubic::interpolateNaive(const PositionVec& p,
                                               const InterpolateTricubic::Coeffs& aijk) const {
  int idx = 0;
  FieldVec value = FieldVec::Zero();
  for (int k = 0; k < 4; k++) {
    PositionVec::Scalar dz_k = pow(p(2), k);
    for (int j = 0; j < 4; j++) {
      PositionVec::Scalar dy_j = pow(p(1), j);
      for (int i = 0; i < 4; i++) {
        // cout << "idx: " << idx << " aijk: " << aijk.row(idx)
        //<< " pow: " << pow(p(0), i) * pow(p(1), j) * pow(p(2), k) << endl;
        value += aijk.row(idx) * pow(p(0), i) * dy_j * dz_k;
        idx += 1;
      }
    }
  }
  return value;
}

// Vector3f InterpolateTricubic::interpolate_opt(const Vector3f& p, const TricubicCoeffs& aijk)
// const {
// Vector4f x_m(1, p(0), p(0)*p(0), p(0)*p(0)*p(0));
// Vector4f y_m(1, p(1), p(1)*p(1), p(0)*p(0)*p(0));
// Vector4f z_m(1, p(2), p(2)*p(2), p(2)*p(2)*p(2));
// Map<const Matrix<float, 16, 1> > yx_mon((y_m.transpose() * x_m).data(), 16);
// Matrix<float, 64, 1> xyz_mon;
// xyz_mon << x_m(0) * yx_mon, x_m(1) * yx_mon, x_m(2) * yx_mon, x_m(3) * yx_mon;
// return aijk.transpose() * xyz_mon;
//}

GradientMat InterpolateTricubic::getGradientImpl(const PositionVec& p) const {
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

    if (k > 0) dz_k_1 = pow(dp(2), k - 1);

    for (int j = 0; j < 4; j++) {
      PositionVec::Scalar dy_j = pow(dp(1), j);
      PositionVec::Scalar dy_j_1 = 0;

      if (j > 0) dy_j_1 = pow(dp(1), j - 1);

      for (int i = 0; i < 4; i++) {
        // cout << "idx: " << idx << " aijk: " << aijk.row(idx)
        //<< " pow: " << pow(p(0), i) * pow(p(1), j) * pow(p(2), k) << endl;
        PositionVec::Scalar dx_i = pow(dp(0), i);
        PositionVec::Scalar dx_i_1 = 0;

        // dV/dx
        if (i > 0) {
          dx_i_1 = pow(dp(0), i - 1);
          grad.col(0) += coeffs_.row(idx) * dx_i_1 * dy_j * dz_k * i;
        }

        if (j > 0) {
          grad.col(1) += coeffs_.row(idx) * dx_i * dy_j_1 * dz_k * j;
        }

        if (k > 0) {
          grad.col(2) += coeffs_.row(idx) * dx_i * dy_j * dz_k_1 * k;
        }

        idx += 1;
      }
    }
  }

  grad.col(0) /= grid_props_.step_x;
  grad.col(1) /= grid_props_.step_y;
  grad.col(2) /= grid_props_.step_z;
  return grad;
}
