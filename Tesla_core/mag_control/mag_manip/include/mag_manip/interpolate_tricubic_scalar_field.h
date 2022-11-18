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
#include <Eigen/OrderingMethods>
#include <Eigen/SparseCore>
#include <Eigen/SparseQR>
#include "mag_manip/interpolate_regular.h"

namespace mag_manip {

/**
 * @brief We use the base-from-member idiom in order to prescale the data before passing it to the
 * InterpolateRegular base class. In contrast to the InterpolateTricubic method, the data needs to
 * be scaled by the step sizes
 */
struct InterpolateTricubicScalarFieldDetail {
  InterpolateTricubicScalarFieldDetail(const DataMat& data, const VFieldGridProperties& props) {
    data_scaled_ =
        data.array().colwise() * Eigen::Vector3d(props.step_x, props.step_y, props.step_z).array();
  }

  DataMat data_scaled_;
};

/**
 * @brief  Performs tricubic interpolation on a magnetic scalar potential.
 * The magnetic field is the negative gradient of the scalar potential
 * The magnetic field at the voxel corners, as well as dbx/dy dbx/dz
 * dby/dz d2bx/dydz are constrained at the corners. the laplacian is
 * also constrained to be 0 at the voxel corners
 */
class InterpolateTricubicScalarField : private InterpolateTricubicScalarFieldDetail,
                                       public InterpolateRegular {
 public:
  using Coeffs = Eigen::VectorXd;
  using SpMat = Eigen::SparseMatrix<double>;

  InterpolateTricubicScalarField(const DataMat& data, const VFieldGridProperties& props);

 private:
  InterpolateTricubicScalarField::Coeffs getCoeffs(const PositionVec& p) const;

  virtual FieldVec interpolateImpl(const PositionVec& p) const override;

  virtual GradientMat getGradientImpl(const PositionVec& p) const override;

  SpMat m_;
  Eigen::SparseQR<SpMat, Eigen::COLAMDOrdering<int> > qr_;
  mutable bool are_coeffs_computed_;
  mutable int coeff_xi_, coeff_yi_, coeff_zi_;
  mutable Coeffs coeffs_;
};

}  // namespace mag_manip
