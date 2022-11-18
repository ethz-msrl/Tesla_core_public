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

#include "mag_manip/backward_model_linear_L2.h"
#include <Eigen/SVD>
#include "mag_manip/exceptions.h"
#include "mag_manip/helpers.h"

using namespace Eigen;

namespace mag_manip {

ActuationMat BackwardModelLinearL2::getFieldActuationMatrix(const PositionVec& position) const {
  return this->getActuationMatrix(position).topRows<3>();
}

ActuationMat BackwardModelLinearL2::getActuationMatrixInverse(const PositionVec& position) const {
  ActuationMat act_mat = this->getActuationMatrix(position);

  JacobiSVD<ActuationMat> svd(act_mat, ComputeThinU | ComputeThinV);

  // this is the value where a singular value is considered 0
  const float eps = std::numeric_limits<float>::epsilon();

  // the minimum singular value
  float tol =
      eps * std::max(act_mat.cols(), act_mat.rows()) * svd.singularValues().array().abs()(0);

  // return the pseudoinverse
  return svd.matrixV() *
         (svd.singularValues().array().abs() > tol)
             .select(svd.singularValues().array().inverse(), 0)
             .matrix()
             .asDiagonal() *
         svd.matrixU().adjoint();
}

ActuationMat BackwardModelLinearL2::getFieldActuationMatrixInverse(
    const PositionVec& position) const {
  ActuationMat act_mat = this->getActuationMatrix(position);
  ActuationMat field_act_mat = act_mat.topRows<3>();

  JacobiSVD<ActuationMat> svd(field_act_mat, ComputeThinU | ComputeThinV);

  // this is the value where a singular value is considered 0
  const float eps = std::numeric_limits<float>::epsilon();

  // the minimum singular value
  float tol = eps * std::max(field_act_mat.cols(), field_act_mat.rows()) *
              svd.singularValues().array().abs()(0);

  // return the pseudoinverse
  return svd.matrixV() *
         (svd.singularValues().array().abs() > tol)
             .select(svd.singularValues().array().inverse(), 0)
             .matrix()
             .asDiagonal() *
         svd.matrixU().adjoint();
}

CurrentsVec BackwardModelLinearL2::computeCurrentsFromField(const PositionVec& position,
                                                            const FieldVec& field) const {
  ActuationMat act_mat = this->getActuationMatrix(position);
  ActuationMat field_act_mat = act_mat.topRows<3>();

  JacobiSVD<ActuationMat> svd(field_act_mat, ComputeThinU | ComputeThinV);

  CurrentsVec currents = svd.solve(field);
  return currents;
}

CurrentsVec BackwardModelLinearL2::computeCurrentsFromFieldGradient5(
    const PositionVec& position, const FieldVec& field, const Gradient5Vec& gradient) const {
  ActuationMat act_mat = this->getActuationMatrix(position);

  FieldGradient5Vec field_gradient;
  field_gradient << field, gradient;

  JacobiSVD<ActuationMat> svd(act_mat, ComputeThinU | ComputeThinV);

  CurrentsVec currents = svd.solve(field_gradient);
  return currents;
}

CurrentsVec BackwardModelLinearL2::computeCurrentsFromFieldDipoleGradient3(
    const PositionVec& position, const FieldVec& field, const DipoleVec& dipole,
    const Gradient3Vec& gradient) const {
  ActuationMat act_mat = this->getActuationMatrix(position);

  // we make a new actuation matrix that is 6xNe that converts currents
  // to field and directed 3D gradient
  ActuationMat act_mat_mod(6, getNumCoils());
  act_mat_mod.topRows<3>() = act_mat.topRows<3>();
  Grad35Mat directed_grad_mat = directedGradient3Mat(dipole);
  act_mat_mod.bottomRows<3>() = directed_grad_mat * act_mat.bottomRows<5>();

  FieldGradient3Vec field_gradient;
  field_gradient << field, gradient;

  JacobiSVD<ActuationMat> svd(act_mat_mod, ComputeThinU | ComputeThinV);

  CurrentsVec currents = svd.solve(field_gradient);
  return currents;
}

void BackwardModelLinearL2::setCachedPosition(const PositionVec& position) {
  ActuationMat act_mat = this->getActuationMatrix(position);
  ActuationMat field_act_mat = act_mat.topRows<3>();

  // we make a new actuation matrix that is 6xNe that converts currents
  // to field and directed 3D gradient
  ActuationMat act_mat_mod(6, getNumCoils());
  act_mat_mod.topRows<3>() = act_mat.topRows<3>();
  // Grad35Mat directed_grad_mat = directedGradient3Mat(dipole);
  // act_mat_mod.bottomRows<3>() = directed_grad_mat * act_mat.bottomRows<5>();

  svd_field_cached_.compute(field_act_mat, ComputeThinU | ComputeThinV);
  svd_field_grad_cached_.compute(act_mat, ComputeThinU | ComputeThinV);
  // svd_field_grad_mod_cached_.compute(act_mat_mod, ComputeThinU | ComputeThinV);

  position_cached_ = position;
}

void BackwardModelLinearL2::setCachedPositionDipole(const PositionVec& position,
                                                    const DipoleVec& dipole) {
  ActuationMat act_mat = this->getActuationMatrix(position);
  // we make a new actuation matrix that is 6xNe that converts currents
  // to field and directed 3D gradient
  ActuationMat act_mat_mod(6, getNumCoils());
  act_mat_mod.topRows<3>() = act_mat.topRows<3>();
  Grad35Mat directed_grad_mat = directedGradient3Mat(dipole);
  act_mat_mod.bottomRows<3>() = directed_grad_mat * act_mat.bottomRows<5>();
  svd_field_grad_mod_cached_.compute(act_mat_mod, ComputeThinU | ComputeThinV);
  dipole_cached_ = dipole;
  setCachedPosition(position);
}

DipoleVec BackwardModelLinearL2::getCachedDipole() const { return dipole_cached_; }

PositionVec BackwardModelLinearL2::getCachedPosition() const { return position_cached_; }

CurrentsVec BackwardModelLinearL2::computeCurrentsFromFieldCached(const FieldVec& field) const {
  if (position_cached_.size() == 0) {
    throw NotCachedException();
  }

  return svd_field_cached_.solve(field);
}

CurrentsVec BackwardModelLinearL2::computeCurrentsFromFieldGradient5Cached(
    const FieldVec& field, const Gradient5Vec& gradient) const {
  if (position_cached_.size() == 0) {
    throw NotCachedException();
  }

  FieldGradient5Vec field_gradient;
  field_gradient << field, gradient;

  return svd_field_grad_cached_.solve(field_gradient);
}

CurrentsVec BackwardModelLinearL2::computeCurrentsFromFieldDipoleGradient3Cached(
    const FieldVec& field, const Gradient3Vec& gradient) const {
  if (position_cached_.size() == 0 || dipole_cached_.size() == 0) {
    throw NotCachedException();
  }

  FieldGradient3Vec field_gradient;
  field_gradient << field, gradient;

  return svd_field_grad_mod_cached_.solve(field_gradient);
}

}  // namespace mag_manip
