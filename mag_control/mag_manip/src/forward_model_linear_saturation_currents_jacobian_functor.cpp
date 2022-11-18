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

#include "mag_manip/forward_model_linear_saturation_currents_jacobian_functor.h"

#include <stdexcept>

using namespace mag_manip;
using namespace std;

void ForwardModelLinearSaturationCurrentsJacobianFunctor::setForwardModelLinearSaturation(
    ForwardModelLinearSaturation::Ptr p_forward_model_sat) {
  p_forward_model_sat_ = p_forward_model_sat;
}

ForwardModelLinearSaturation::Ptr
ForwardModelLinearSaturationCurrentsJacobianFunctor::getForwardModelLinearSaturation() const {
  return p_forward_model_sat_;
}

CurrentsJacobian ForwardModelLinearSaturationCurrentsJacobianFunctor::operator()(
    const PositionVec& position, const CurrentsVec& currents) const {
  const auto sat_functions = p_forward_model_sat_->getSaturationFunctions();
  const int Ne = currents.size();

  if (sat_functions.size() != Ne) {
    throw InvalidCurrentsLength();
  }

  Eigen::VectorXd jac_sats(Ne);

  ActuationMat act_mat = p_forward_model_sat_->getLinearModel()->getActuationMatrix(position);
  CurrentsJacobian jac(act_mat.rows(), Ne);
  for (int k = 0; k < Ne; k++) {
    jac.col(k) = act_mat.col(k) * sat_functions[k]->jacobian(currents(k))(0);
  }

  return jac;
}

int ForwardModelLinearSaturationCurrentsJacobianFunctor::getNumCoils() const {
  return p_forward_model_sat_->getNumCoils();
}

void ForwardModelLinearSaturationCurrentsJacobianFunctor::setCachedPosition(
    const PositionVec& position) {
  act_mat_cached_ = p_forward_model_sat_->getLinearModel()->getActuationMatrix(position);
  position_cached_ = position;
}

PositionVec ForwardModelLinearSaturationCurrentsJacobianFunctor::getCachedPosition() const {
  return position_cached_;
}

CurrentsJacobian ForwardModelLinearSaturationCurrentsJacobianFunctor::cached(
    const CurrentsVec& currents) const {
  const auto sat_functions = p_forward_model_sat_->getSaturationFunctions();
  const int Ne = currents.size();

  if (sat_functions.size() != Ne) {
    throw InvalidCurrentsLength();
  }

  if (act_mat_cached_.size() == 0) throw NotCachedException();

  Eigen::VectorXd jac_sats(Ne);

  CurrentsJacobian jac(act_mat_cached_.rows(), Ne);
  for (int k = 0; k < Ne; k++) {
    jac.col(k) = act_mat_cached_.col(k) * sat_functions[k]->jacobian(currents(k))(0);
  }

  return jac;
}
