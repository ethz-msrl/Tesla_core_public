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

#include "mag_manip/forward_model_linear_currents_jacobian_functor.h"

#include <stdexcept>

using namespace mag_manip;
using namespace std;

void ForwardModelLinearCurrentsJacobianFunctor::setForwardModelLinear(
    ForwardModelLinear::Ptr p_forward_model) {
  p_forward_model_ = p_forward_model;
}

ForwardModelLinear::Ptr ForwardModelLinearCurrentsJacobianFunctor::getForwardModelLinear() const {
  return p_forward_model_;
}

CurrentsJacobian ForwardModelLinearCurrentsJacobianFunctor::operator()(
    const PositionVec& position, const CurrentsVec& currents) const {
  ActuationMat act_mat = p_forward_model_->getActuationMatrix(position);
  return act_mat;
}

int ForwardModelLinearCurrentsJacobianFunctor::getNumCoils() const {
  return p_forward_model_->getNumCoils();
}

void ForwardModelLinearCurrentsJacobianFunctor::setCachedPosition(const PositionVec& position) {
  act_mat_cached_ = p_forward_model_->getActuationMatrix(position);
  position_cached_ = position;
}

PositionVec ForwardModelLinearCurrentsJacobianFunctor::getCachedPosition() const {
  return position_cached_;
}

CurrentsJacobian ForwardModelLinearCurrentsJacobianFunctor::cached(
    const CurrentsVec& currents) const {
  if (act_mat_cached_.size() == 0) throw NotCachedException();
  return act_mat_cached_;
}
