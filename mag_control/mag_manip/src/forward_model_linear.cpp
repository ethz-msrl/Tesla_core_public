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

#include "mag_manip/forward_model_linear.h"
#include "mag_manip/exceptions.h"

using namespace mag_manip;

ActuationMat ForwardModelLinear::getFieldActuationMatrix(const PositionVec& position) const {
  ActuationMat m = getActuationMatrix(position);
  return m.topRows<3>();
}

FieldVec ForwardModelLinear::computeFieldFromCurrents(const PositionVec& position,
                                                      const CurrentsVec& currents) const {
  if (!isValid()) {
    throw CalibrationNotLoaded();
  }

  if (currents.size() != getNumCoils()) {
    throw InvalidCurrentsLength();
  }

  return getFieldActuationMatrix(position) * currents;
}

Gradient5Vec ForwardModelLinear::computeGradient5FromCurrents(const PositionVec& position,
                                                              const CurrentsVec& currents) const {
  if (!isValid()) {
    throw CalibrationNotLoaded();
  }
  if (currents.size() != getNumCoils()) {
    throw InvalidCurrentsLength();
  }

  ActuationMat act_mat = getActuationMatrix(position);
  ActuationMat grad_act_mat = act_mat.bottomRows(5);
  return grad_act_mat * currents;
}

FieldGradient5Vec ForwardModelLinear::computeFieldGradient5FromCurrents(
    const PositionVec& position, const CurrentsVec& currents) const {
  if (!isValid()) {
    throw CalibrationNotLoaded();
  }
  if (currents.size() != getNumCoils()) {
    throw InvalidCurrentsLength();
  }

  ActuationMat act_mat = getActuationMatrix(position);
  return act_mat * currents;
}

void ForwardModelLinear::setCachedPosition(const PositionVec& position) {
  act_mat_cached_ = getActuationMatrix(position);
  position_cached_ = position;
}

PositionVec ForwardModelLinear::getCachedPosition() const { return position_cached_; }

FieldVec ForwardModelLinear::computeFieldFromCurrentsCached(const CurrentsVec& currents) const {
  if (act_mat_cached_.size() == 0) {
    throw NotCachedException();
  }

  return act_mat_cached_.topRows<3>() * currents;
}

Gradient5Vec ForwardModelLinear::computeGradient5FromCurrentsCached(
    const CurrentsVec& currents) const {
  if (act_mat_cached_.size() == 0) {
    throw NotCachedException();
  }

  return act_mat_cached_.bottomRows<5>() * currents;
}

FieldGradient5Vec ForwardModelLinear::computeFieldGradient5FromCurrentsCached(
    const CurrentsVec& currents) const {
  if (act_mat_cached_.size() == 0) {
    throw NotCachedException();
  }

  return act_mat_cached_ * currents;
}
