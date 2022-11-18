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

#include "mag_manip/currents_jacobian_functor.h"
#include "mag_manip/forward_model_linear_saturation.h"

namespace mag_manip {
/**
 * @brief Computes the analytical jacobian of a ForwardLinear model with saturation.
 * This is used by the Ceres CostFunction ComputeCurrentsFromAnalyticalCostFunctor
 *
 * WARNING: Make sure that you set the forward model and saturation functions identically to the
 * CostFunction!
 */
class ForwardModelLinearSaturationCurrentsJacobianFunctor : public CurrentsJacobianFunctor {
 public:
  /**
   * @brief Gets the derivative in field_gradient5 to the currents
   * using analytical derivatives
   *
   * @param position: position at which to calculate the Jacobian
   * @param currents: currents vector at which to calculate the Jacobian
   *
   * @return a 8xNe matrix containing the derivative of field_gradient_matrix
   * to current values
   */
  virtual CurrentsJacobian operator()(const PositionVec& position,
                                      const CurrentsVec& currents) const override;

  virtual int getNumCoils() const override;

  void setForwardModelLinearSaturation(ForwardModelLinearSaturation::Ptr p_forward_model_sat);

  ForwardModelLinearSaturation::Ptr getForwardModelLinearSaturation() const;

  /**
   * @brief sets the position cache for faster repeated computations
   *
   * Must be called after setForwardModelLinearSaturation
   *
   * @param position to cache
   */
  virtual void setCachedPosition(const PositionVec& position) override;

  /**
   * @brief Returns the cached position
   *
   * @return the cached position
   */
  virtual PositionVec getCachedPosition() const override;

  /**
   * @brief Cached version of the functor that does not recompute the
   * position dependence
   *
   * @param currents: currents vector at which to calculate the Jacobian
   *
   * @return a 8xNe matrix containing the derivative of field_gradient_matrix
   * to current values
   */
  virtual CurrentsJacobian cached(const CurrentsVec& currents) const override;

 protected:
  ForwardModelLinearSaturation::Ptr p_forward_model_sat_;

 private:
  ActuationMat act_mat_cached_;
  PositionVec position_cached_;
};
}  // namespace mag_manip
