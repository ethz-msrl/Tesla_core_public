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

#include <cmath>
#include <memory>
#include <string>

#include "ceres/cost_function.h"
#include "mag_manip/currents_jacobian_functor.h"
#include "mag_manip/forward_model.h"
#include "mag_manip/types.h"

namespace mag_manip {

class CurrentsRegularizationFunctor {
 public:
  CurrentsRegularizationFunctor(const int num_electomagnets, const double lambda)
      : num_em_(num_electomagnets), sqrt_lambda_(std::sqrt(lambda)) {}

  template <typename T>
  inline bool operator()(T const* const* currents, T* y) const {
    for (int i = 0; i < num_em_; i++) {
      y[i] = sqrt_lambda_ * currents[0][i];
    }

    return true;
  }

 private:
  const int num_em_;
  const double sqrt_lambda_;
};

class CurrentsFromFieldCostFunctor {
 public:
  /**
   * @brief Creates a cost function that computes the residual error for
   * computeCurrentsFromField.
   *
   * @param p_model: The model is assumed to be owned by this cost functor and should not be shared
   * with others
   * @param position: the position at which to calculate the residual
   * @param field: the desired field
   * @param use_cached_position: if true, will cache the position for faster
   * repeated residual evaluation
   */
  CurrentsFromFieldCostFunctor(ForwardModel::Ptr p_model, const PositionVec& position,
                               const FieldVec& field, bool use_cached_position);
  bool operator()(double const* const* parameters, double* residuals) const;

 private:
  ForwardModel::Ptr p_model_;
  PositionVec position_;
  FieldVec field_;
  int num_coils_;
  bool use_cached_position_;
};

class CurrentsFromFieldAnalyticalCostFunction : public ceres::CostFunction {
 public:
  /**
   * @brief ceres::CostFunction that computes the residuals for a desired field from currents
   * and an analytical Jacobian
   *
   * @param p_model: the forward model used in the computation. The forward model is assumed
   * to be valid already
   * @param position: the position of the desired field
   * @param field: the desired field
   * @param p_jac_func: assumed that the Jacobian object is already valid
   * @param use_cached_position: if set to true, it will use a cached position
   * which speeds up repeated computations
   */
  CurrentsFromFieldAnalyticalCostFunction(const ForwardModel::Ptr p_model,
                                          const PositionVec& position, const FieldVec& field,
                                          const CurrentsJacobianFunctor::Ptr p_jac_func,
                                          const bool use_cached_position);

  bool Evaluate(double const* const* parameters, double* residuals,
                double** jacobians) const override;

 private:
  const ForwardModel::Ptr p_forward_model_;
  const CurrentsJacobianFunctor::Ptr p_jac_func_;
  PositionVec position_;
  const bool use_cached_position_;
  FieldVec field_;
  int num_coils_;
};

class CurrentsFromFieldGradient5AnalyticalCostFunction : public ceres::CostFunction {
 public:
  CurrentsFromFieldGradient5AnalyticalCostFunction(ForwardModel::Ptr p_model,
                                                   const PositionVec& position,
                                                   const FieldVec& field,
                                                   const Gradient5Vec& gradient,
                                                   const CurrentsJacobianFunctor::Ptr p_jac_func,
                                                   const bool use_cached_position);

  bool Evaluate(double const* const* parameters, double* residuals,
                double** jacobians) const override;

 private:
  const ForwardModel::Ptr p_forward_model_;
  const CurrentsJacobianFunctor::Ptr p_jac_func_;
  const bool use_cached_position_;
  PositionVec position_;
  FieldVec field_;
  Gradient5Vec gradient_;
  int num_coils_;
};

class CurrentsFromFieldDipoleGradient3AnalyticalCostFunction : public ceres::CostFunction {
 public:
  CurrentsFromFieldDipoleGradient3AnalyticalCostFunction(
      ForwardModel::Ptr p_model, const PositionVec& position, const FieldVec& field,
      const DipoleVec& dipole, const Gradient3Vec& gradient,
      const CurrentsJacobianFunctor::Ptr p_jac_func, const bool use_cached_position);

  bool Evaluate(double const* const* parameters, double* residuals,
                double** jacobians) const override;

 private:
  const ForwardModel::Ptr p_forward_model_;
  const bool use_cached_position_;
  const CurrentsJacobianFunctor::Ptr p_jac_func_;
  PositionVec position_;
  FieldVec field_;
  DipoleVec dipole_;
  Gradient3Vec gradient_;
  int num_coils_;
};

class CurrentsFromFieldGradient5CostFunctor {
 public:
  CurrentsFromFieldGradient5CostFunctor(ForwardModel::Ptr p_model, const PositionVec& position,
                                        const FieldVec& field, const Gradient5Vec& gradient,
                                        const bool use_cached_position);

  bool operator()(double const* const* parameters, double* residuals) const;

 private:
  ForwardModel::Ptr p_model_;
  const bool use_cached_position_;
  PositionVec position_;
  FieldGradient5Vec field_gradient_;
  int num_coils_;
};

class CurrentsFromFieldDipoleGradient3CostFunctor {
 public:
  CurrentsFromFieldDipoleGradient3CostFunctor(ForwardModel::Ptr p_model,
                                              const PositionVec& position, const FieldVec& field,
                                              const DipoleVec& dipole, const Gradient3Vec& gradient,
                                              const bool use_cached_position);

  bool operator()(double const* const* parameters, double* residuals) const;

 private:
  ForwardModel::Ptr p_model_;
  const bool use_cached_position_;
  PositionVec position_;
  FieldVec field_;
  DipoleVec dipole_;
  Gradient3Vec gradient_;
  int num_coils_;
};
}  // namespace mag_manip
