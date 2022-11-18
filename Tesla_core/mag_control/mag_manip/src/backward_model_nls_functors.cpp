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

#include "mag_manip/backward_model_nls_functors.h"

#include "mag_manip/exceptions.h"
#include "mag_manip/helpers.h"

using namespace mag_manip;
using namespace std;

CurrentsFromFieldCostFunctor::CurrentsFromFieldCostFunctor(ForwardModel::Ptr p_forward_model,
                                                           const PositionVec& position,
                                                           const FieldVec& field,
                                                           bool use_cached_position)
    : p_model_(p_forward_model),
      position_(position),
      field_(field),
      use_cached_position_(use_cached_position) {
  num_coils_ = p_model_->getNumCoils();
  if (use_cached_position_) {
    p_model_->setCachedPosition(position);
  }
}

bool CurrentsFromFieldCostFunctor::operator()(double const* const* parameters,
                                              double* residuals) const {
  Eigen::Map<const CurrentsVec> currents(parameters[0], num_coils_);

  // Not sure if it's better to let the model fail or return false telling
  // Ceres the residual computation failed
  FieldVec field_pred;

  try {
    if (use_cached_position_) {
      field_pred = p_model_->computeFieldFromCurrentsCached(currents);
    } else {
      field_pred = p_model_->computeFieldFromCurrents(position_, currents);
    }
  } catch (std::exception& e) {
    return false;
  }

  residuals[0] = field_(0) - field_pred(0);
  residuals[1] = field_(1) - field_pred(1);
  residuals[2] = field_(2) - field_pred(2);

  return true;
}

CurrentsFromFieldAnalyticalCostFunction::CurrentsFromFieldAnalyticalCostFunction(
    const ForwardModel::Ptr p_forward_model, const PositionVec& position, const FieldVec& field,
    const CurrentsJacobianFunctor::Ptr p_jac_func, bool use_cached_position)
    : p_forward_model_(p_forward_model),
      position_(position),
      field_(field),
      p_jac_func_(p_jac_func),
      use_cached_position_(use_cached_position) {
  num_coils_ = p_forward_model->getNumCoils();

  if (p_jac_func_->getNumCoils() != num_coils_) {
    throw InvalidCurrentsLength();
  }

  // we already know what the size of the parameters are
  // since we have the number of electromagnets
  mutable_parameter_block_sizes()->push_back(num_coils_);
  // residuals is just the field so of length 3
  set_num_residuals(3);

  if (use_cached_position_) {
    p_forward_model_->setCachedPosition(position_);
    p_jac_func_->setCachedPosition(position_);
  }
}

bool CurrentsFromFieldAnalyticalCostFunction::Evaluate(double const* const* parameters,
                                                       double* residuals,
                                                       double** jacobians) const {
  Eigen::Map<const CurrentsVec> currents(parameters[0], num_coils_);

  // Not sure if it's better to let the model fail or return false telling
  // Ceres the residual computation failed
  FieldVec field_pred;
  try {
    if (use_cached_position_) {
      field_pred = p_forward_model_->computeFieldFromCurrentsCached(currents);
    } else {
      field_pred = p_forward_model_->computeFieldFromCurrents(position_, currents);
    }
  } catch (std::exception& e) {
    return false;
  }

  residuals[0] = field_(0) - field_pred(0);
  residuals[1] = field_(1) - field_pred(1);
  residuals[2] = field_(2) - field_pred(2);

  // only one parameter block of size Ne for the currents
  if (jacobians != nullptr && jacobians[0] != nullptr) {
    // here we also catch exceptions as failures in the single evaluation that don't necessarily
    // interrupt the whole computation
    try {
      // the currents jacobian contains the field and gradient jacobians. we only need the top 3
      // rows for the field
      CurrentsJacobian currents_jac;
      if (use_cached_position_) {
        currents_jac = p_jac_func_->cached(currents).topRows<3>();
      } else {
        currents_jac = (*p_jac_func_)(position_, currents).topRows<3>();
      }
      // rows are the entries of the field
      for (int i = 0; i < 3; i++) {
        // cols are entries of the current
        for (int k = 0; k < num_coils_; k++) {
          // the residual is bd - f(i) so the jacobian is
          // -df(i)/di
          jacobians[0][i * num_coils_ + k] = -currents_jac(i, k);
        }
      }
    } catch (std::exception& e) {
      return false;
    }
  }

  return true;
}

CurrentsFromFieldGradient5AnalyticalCostFunction::CurrentsFromFieldGradient5AnalyticalCostFunction(
    const ForwardModel::Ptr p_forward_model, const PositionVec& position, const FieldVec& field,
    const Gradient5Vec& gradient, const CurrentsJacobianFunctor::Ptr p_jac_func,
    const bool use_cached_position)
    : p_forward_model_(p_forward_model),
      use_cached_position_(use_cached_position),
      position_(position),
      field_(field),
      gradient_(gradient),
      p_jac_func_(p_jac_func) {
  num_coils_ = p_forward_model->getNumCoils();

  if (p_jac_func_->getNumCoils() != num_coils_) {
    throw InvalidCurrentsLength();
  }

  // we already know what the size of the parameters are
  // since we have the number of electromagnets
  mutable_parameter_block_sizes()->push_back(num_coils_);
  set_num_residuals(8);

  if (use_cached_position_) p_forward_model->setCachedPosition(position);
}

bool CurrentsFromFieldGradient5AnalyticalCostFunction::Evaluate(double const* const* parameters,
                                                                double* residuals,
                                                                double** jacobians) const {
  Eigen::Map<const CurrentsVec> currents(parameters[0], num_coils_);

  // Not sure if it's better to let the model fail or return false telling
  // Ceres the residual computation failed
  FieldGradient5Vec field_grad_pred;
  try {
    if (use_cached_position_)
      field_grad_pred = p_forward_model_->computeFieldGradient5FromCurrentsCached(currents);
    else
      field_grad_pred = p_forward_model_->computeFieldGradient5FromCurrents(position_, currents);
  } catch (std::exception& e) {
    return false;
  }

  residuals[0] = field_(0) - field_grad_pred(0);
  residuals[1] = field_(1) - field_grad_pred(1);
  residuals[2] = field_(2) - field_grad_pred(2);
  residuals[3] = gradient_(0) - field_grad_pred(3);
  residuals[4] = gradient_(1) - field_grad_pred(4);
  residuals[5] = gradient_(2) - field_grad_pred(5);
  residuals[6] = gradient_(3) - field_grad_pred(6);
  residuals[7] = gradient_(4) - field_grad_pred(7);

  // only one parameter block of size Ne for the currents
  if (jacobians != nullptr && jacobians[0] != nullptr) {
    // here we also catch exceptions as failures in the single evaluation that don't necessarily
    // interrupt the whole computation
    try {
      // the currents jacobian contains the field and gradient jacobians. we only need the top 3
      // rows for the field
      CurrentsJacobian currents_jac;
      if (use_cached_position_) {
        currents_jac = p_jac_func_->cached(currents);
      } else {
        currents_jac = (*p_jac_func_)(position_, currents);
      }
      // rows are the entries of the field
      for (int i = 0; i < 8; i++) {
        // cols are entries of the current
        for (int k = 0; k < num_coils_; k++) {
          // the residual is bd - f(i) so the jacobian is
          // -df(i)/di
          jacobians[0][i * num_coils_ + k] = -currents_jac(i, k);
        }
      }
    } catch (std::exception& e) {
      return false;
    }
  }

  return true;
}

CurrentsFromFieldDipoleGradient3AnalyticalCostFunction::
    CurrentsFromFieldDipoleGradient3AnalyticalCostFunction(
        ForwardModel::Ptr p_forward_model, const PositionVec& position, const FieldVec& field,
        const DipoleVec& dipole, const Gradient3Vec& gradient,
        const CurrentsJacobianFunctor::Ptr p_jac_func, const bool use_cached_position)
    : p_forward_model_(p_forward_model),
      use_cached_position_(use_cached_position),
      position_(position),
      field_(field),
      dipole_(dipole),
      gradient_(gradient),
      p_jac_func_(p_jac_func) {
  num_coils_ = p_forward_model->getNumCoils();

  if (p_jac_func_->getNumCoils() != num_coils_) {
    throw InvalidCurrentsLength();
  }

  // we already know what the size of the parameters are
  // since we have the number of electromagnets
  mutable_parameter_block_sizes()->push_back(num_coils_);
  set_num_residuals(6);

  if (use_cached_position_) {
    p_forward_model_->setCachedPosition(position);
  }
}

bool CurrentsFromFieldDipoleGradient3AnalyticalCostFunction::Evaluate(
    double const* const* parameters, double* residuals, double** jacobians) const {
  Eigen::Map<const CurrentsVec> currents(parameters[0], num_coils_);

  // Not sure if it's better to let the model fail or return false telling
  // Ceres the residual computation failed
  FieldGradient5Vec bg_pred;
  FieldVec field_pred;
  Gradient3Vec grad_pred;
  Grad35Mat dg_mat = directedGradient3Mat(dipole_);
  try {
    FieldGradient5Vec bg_pred;
    if (use_cached_position_) {
      bg_pred = p_forward_model_->computeFieldGradient5FromCurrentsCached(currents);
    } else {
      bg_pred = p_forward_model_->computeFieldGradient5FromCurrents(position_, currents);
    }
    field_pred = bg_pred.head<3>();
    grad_pred = dg_mat * bg_pred.tail(5);
  } catch (std::exception& e) {
    return false;
  }

  residuals[0] = field_(0) - field_pred(0);
  residuals[1] = field_(1) - field_pred(1);
  residuals[2] = field_(2) - field_pred(2);
  residuals[3] = gradient_(0) - grad_pred(0);
  residuals[4] = gradient_(1) - grad_pred(1);
  residuals[5] = gradient_(2) - grad_pred(2);

  // only one parameter block of size Ne for the currents
  if (jacobians != nullptr && jacobians[0] != nullptr) {
    // here we also catch exceptions as failures in the single evaluation that don't necessarily
    // interrupt the whole computation
    try {
      // the currents jacobian contains the field and gradient jacobians. we only need the top 3
      // rows for the field
      // the bottom 5 rows are for the gradient
      CurrentsJacobian currents_jac;
      if (use_cached_position_) {
        currents_jac = p_jac_func_->cached(currents);
      } else {
        currents_jac = (*p_jac_func_)(position_, currents);
      }
      // this converts the gradient 5 Jacobian to a gradient3 Jacobian
      Eigen::Matrix<double, 3, 8> grad_jac = dg_mat * currents_jac.bottomRows(5);
      // rows are the entries of the field
      for (int i = 0; i < 3; i++) {
        // cols are entries of the current
        for (int k = 0; k < num_coils_; k++) {
          // the residual is bd - f(i) so the jacobian is
          // -df(i)/di
          jacobians[0][i * num_coils_ + k] = -currents_jac(i, k);
        }
      }
      for (int i = 3; i < 6; i++) {
        // cols are entries of the current
        for (int k = 0; k < num_coils_; k++) {
          jacobians[0][i * num_coils_ + k] = -grad_jac(i - 3, k);
        }
      }
    } catch (std::exception& e) {
      return false;
    }
  }

  return true;
}

CurrentsFromFieldGradient5CostFunctor::CurrentsFromFieldGradient5CostFunctor(
    ForwardModel::Ptr p_forward_model, const PositionVec& position, const FieldVec& field,
    const Gradient5Vec& gradient, const bool use_cached_position)
    : p_model_(p_forward_model), use_cached_position_(use_cached_position), position_(position) {
  field_gradient_.head(3) = field;
  field_gradient_.tail(5) = gradient;
  num_coils_ = p_model_->getNumCoils();
  if (use_cached_position_) p_model_->setCachedPosition(position);
}

bool CurrentsFromFieldGradient5CostFunctor::operator()(double const* const* parameters,
                                                       double* residuals) const {
  Eigen::Map<const CurrentsVec> currents(parameters[0], num_coils_);

  // Not sure if it's better to let the model fail or return false telling
  // Ceres the residual computation failed
  FieldGradient5Vec bg_pred;
  try {
    if (use_cached_position_) {
      bg_pred = p_model_->computeFieldGradient5FromCurrentsCached(currents);
    } else {
      bg_pred = p_model_->computeFieldGradient5FromCurrents(position_, currents);
    }
  } catch (std::exception& e) {
    return false;
  }

  for (int i = 0; i < bg_pred.size(); i++) {
    residuals[i] = field_gradient_(i) - bg_pred(i);
  }

  return true;
}

CurrentsFromFieldDipoleGradient3CostFunctor::CurrentsFromFieldDipoleGradient3CostFunctor(
    ForwardModel::Ptr p_model, const PositionVec& position, const FieldVec& field,
    const DipoleVec& dipole, const Gradient3Vec& gradient, const bool use_cached_position)
    : p_model_(p_model),
      use_cached_position_(use_cached_position),
      position_(position),
      field_(field),
      dipole_(dipole),
      gradient_(gradient) {
  num_coils_ = p_model_->getNumCoils();
  if (use_cached_position_) p_model->setCachedPosition(position);
}

bool CurrentsFromFieldDipoleGradient3CostFunctor::operator()(double const* const* parameters,
                                                             double* residuals) const {
  Eigen::Map<const CurrentsVec> currents(parameters[0], num_coils_);

  // Not sure if it's better to let the model fail or return false telling
  // Ceres the residual computation failed
  FieldVec field_pred;
  Gradient3Vec gradient_pred;
  try {
    FieldGradient5Vec bg_pred;
    if (use_cached_position_) {
      bg_pred = p_model_->computeFieldGradient5FromCurrentsCached(currents);
    } else {
      bg_pred = p_model_->computeFieldGradient5FromCurrents(position_, currents);
    }
    field_pred = bg_pred.head<3>();
    gradient_pred = directedGradient3Mat(dipole_) * bg_pred.tail(5);
  } catch (std::exception& e) {
    return false;
  }

  for (int i = 0; i < 3; i++) {
    residuals[i] = field_(i) - field_pred(i);
  }

  for (int i = 0; i < 3; i++) {
    residuals[i + 3] = gradient_(i) - gradient_pred(i);
  }

  return true;
}
