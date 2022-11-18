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

#include "mag_manip/backward_model_nls.h"

#include <ceres/ceres.h>
#include <yaml-cpp/yaml.h>

#include <exception>
#include <iostream>
#include <stdexcept>

#include "mag_manip/backward_model_nls_functors.h"
#include "mag_manip/currents_jacobian_functor_factory.h"
#include "mag_manip/exceptions.h"
#include "mag_manip/forward_model_factory.h"
#include "mag_manip/helpers.h"
#include "mag_manip/utils.h"

using namespace mag_manip;
using namespace std;

BackwardModelNLS::BackwardModelNLS()
    : do_print_summary_(false),
      do_minimize_currents_(false),
      weight_currents_(1e-8),
      use_analytical_gradients_(false),
      do_cache_position_(false),
      do_throw_exceptions_(false),
      max_field_dist_(1e-6),
      max_gradient_dist_(1e-3),
      do_log_iterations_to_console_(false) {
  // On Ubuntu 18.04 the DENSE_QR solver does not work for some reason.
  // It seems like it can't handle the conditioning for fields that are
  // aligned in a certain direction
  // switching to a sparse solver instead
  ceres_options_.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
}

void BackwardModelNLS::setCalibrationFile(const std::string& filename) {
  YAML::Node config;

  try {
    config = YAML::LoadFile(filename);
  } catch (YAML::BadFile& e) {
    throw InvalidFile(filename, e.what());
  }

  string parent_dir = getFileDirectory(filename);

  if (parent_dir.empty()) {
    throw std::runtime_error("filename has empty parent directory");
  }

  try {
    cal_name_ = config["name"].as<std::string>();
  } catch (YAML::Exception& e) {
    throw InvalidCalibration("Unable to set name. Reason: " + string(e.what()));
  }

  string cal_type;
  try {
    cal_type = config["type"].as<std::string>();
  } catch (YAML::Exception& e) {
    throw InvalidCalibration("Unable to set calibration type. Because: " + string(e.what()));
  }

  if (cal_type != "backward_model_nls") {
    throw InvalidCalibration("Invalid calibration type. Should be backward_model_nls");
  }

  YAML::Node n_for_model = config["forward_model"];

  if (!n_for_model.IsMap()) {
    cout << "foo" << endl;
    throw InvalidCalibration("Invalid map forward_model");
  }

  string for_model_type;

  try {
    for_model_type = n_for_model["type"].as<string>();
  } catch (YAML::Exception& e) {
    throw InvalidCalibration("Unable to set forward model type.");
  }

  string for_model_fn;

  try {
    for_model_fn = n_for_model["filename"].as<string>();
  } catch (YAML::Exception& e) {
    throw InvalidCalibration("Unable to set forward model filename.");
  }

  auto fact = ForwardModelFactory();
  // also sets num_coils_
  setForwardModel(fact.create(for_model_type, pathAppend(parent_dir, for_model_fn)));

  // parse remaining of parameters for solver
  YAML::Node n_nls = config["nls"];
  if (!n_nls.IsMap()) {
    throw InvalidCalibration("Invalid map nls");
  }

  if (n_nls["do_print_summary"].IsDefined()) {
    try {
      do_print_summary_ = n_nls["do_print_summary"].as<bool>();
    } catch (YAML::Exception& e) {
      throw InvalidCalibration("Unable to set do_print_summary because " + string(e.what()));
    }
  }

  if (n_nls["do_minimize_currents"].IsDefined()) {
    try {
      do_minimize_currents_ = n_nls["do_minimize_currents"].as<bool>();
    } catch (YAML::Exception& e) {
      throw InvalidCalibration("Unable to set do_minimize_currents because " + string(e.what()));
    }
  }

  if (n_nls["weight_currents"].IsDefined()) {
    try {
      weight_currents_ = n_nls["weight_currents"].as<double>();
    } catch (YAML::Exception& e) {
      throw InvalidCalibration("Unable to set weight_currents because " + string(e.what()));
    }
  }

  if (n_nls["min_currents"].IsDefined()) {
    std::vector<double> v_min_currents;
    try {
      v_min_currents = n_nls["min_currents"].as<std::vector<double> >();
    } catch (YAML::Exception& e) {
      throw InvalidCalibration("Unable to set min_currents because " + string(e.what()));
    }

    if (v_min_currents.size() != num_coils_) {
      throw InvalidCurrentsLength();
    }

    min_currents_ =
        Eigen::Map<CurrentsVec, Eigen::Unaligned>(v_min_currents.data(), v_min_currents.size());
  }

  if (n_nls["max_currents"].IsDefined()) {
    std::vector<double> v_max_currents;
    try {
      v_max_currents = n_nls["max_currents"].as<std::vector<double> >();
    } catch (YAML::Exception& e) {
      throw InvalidCalibration("Unable to set max_currents because " + string(e.what()));
    }

    if (v_max_currents.size() != num_coils_) {
      throw InvalidCurrentsLength();
    }

    max_currents_ =
        Eigen::Map<CurrentsVec, Eigen::Unaligned>(v_max_currents.data(), v_max_currents.size());
  }

  if (n_nls["max_field_dist"].IsDefined()) {
    try {
      max_field_dist_ = n_nls["max_field_dist"].as<double>();
    } catch (YAML::Exception& e) {
      throw InvalidCalibration("Unable to set max_field_dist because " + string(e.what()));
    }
  }

  if (n_nls["max_gradient_dist"].IsDefined()) {
    try {
      max_gradient_dist_ = n_nls["max_gradient_dist"].as<double>();
    } catch (YAML::Exception& e) {
      throw InvalidCalibration("Unable to set max_gradient_dist because " + string(e.what()));
    }
  }

  if (n_nls["do_cache_position"].IsDefined()) {
    try {
      do_cache_position_ = n_nls["do_cache_position"].as<bool>();
    } catch (YAML::Exception& e) {
      throw InvalidCalibration("Unable to set do_cache_positon because " + string(e.what()));
    }
  }

  if (n_nls["do_log_iterations_to_console"].IsDefined()) {
    try {
      do_log_iterations_to_console_ = n_nls["do_log_iterations_to_console"].as<bool>();
    } catch (YAML::Exception& e) {
      throw InvalidCalibration("Unable to set do_log_iterations_to_console because " +
                               string(e.what()));
    }
  }

  if (n_nls["do_throw_exceptions"].IsDefined()) {
    try {
      do_throw_exceptions_ = n_nls["do_throw_exceptions"].as<bool>();
    } catch (YAML::Exception& e) {
      throw InvalidCalibration("Unable to set do_throw_exceptions because " + string(e.what()));
    }
  }

  YAML::Node n_jacs = n_nls["analytical_gradients"];
  string analytical_gradients_type;
  string analytical_gradients_fn;
  if (n_jacs.IsDefined()) {
    if (!n_jacs.IsMap()) {
      throw InvalidCalibration("analytical_gradients should be a map");
    }

    try {
      analytical_gradients_type = n_jacs["type"].as<string>();
    } catch (YAML::Exception& e) {
      throw InvalidCalibration("Unable to set analytical gradients type because " +
                               string(e.what()));
    }

    try {
      analytical_gradients_fn = n_jacs["filename"].as<string>();
    } catch (YAML::Exception& e) {
      throw InvalidCalibration("Unable to set analytical gradients filename because " +
                               string(e.what()));
    }

    CurrentsJacobianFunctor::Ptr p_jac;
    try {
      p_jac = CurrentsJacobianFunctorFactory::create(
          analytical_gradients_type, pathAppend(parent_dir, analytical_gradients_fn));
    } catch (std::invalid_argument& e) {
      throw InvalidCalibration("Invalid analytical_gradients_type");
    }

    p_currents_jac_func_ = p_jac;
    // we don't need to set the cached position of the currents jacobian
    // since it will be done in the compute function

    use_analytical_gradients_ = true;

  } else {
    use_analytical_gradients_ = false;
  }
}

void BackwardModelNLS::setForwardModelCalibrationFile(const std::string& filename) {
  if (p_forward_model_) {
    p_forward_model_->setCalibrationFile(filename);
    num_coils_ = p_forward_model_->getNumCoils();
    initial_currents_ = CurrentsVec::Zero(num_coils_);
  } else {
    throw CalibrationNotLoaded();
  }
}

bool BackwardModelNLS::isValid() const {
  if (p_forward_model_)
    return p_forward_model_->isValid();
  else
    return false;
}

int BackwardModelNLS::getNumCoils() const {
  if (p_forward_model_) {
    return num_coils_;
  } else {
    throw CalibrationNotLoaded();
  }
}

void BackwardModelNLS::setForwardModel(ForwardModel::Ptr p_forward_model) {
  p_forward_model_ = p_forward_model;
  num_coils_ = p_forward_model_->getNumCoils();
  initial_currents_ = CurrentsVec::Zero(num_coils_);
}

const ForwardModel& BackwardModelNLS::getForwardModel() const { return *p_forward_model_.get(); }

void BackwardModelNLS::setInitialCurrents(const CurrentsVec& currents) {
  if (initial_currents_.size() != num_coils_) {
    throw InvalidCurrentsLength();
  }

  initial_currents_ = currents;
}

void BackwardModelNLS::setSolverOptions(const ceres::Solver::Options& options) {
  ceres_options_ = options;
}

ceres::Solver::Options BackwardModelNLS::getSolverOptions() const { return ceres_options_; }

void BackwardModelNLS::setDoPrintSummary(const bool enable) { do_print_summary_ = enable; }

bool BackwardModelNLS::getDoPrintSummary() const { return do_print_summary_; }

void BackwardModelNLS::setDoMinimizeCurrents(const bool enable) { do_minimize_currents_ = enable; }

bool BackwardModelNLS::getDoMinimizeCurrents() const { return do_minimize_currents_; }

void BackwardModelNLS::setWeightCurrents(const double lambda) { weight_currents_ = lambda; }

double BackwardModelNLS::getWeightCurrents() const { return weight_currents_; }

void BackwardModelNLS::setDoCachePosition(const bool enable) { do_cache_position_ = enable; }

bool BackwardModelNLS::getDoCachePosition() const { return do_cache_position_; }

void BackwardModelNLS::setDoLogIterationsToConsole(const bool enable) {
  do_log_iterations_to_console_ = enable;
}

bool BackwardModelNLS::getDoLogIterationsToConsole() const { return do_log_iterations_to_console_; }

void BackwardModelNLS::setDoThroughExceptions(const bool enable) { do_throw_exceptions_ = enable; }

bool BackwardModelNLS::getDoThroughExceptions() const { return do_throw_exceptions_; }

CurrentsVec BackwardModelNLS::computeCurrentsFromField(const PositionVec& position,
                                                       const FieldVec& field) const {
  const auto ret_vals = computeCurrentsFromFieldRet(position, field);
  return std::get<1>(ret_vals);
}

void BackwardModelNLS::setCurrentsJacobianFunctor(CurrentsJacobianFunctor::Ptr p_jac) {
  p_currents_jac_func_ = p_jac;
  // should probably ensure that the forward model is the same here
}

void BackwardModelNLS::setUseAnalyticalGradients(const bool enable) {
  use_analytical_gradients_ = enable;
}

bool BackwardModelNLS::getUseAnalyticalGradients() const { return use_analytical_gradients_; }

std::pair<bool, CurrentsVec> BackwardModelNLS::computeCurrentsFromFieldRet(
    const PositionVec& position, const FieldVec& field) const {
  if (!p_forward_model_) {
    throw CalibrationNotLoaded();
  }

  ceres::Problem problem;

  auto* currents_init = new double[num_coils_];
  for (int i = 0; i < num_coils_; i++) {
    currents_init[i] = initial_currents_(i);
  }

  ceres::CostFunction* p_cost_function;
  if (use_analytical_gradients_) {
    if (!p_currents_jac_func_) {
      throw std::runtime_error(
          "Currents Jacobian functor not set. setCurrentsJacobianFunctor if using analytical "
          "gradients.");
    }

    // this ensures that the cached position in the Jacobian functor
    // matches the desired position
    if (do_cache_position_) p_currents_jac_func_->setCachedPosition(position);

    p_cost_function = new CurrentsFromFieldAnalyticalCostFunction(
        p_forward_model_, position, field, p_currents_jac_func_, do_cache_position_);

  } else {
    auto p_dyn_diff_cost_function =
        new ceres::DynamicNumericDiffCostFunction<CurrentsFromFieldCostFunctor>(
            new CurrentsFromFieldCostFunctor(p_forward_model_, position, field,
                                             do_cache_position_));
    p_dyn_diff_cost_function->AddParameterBlock(num_coils_);
    p_dyn_diff_cost_function->SetNumResiduals(3);
    p_cost_function = p_dyn_diff_cost_function;
  }

  problem.AddParameterBlock(currents_init, num_coils_);
  problem.AddResidualBlock(p_cost_function, nullptr, currents_init);

  if (do_minimize_currents_) {
    auto p_regularizer = new ceres::DynamicAutoDiffCostFunction<CurrentsRegularizationFunctor>(
        new CurrentsRegularizationFunctor(num_coils_, weight_currents_));
    p_regularizer->SetNumResiduals(num_coils_);
    p_regularizer->AddParameterBlock(num_coils_);
    problem.AddResidualBlock(p_regularizer, nullptr, currents_init);
  }

  if (min_currents_.size() > 0) {
    for (int i = 0; i < num_coils_; i++) {
      problem.SetParameterLowerBound(currents_init, i, min_currents_(i));
    }
  }

  if (max_currents_.size() > 0) {
    for (int i = 0; i < num_coils_; i++) {
      problem.SetParameterUpperBound(currents_init, i, max_currents_(i));
    }
  }

  ceres::Solver solver;
  ceres::Solver::Summary summary;

  // copying over the settings to they can be overridden
  // with additional settings from members
  ceres::Solver::Options ceres_options = ceres_options_;

  if (do_log_iterations_to_console_) {
    // ceres makes you specify which iterations to log
    // we want to log all, so we just create a vector with
    // 0, 1, .. max_iterations-1
    std::vector<int> iterations(ceres_options_.max_num_iterations);
    std::iota(iterations.begin(), iterations.end(), 0);
    ceres_options.trust_region_minimizer_iterations_to_dump = iterations;
    ceres_options.trust_region_problem_dump_format_type = ceres::CONSOLE;
  } else {
    ceres_options.trust_region_minimizer_iterations_to_dump = {};
  }

  ceres::Solve(ceres_options, &problem, &summary);

  if (do_print_summary_) {
    std::cout << summary.FullReport() << std::endl;
  }

  CurrentsVec currents;
  currents.setZero(num_coils_);
  for (int i = 0; i < num_coils_; i++) {
    currents(i) = currents_init[i];
  }

  auto field_final = p_forward_model_->computeFieldFromCurrents(position, currents);

  bool is_field_valid = (field_final - field).norm() < max_field_dist_;

  if (do_throw_exceptions_) {
    if (!summary.IsSolutionUsable()) {
      std::stringstream ss;
      ss << "The BackwardNLS solver failed to compute a solution." << endl;
      ss << summary.BriefReport() << endl;
      throw FailedComputeCurrentsException(ss.str());
    }

    if (!is_field_valid) {
      throw FailedComputeCurrentsException(
          "The distance between the predicted field and the desired field exceeds the maximum "
          "field distance parameter.");
    }
  }

  // IsSolutionUsuable may return true if the convergence is reached because
  // the solution did not change between steps but the field may still not be reached
  return std::make_pair(summary.IsSolutionUsable() && is_field_valid, currents);
}

CurrentsVec BackwardModelNLS::computeCurrentsFromFieldGradient5(
    const PositionVec& position, const FieldVec& field, const Gradient5Vec& gradient) const {
  auto ret_pair = computeCurrentsFromFieldGradient5Ret(position, field, gradient);
  return std::get<1>(ret_pair);
}

std::pair<bool, CurrentsVec> BackwardModelNLS::computeCurrentsFromFieldGradient5Ret(
    const PositionVec& position, const FieldVec& field, const Gradient5Vec& gradient) const {
  if (!p_forward_model_) {
    throw CalibrationNotLoaded();
  }

  auto* currents_init = new double[num_coils_];
  for (int i = 0; i < num_coils_; i++) {
    currents_init[i] = initial_currents_(i);
  }

  ceres::Problem problem;

  ceres::CostFunction* p_cost_function;
  if (use_analytical_gradients_) {
    if (!p_currents_jac_func_) {
      throw std::runtime_error(
          "Currents Jacobian functor not set. setCurrentsJacobianFunctor if using analytical "
          "gradients.");
    }

    // this ensures that the cached position in the Jacobian functor
    // matches the desired position
    if (do_cache_position_) p_currents_jac_func_->setCachedPosition(position);

    p_cost_function = new CurrentsFromFieldGradient5AnalyticalCostFunction(
        p_forward_model_, position, field, gradient, p_currents_jac_func_, do_cache_position_);

  } else {
    auto p_dyn_diff_cost_function =
        new ceres::DynamicNumericDiffCostFunction<CurrentsFromFieldGradient5CostFunctor>(
            new CurrentsFromFieldGradient5CostFunctor(p_forward_model_, position, field, gradient,
                                                      do_cache_position_));
    p_dyn_diff_cost_function->AddParameterBlock(num_coils_);
    p_dyn_diff_cost_function->SetNumResiduals(8);
    p_cost_function = p_dyn_diff_cost_function;
  }

  problem.AddParameterBlock(currents_init, num_coils_);
  problem.AddResidualBlock(p_cost_function, nullptr, currents_init);

  if (do_minimize_currents_) {
    auto p_regularizer = new ceres::DynamicAutoDiffCostFunction<CurrentsRegularizationFunctor>(
        new CurrentsRegularizationFunctor(num_coils_, weight_currents_));
    p_regularizer->SetNumResiduals(num_coils_);
    p_regularizer->AddParameterBlock(num_coils_);
    problem.AddResidualBlock(p_regularizer, nullptr, currents_init);
  }

  if (min_currents_.size() > 0) {
    for (int i = 0; i < num_coils_; i++) {
      problem.SetParameterLowerBound(currents_init, i, min_currents_(i));
    }
  }

  if (max_currents_.size() > 0) {
    for (int i = 0; i < num_coils_; i++) {
      problem.SetParameterUpperBound(currents_init, i, max_currents_(i));
    }
  }

  ceres::Solver solver;
  ceres::Solver::Summary summary;

  // copying over the settings to they can be overridden
  // with additional settings from members
  ceres::Solver::Options ceres_options = ceres_options_;

  if (do_log_iterations_to_console_) {
    // ceres makes you specify which iterations to log
    // we want to log all, so we just create a vector with
    // 0, 1, .. max_iterations-1
    std::vector<int> iterations(ceres_options_.max_num_iterations);
    std::iota(iterations.begin(), iterations.end(), 0);
    ceres_options.trust_region_minimizer_iterations_to_dump = iterations;
    ceres_options.trust_region_problem_dump_format_type = ceres::CONSOLE;
  } else {
    ceres_options.trust_region_minimizer_iterations_to_dump = {};
  }

  ceres::Solve(ceres_options, &problem, &summary);

  if (do_print_summary_) {
    std::cout << summary.FullReport() << std::endl;
  }

  CurrentsVec currents;
  currents.setZero(num_coils_);
  for (int i = 0; i < num_coils_; i++) {
    currents(i) = currents_init[i];
  }

  auto field_gradient_final =
      p_forward_model_->computeFieldGradient5FromCurrents(position, currents);
  FieldVec field_final = field_gradient_final.head(3);
  Gradient5Vec gradient_final = field_gradient_final.tail(5);
  bool is_field_valid = (field_final - field).norm() < max_field_dist_;

  bool is_gradient_valid = (gradient_final - gradient).norm() < max_gradient_dist_;

  if (do_throw_exceptions_) {
    if (!summary.IsSolutionUsable()) {
      throw FailedComputeCurrentsException(
          "The BackwardNLS solver failed to compute a solution. Print to solver summary for more "
          "information.");
    }

    if (!is_field_valid) {
      throw FailedComputeCurrentsException(
          "The distance between the predicted field and the desired field exceeds the maximum "
          "field distance parameter.");
    }

    if (!is_gradient_valid) {
      throw FailedComputeCurrentsException(
          "The distance between the predicted gradient and the desired gradient exceeds the "
          "maximum gradient distance parameter.");
    }
  }

  // IsSolutionUsuable may return true if the convergence is reached because
  // the solution did not change between steps but the field may still not be reached
  return std::make_pair(summary.IsSolutionUsable() && is_field_valid && is_gradient_valid,
                        currents);
}

CurrentsVec BackwardModelNLS::computeCurrentsFromFieldDipoleGradient3(
    const PositionVec& position, const FieldVec& field, const DipoleVec& dipole,
    const Gradient3Vec& gradient) const {
  auto ret_pair = computeCurrentsFromFieldDipoleGradient3Ret(position, field, dipole, gradient);
  return std::get<1>(ret_pair);
}

std::pair<bool, CurrentsVec> BackwardModelNLS::computeCurrentsFromFieldDipoleGradient3Ret(
    const PositionVec& position, const FieldVec& field, const DipoleVec& dipole,
    const Gradient3Vec& gradient) const {
  if (!p_forward_model_) {
    throw CalibrationNotLoaded();
  }

  auto* currents_init = new double[num_coils_];
  for (int i = 0; i < num_coils_; i++) {
    currents_init[i] = initial_currents_(i);
  }

  ceres::Problem problem;

  ceres::CostFunction* p_cost_function;
  if (use_analytical_gradients_) {
    if (!p_currents_jac_func_) {
      throw std::runtime_error(
          "Currents Jacobian functor not set. setCurrentsJacobianFunctor if using analytical "
          "gradients.");
    }

    // this ensures that the cached position in the Jacobian functor
    // matches the desired position
    if (do_cache_position_) p_currents_jac_func_->setCachedPosition(position);

    p_cost_function = new CurrentsFromFieldDipoleGradient3AnalyticalCostFunction(
        p_forward_model_, position, field, dipole, gradient, p_currents_jac_func_,
        do_cache_position_);

  } else {
    auto p_dyn_diff_cost_function =
        new ceres::DynamicNumericDiffCostFunction<CurrentsFromFieldDipoleGradient3CostFunctor>(
            new CurrentsFromFieldDipoleGradient3CostFunctor(p_forward_model_, position, field,
                                                            dipole, gradient, do_cache_position_));
    p_dyn_diff_cost_function->AddParameterBlock(num_coils_);
    p_dyn_diff_cost_function->SetNumResiduals(6);
    p_cost_function = p_dyn_diff_cost_function;
  }

  problem.AddParameterBlock(currents_init, num_coils_);
  problem.AddResidualBlock(p_cost_function, nullptr, currents_init);

  if (do_minimize_currents_) {
    auto p_regularizer = new ceres::DynamicAutoDiffCostFunction<CurrentsRegularizationFunctor>(
        new CurrentsRegularizationFunctor(num_coils_, weight_currents_));
    p_regularizer->SetNumResiduals(num_coils_);
    p_regularizer->AddParameterBlock(num_coils_);
    problem.AddResidualBlock(p_regularizer, nullptr, currents_init);
  }

  if (min_currents_.size() > 0) {
    for (int i = 0; i < num_coils_; i++) {
      problem.SetParameterLowerBound(currents_init, i, min_currents_(i));
    }
  }

  if (max_currents_.size() > 0) {
    for (int i = 0; i < num_coils_; i++) {
      problem.SetParameterUpperBound(currents_init, i, max_currents_(i));
    }
  }

  ceres::Solver solver;
  ceres::Solver::Summary summary;

  // copying over the settings to they can be overridden
  // with additional settings from members
  ceres::Solver::Options ceres_options = ceres_options_;

  if (do_log_iterations_to_console_) {
    // ceres makes you specify which iterations to log
    // we want to log all, so we just create a vector with
    // 0, 1, .. max_iterations-1
    std::vector<int> iterations(ceres_options_.max_num_iterations);
    std::iota(iterations.begin(), iterations.end(), 0);
    ceres_options.trust_region_minimizer_iterations_to_dump = iterations;
    ceres_options.trust_region_problem_dump_format_type = ceres::CONSOLE;
  } else {
    ceres_options.trust_region_minimizer_iterations_to_dump = {};
  }

  ceres::Solve(ceres_options, &problem, &summary);

  if (do_print_summary_) {
    std::cout << summary.FullReport() << std::endl;
  }

  CurrentsVec currents;
  currents.setZero(num_coils_);
  for (int i = 0; i < num_coils_; i++) {
    currents(i) = currents_init[i];
  }

  auto field_gradient_final =
      p_forward_model_->computeFieldGradient5FromCurrents(position, currents);
  FieldVec field_final = field_gradient_final.head(3);
  Gradient5Vec gradient5 = field_gradient_final.tail(5);
  Gradient3Vec gradient_final = directedGradient3Mat(dipole) * gradient5;
  bool is_field_valid = (field_final - field).norm() < max_field_dist_;

  bool is_gradient_valid = (gradient_final - gradient).norm() < max_gradient_dist_;

  if (do_throw_exceptions_) {
    if (!summary.IsSolutionUsable()) {
      throw FailedComputeCurrentsException(
          "The BackwardNLS solver failed to compute a solution. Print to solver summary for more "
          "information.");
    }

    if (!is_field_valid) {
      throw FailedComputeCurrentsException(
          "The distance between the predicted field and the desired field exceeds the maximum "
          "field distance parameter.");
    }

    if (!is_gradient_valid) {
      throw FailedComputeCurrentsException(
          "The distance between the predicted gradient and the desired gradient exceeds the "
          "maximum gradient distance parameter.");
    }
  }

  // IsSolutionUsuable may return true if the convergence is reached because
  // the solution did not change between steps but the field may still not be reached
  return std::make_pair(summary.IsSolutionUsable() && is_field_valid && is_gradient_valid,
                        currents);
}

void BackwardModelNLS::setMaxCurrents(const CurrentsVec& currents) {
  if (currents.size() != num_coils_) {
    throw InvalidCurrentsLength();
  }

  max_currents_ = currents;
}

CurrentsVec BackwardModelNLS::getMaxCurrents() const { return max_currents_; }

void BackwardModelNLS::setMinCurrents(const CurrentsVec& currents) {
  if (currents.size() != num_coils_) {
    throw InvalidCurrentsLength();
  }

  min_currents_ = currents;
}

CurrentsVec BackwardModelNLS::getMinCurrents() const { return min_currents_; }

void BackwardModelNLS::setMaximumFieldDistance(const double dist) {
  if (dist < 0) {
    throw std::invalid_argument("The maximum field distance cannot be negative");
  }

  max_field_dist_ = dist;
}

double BackwardModelNLS::getMaximumFieldDistance() const { return max_field_dist_; }

void BackwardModelNLS::setMaximumGradientDistance(const double dist) {
  if (dist < 0) {
    throw std::invalid_argument("The maximum gradient distance cannot be negative");
  }

  max_gradient_dist_ = dist;
}

double BackwardModelNLS::getMaximumGradientDistance() const { return max_gradient_dist_; }
