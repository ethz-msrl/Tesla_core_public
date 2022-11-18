/* * Tesla - A ROS-based framework for performing magnetic manipulation
 *
 * Copyright 2018 Multi Scale Robotics Lab
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <benchmark/benchmark.h>
#include <ros/package.h>

#include "mag_manip/backward_model_nls.h"
#include "mag_manip/exceptions.h"
#include "mag_manip/forward_model_linear_saturation.h"
#include "mag_manip/forward_model_linear_saturation_currents_jacobian_functor.h"
#include "mag_manip/forward_model_mpem.h"
#include "mag_manip/helpers.h"

#include <ceres/solver.h>

using namespace std;
using namespace mag_manip;

static void bm_computeCurrentsFromField_DENSE_QR(benchmark::State& state) {
  ForwardModel::UPtr p_forward_model(new ForwardModelMPEM);
  string cal_file = ros::package::getPath("mag_manip") + "/test/OctoMag_Calibration.yaml";
  p_forward_model->setCalibrationFile(cal_file);
  BackwardModelNLS backward_model;
  backward_model.setForwardModel(std::move(p_forward_model));
  ceres::Solver::Options ceres_options;
  ceres_options.linear_solver_type = ceres::DENSE_QR;
  backward_model.setSolverOptions(ceres_options);

  FieldVec field(20e-3, 0, 0);
  PositionVec position(0, 0, 0);
  CurrentsVec init_currents(backward_model.getNumCoils());
  init_currents << 1, 1, 1, 1, 1, 1, 1, 1;
  backward_model.setInitialCurrents(init_currents);

  while (state.KeepRunning()) {
    CurrentsVec currents = backward_model.computeCurrentsFromField(position, field);
  }
}

BENCHMARK(bm_computeCurrentsFromField_DENSE_QR);

static void bm_computeCurrentsFromField_SPARSE_NORMAL_CHOLESKY(benchmark::State& state) {
  ForwardModel::UPtr p_forward_model(new ForwardModelMPEM);
  string cal_file = ros::package::getPath("mag_manip") + "/test/OctoMag_Calibration.yaml";
  p_forward_model->setCalibrationFile(cal_file);
  BackwardModelNLS backward_model;
  backward_model.setForwardModel(std::move(p_forward_model));
  ceres::Solver::Options ceres_options;
  ceres_options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  backward_model.setSolverOptions(ceres_options);

  FieldVec field(20e-3, 0, 0);
  PositionVec position(0, 0, 0);
  CurrentsVec init_currents(backward_model.getNumCoils());
  init_currents << 1, 1, 1, 1, 1, 1, 1, 1;
  backward_model.setInitialCurrents(init_currents);

  while (state.KeepRunning()) {
    CurrentsVec currents = backward_model.computeCurrentsFromField(position, field);
  }
}

BENCHMARK(bm_computeCurrentsFromField_SPARSE_NORMAL_CHOLESKY);

static void bm_computeCurrentsFromFieldMinimizeCurrents_DENSE_QR(benchmark::State& state) {
  ForwardModel::UPtr p_forward_model(new ForwardModelMPEM);
  string cal_file = ros::package::getPath("mag_manip") + "/test/OctoMag_Calibration.yaml";
  p_forward_model->setCalibrationFile(cal_file);
  BackwardModelNLS backward_model;
  backward_model.setForwardModel(std::move(p_forward_model));
  ceres::Solver::Options ceres_options;
  ceres_options.linear_solver_type = ceres::DENSE_QR;
  backward_model.setSolverOptions(ceres_options);
  FieldVec field(20e-3, 0, 0);
  PositionVec position(0, 0, 0);
  CurrentsVec init_currents(backward_model.getNumCoils());
  init_currents << 1, 1, 1, 1, 1, 1, 1, 1;
  backward_model.setInitialCurrents(init_currents);
  backward_model.setDoMinimizeCurrents(true);
  backward_model.setWeightCurrents(1e-8);

  while (state.KeepRunning()) {
    CurrentsVec currents = backward_model.computeCurrentsFromField(position, field);
  }
}

BENCHMARK(bm_computeCurrentsFromFieldMinimizeCurrents_DENSE_QR);

static void bm_computeCurrentsFromFieldMinimizeCurrents_SPARSE_NORMAL_CHOLESKY(
    benchmark::State& state) {
  ForwardModel::UPtr p_forward_model(new ForwardModelMPEM);
  string cal_file = ros::package::getPath("mag_manip") + "/test/OctoMag_Calibration.yaml";
  p_forward_model->setCalibrationFile(cal_file);
  BackwardModelNLS backward_model;
  backward_model.setForwardModel(std::move(p_forward_model));
  ceres::Solver::Options ceres_options;
  ceres_options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  backward_model.setSolverOptions(ceres_options);
  FieldVec field(20e-3, 0, 0);
  PositionVec position(0, 0, 0);
  CurrentsVec init_currents(backward_model.getNumCoils());
  init_currents << 1, 1, 1, 1, 1, 1, 1, 1;
  backward_model.setInitialCurrents(init_currents);
  backward_model.setDoMinimizeCurrents(true);
  backward_model.setWeightCurrents(1e-8);

  while (state.KeepRunning()) {
    CurrentsVec currents = backward_model.computeCurrentsFromField(position, field);
  }
}

BENCHMARK(bm_computeCurrentsFromFieldMinimizeCurrents_SPARSE_NORMAL_CHOLESKY);

static void bm_computeCurrentsFromFieldGradient5_DENSE_QR(benchmark::State& state) {
  ForwardModel::UPtr p_forward_model(new ForwardModelMPEM);
  string cal_file = ros::package::getPath("mag_manip") + "/test/OctoMag_Calibration.yaml";
  p_forward_model->setCalibrationFile(cal_file);
  BackwardModelNLS backward_model;
  backward_model.setForwardModel(std::move(p_forward_model));
  ceres::Solver::Options ceres_options;
  ceres_options.linear_solver_type = ceres::DENSE_QR;
  backward_model.setSolverOptions(ceres_options);
  FieldVec field(20e-3, 0, 0);
  Gradient5Vec gradient;
  gradient << 0.4, 0, 0, 0, 0;
  PositionVec position(0, 0, 0);
  while (state.KeepRunning()) {
    CurrentsVec currents =
        backward_model.computeCurrentsFromFieldGradient5(position, field, gradient);
  }
}

BENCHMARK(bm_computeCurrentsFromFieldGradient5_DENSE_QR);

static void bm_computeCurrentsFromFieldGradient5_SPARSE_NORMAL_CHOLESKY(benchmark::State& state) {
  ForwardModel::UPtr p_forward_model(new ForwardModelMPEM);
  string cal_file = ros::package::getPath("mag_manip") + "/test/OctoMag_Calibration.yaml";
  p_forward_model->setCalibrationFile(cal_file);
  BackwardModelNLS backward_model;
  backward_model.setForwardModel(std::move(p_forward_model));
  ceres::Solver::Options ceres_options;
  ceres_options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  backward_model.setSolverOptions(ceres_options);
  FieldVec field(20e-3, 0, 0);
  Gradient5Vec gradient;
  gradient << 0.4, 0, 0, 0, 0;
  PositionVec position(0, 0, 0);
  while (state.KeepRunning()) {
    CurrentsVec currents =
        backward_model.computeCurrentsFromFieldGradient5(position, field, gradient);
  }
}

BENCHMARK(bm_computeCurrentsFromFieldGradient5_SPARSE_NORMAL_CHOLESKY);

static void bm_computeCurrentsFromField_LINSAT(benchmark::State& state) {
  ForwardModelLinearSaturation::Ptr p_fmls(new ForwardModelLinearSaturation);
  string cal_file = ros::package::getPath("mag_manip") + "/models/cmag_fmls_v1/params.yaml";
  ForwardModelLinear::Ptr p_fml = make_shared<ForwardModelMPEM>();
  p_fml->setCalibrationFile(ros::package::getPath("mag_manip") +
                            "/models/cmag_fmls_v1/CardioMag_CalibrationCube_03-04-19.yaml");
  p_fmls->setLinearModel(p_fml);
  p_fmls->setSaturationFunctionsFile(ros::package::getPath("mag_manip") +
                                     "/models/cmag_fmls_v1/cmag_sat_erf_v1.yaml");

  BackwardModelNLS backward_model;
  backward_model.setForwardModel(p_fmls);
  backward_model.setUseAnalyticalGradients(false);

  FieldVec field(20e-3, 0, 0);
  PositionVec position(0, 0, 0);
  while (state.KeepRunning()) {
    CurrentsVec currents = backward_model.computeCurrentsFromField(position, field);
  }
}

BENCHMARK(bm_computeCurrentsFromField_LINSAT);

static void bm_computeCurrentsFromField_LINSAT_ANALYTICAL(benchmark::State& state) {
  ForwardModelLinearSaturation::Ptr p_fmls(new ForwardModelLinearSaturation);
  string cal_file = ros::package::getPath("mag_manip") + "/models/cmag_fmls_v1/params.yaml";
  ForwardModelLinear::Ptr p_fml = make_shared<ForwardModelMPEM>();
  p_fml->setCalibrationFile(ros::package::getPath("mag_manip") +
                            "/models/cmag_fmls_v1/CardioMag_CalibrationCube_03-04-19.yaml");
  p_fmls->setLinearModel(p_fml);
  p_fmls->setSaturationFunctionsFile(ros::package::getPath("mag_manip") +
                                     "/models/cmag_fmls_v1/cmag_sat_erf_v1.yaml");

  auto p_jac = make_shared<ForwardModelLinearSaturationCurrentsJacobianFunctor>();
  p_jac->setForwardModelLinearSaturation(p_fmls);

  BackwardModelNLS backward_model;
  backward_model.setForwardModel(p_fmls);
  backward_model.setCurrentsJacobianFunctor(p_jac);
  backward_model.setUseAnalyticalGradients(true);

  FieldVec field(20e-3, 0, 0);
  PositionVec position(0, 0, 0);
  while (state.KeepRunning()) {
    CurrentsVec currents = backward_model.computeCurrentsFromField(position, field);
  }
}

BENCHMARK(bm_computeCurrentsFromField_LINSAT_ANALYTICAL);

static void bm_computeCurrentsFromField_LINSAT_CACHED(benchmark::State& state) {
  ForwardModelLinearSaturation::Ptr p_fmls(new ForwardModelLinearSaturation);
  string cal_file = ros::package::getPath("mag_manip") + "/models/cmag_fmls_v1/params.yaml";
  ForwardModelLinear::Ptr p_fml = make_shared<ForwardModelMPEM>();
  p_fml->setCalibrationFile(ros::package::getPath("mag_manip") +
                            "/models/cmag_fmls_v1/CardioMag_CalibrationCube_03-04-19.yaml");
  p_fmls->setLinearModel(p_fml);
  p_fmls->setSaturationFunctionsFile(ros::package::getPath("mag_manip") +
                                     "/models/cmag_fmls_v1/cmag_sat_erf_v1.yaml");

  FieldVec field(20e-3, 0, 0);
  PositionVec position(0, 0, 0);

  BackwardModelNLS backward_model;
  backward_model.setForwardModel(p_fmls);
  backward_model.setUseAnalyticalGradients(false);
  backward_model.setDoCachePosition(true);

  while (state.KeepRunning()) {
    CurrentsVec currents = backward_model.computeCurrentsFromField(position, field);
  }
}

BENCHMARK(bm_computeCurrentsFromField_LINSAT_CACHED);

static void bm_computeCurrentsFromField_LINSAT_ANALYTICAL_CACHED(benchmark::State& state) {
  ForwardModelLinearSaturation::Ptr p_fmls(new ForwardModelLinearSaturation);
  string cal_file = ros::package::getPath("mag_manip") + "/models/cmag_fmls_v1/params.yaml";
  ForwardModelLinear::Ptr p_fml = make_shared<ForwardModelMPEM>();
  p_fml->setCalibrationFile(ros::package::getPath("mag_manip") +
                            "/models/cmag_fmls_v1/CardioMag_CalibrationCube_03-04-19.yaml");
  p_fmls->setLinearModel(p_fml);
  p_fmls->setSaturationFunctionsFile(ros::package::getPath("mag_manip") +
                                     "/models/cmag_fmls_v1/cmag_sat_erf_v1.yaml");

  FieldVec field(20e-3, 0, 0);
  PositionVec position(0, 0, 0);

  auto p_jac = make_shared<ForwardModelLinearSaturationCurrentsJacobianFunctor>();
  p_jac->setForwardModelLinearSaturation(p_fmls);
  p_jac->setCachedPosition(position);

  BackwardModelNLS backward_model;
  backward_model.setForwardModel(p_fmls);
  backward_model.setCurrentsJacobianFunctor(p_jac);
  backward_model.setUseAnalyticalGradients(true);
  backward_model.setDoCachePosition(true);

  while (state.KeepRunning()) {
    CurrentsVec currents = backward_model.computeCurrentsFromField(position, field);
  }
}

BENCHMARK(bm_computeCurrentsFromField_LINSAT_ANALYTICAL_CACHED);

static void bm_forwardModelLinearSaturationCurrentsJacobian_ANALYTICAL(benchmark::State& state) {
  ForwardModelLinearSaturation::Ptr p_fmls(new ForwardModelLinearSaturation);
  string cal_file = ros::package::getPath("mag_manip") + "/models/cmag_fmls_v1/params.yaml";
  ForwardModelLinear::Ptr p_fml = make_shared<ForwardModelMPEM>();
  p_fml->setCalibrationFile(ros::package::getPath("mag_manip") +
                            "/models/cmag_fmls_v1/CardioMag_CalibrationCube_03-04-19.yaml");
  p_fmls->setLinearModel(p_fml);
  p_fmls->setSaturationFunctionsFile(ros::package::getPath("mag_manip") +
                                     "/models/cmag_fmls_v1/cmag_sat_erf_v1.yaml");

  const int Ne = p_fml->getNumCoils();

  auto p_jac = make_shared<ForwardModelLinearSaturationCurrentsJacobianFunctor>();
  p_jac->setForwardModelLinearSaturation(p_fmls);

  PositionVec position;
  position.setZero();
  const CurrentsVec currents = CurrentsVec::Ones(Ne) * 12;

  while (state.KeepRunning()) {
    const auto jac = (*p_jac)(position, currents);
  }
}

BENCHMARK(bm_forwardModelLinearSaturationCurrentsJacobian_ANALYTICAL);

static void bm_forwardModelLinearSaturationCurrentsJacobian_NUMERICAL(benchmark::State& state) {
  ForwardModelLinearSaturation::Ptr p_fmls(new ForwardModelLinearSaturation);
  string cal_file = ros::package::getPath("mag_manip") + "/models/cmag_fmls_v1/params.yaml";
  ForwardModelLinear::Ptr p_fml = make_shared<ForwardModelMPEM>();
  p_fml->setCalibrationFile(ros::package::getPath("mag_manip") +
                            "/models/cmag_fmls_v1/CardioMag_CalibrationCube_03-04-19.yaml");
  p_fmls->setLinearModel(p_fml);
  p_fmls->setSaturationFunctionsFile(ros::package::getPath("mag_manip") +
                                     "/models/cmag_fmls_v1/cmag_sat_erf_v1.yaml");

  const int Ne = p_fml->getNumCoils();

  PositionVec position;
  position.setZero();
  const CurrentsVec currents = CurrentsVec::Ones(Ne) * 12;

  const FieldGradient5Vec field_grad =
      p_fmls->computeFieldGradient5FromCurrents(position, currents);

  const double eps = 1e-6;
  const Eigen::MatrixXd del = eps * Eigen::MatrixXd::Identity(Ne, Ne);

  CurrentsJacobian jac_fd(8, Ne);

  while (state.KeepRunning()) {
    for (int k = 0; k < Ne; k++) {
      CurrentsVec currents_ = currents + del.col(k);
      const FieldGradient5Vec field_grad_ =
          p_fmls->computeFieldGradient5FromCurrents(position, currents_);
      jac_fd.col(k) = (field_grad_ - field_grad) / eps;
    }
  }
}

BENCHMARK(bm_forwardModelLinearSaturationCurrentsJacobian_NUMERICAL);

BENCHMARK_MAIN();
