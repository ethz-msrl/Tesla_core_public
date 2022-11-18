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

#include <gtest/gtest.h>
#include <ros/package.h>
#include <tsc_utils/eigen_matrix_compare.h>
#include <numeric>
// for calculation of the least squares linear solution
#include <Eigen/SVD>

#include "mag_manip/backward_model_nls.h"
#include "mag_manip/exceptions.h"
#include "mag_manip/forward_model_linear_saturation.h"
#include "mag_manip/forward_model_linear_saturation_currents_jacobian_functor.h"
#include "mag_manip/forward_model_mpem.h"
#include "mag_manip/helpers.h"

using namespace mag_manip;
using namespace std;

TEST(Constructor, def) {
  BackwardModelNLS model;
  ASSERT_FALSE(model.isValid());
}

TEST(setCalibrationFile, createAndComputeCurrentsFromField) {
  BackwardModelNLS model;
  string cal_file = ros::package::getPath("mag_manip") + "/test/cmag_bmnlsmpem_v1/params.yaml";
  model.setCalibrationFile(cal_file);
  EXPECT_TRUE(model.isValid());
  EXPECT_TRUE(model.getDoMinimizeCurrents());
  EXPECT_FALSE(model.getDoPrintSummary());
  // EXPECT_NEAR(model.getMaximumFieldDistance(), 1e-6, 1e-9);

  FieldVec field(20e-3, 0, 0);
  PositionVec position(0, 0, 0);
  auto ret_vals = model.computeCurrentsFromFieldRet(position, field);
  EXPECT_TRUE(ret_vals.first);
}

TEST(setForwardModelCalibrationFile, constructWithoutModel) {
  BackwardModelNLS model;
  string cal_file = ros::package::getPath("mag_manip") + "/test/OctoMag_Calibration.yaml";
  EXPECT_THROW(model.setForwardModelCalibrationFile(cal_file), CalibrationNotLoaded);
}

TEST(isValid, valid) {
  ForwardModel::UPtr p_forward_model(new ForwardModelMPEM);
  string cal_file = ros::package::getPath("mag_manip") + "/test/OctoMag_Calibration.yaml";
  p_forward_model->setCalibrationFile(cal_file);
  BackwardModelNLS backward_model;
  backward_model.setForwardModel(std::move(p_forward_model));
  EXPECT_TRUE(backward_model.isValid());
}

TEST(getNumCoils, valid) {
  ForwardModel::UPtr p_forward_model(new ForwardModelMPEM);
  string cal_file = ros::package::getPath("mag_manip") + "/test/OctoMag_Calibration.yaml";
  p_forward_model->setCalibrationFile(cal_file);
  BackwardModelNLS backward_model;
  backward_model.setForwardModel(std::move(p_forward_model));
  EXPECT_EQ(backward_model.getNumCoils(), 8);
}

TEST(computeCurrentsFromField, success) {
  ForwardModel::UPtr p_forward_model(new ForwardModelMPEM);
  string cal_file = ros::package::getPath("mag_manip") + "/test/OctoMag_Calibration.yaml";
  p_forward_model->setCalibrationFile(cal_file);
  BackwardModelNLS backward_model;
  backward_model.setForwardModel(std::move(p_forward_model));

  FieldVec field(20e-3, 0, 0);
  PositionVec position(0, 0, 0);
  auto ret_vals = backward_model.computeCurrentsFromFieldRet(position, field);
  EXPECT_TRUE(ret_vals.first);
}

TEST(computeCurrentsFromField, failure) {
  ForwardModel::UPtr p_forward_model(new ForwardModelMPEM);
  string cal_file = ros::package::getPath("mag_manip") + "/test/OctoMag_Calibration.yaml";
  p_forward_model->setCalibrationFile(cal_file);
  BackwardModelNLS backward_model;
  backward_model.setForwardModel(std::move(p_forward_model));

  FieldVec field(NAN, 0, 0);
  PositionVec position(0, 0, 0);
  auto ret_vals = backward_model.computeCurrentsFromFieldRet(position, field);
  EXPECT_FALSE(ret_vals.first);
}

TEST(setSolverOptions, test) {
  ForwardModel::UPtr p_forward_model(new ForwardModelMPEM);
  string cal_file = ros::package::getPath("mag_manip") + "/test/OctoMag_Calibration.yaml";
  p_forward_model->setCalibrationFile(cal_file);
  BackwardModelNLS backward_model;
  backward_model.setForwardModel(std::move(p_forward_model));
  ceres::Solver::Options options;
  options.minimizer_progress_to_stdout = true;
  backward_model.setSolverOptions(options);
  FieldVec field(20e-3, 0, 0);
  PositionVec position(0, 0, 0);
  CurrentsVec currents = backward_model.computeCurrentsFromField(position, field);
  FieldVec field_ = backward_model.getForwardModel().computeFieldFromCurrents(position, currents);
  EXPECT_TRUE(EIGEN_MATRIX_NEAR_ABS(field, field_, 1e-6));
}

TEST(computeCurrentsFromField, def) {
  ForwardModel::UPtr p_forward_model(new ForwardModelMPEM);
  string cal_file = ros::package::getPath("mag_manip") + "/test/OctoMag_Calibration.yaml";
  p_forward_model->setCalibrationFile(cal_file);
  BackwardModelNLS backward_model;
  backward_model.setForwardModel(std::move(p_forward_model));
  FieldVec field(20e-3, 0, 0);
  PositionVec position(0, 0, 0);
  CurrentsVec currents = backward_model.computeCurrentsFromField(position, field);
  cout << "currents to generate field: " << field.transpose() << endl;
  cout << currents.transpose() << endl;
  FieldVec field_ = backward_model.getForwardModel().computeFieldFromCurrents(position, currents);
  EXPECT_TRUE(EIGEN_MATRIX_NEAR_ABS(field, field_, 1e-6));
}

TEST(computeCurrentsFromField, minimizeCurrents) {
  ForwardModelMPEM::UPtr p_forward_model(new ForwardModelMPEM);
  string cal_file = ros::package::getPath("mag_manip") + "/test/OctoMag_Calibration.yaml";
  PositionVec position(0, 0, 0);
  p_forward_model->setCalibrationFile(cal_file);
  ActuationMat act_mat = p_forward_model->getFieldActuationMatrix(position);
  BackwardModelNLS backward_model;
  backward_model.setForwardModel(std::move(p_forward_model));
  backward_model.setDoMinimizeCurrents(true);
  backward_model.setWeightCurrents(1e-9);

  FieldVec field(20e-3, 0, 0);

  CurrentsVec currents = backward_model.computeCurrentsFromField(position, field);
  cout << "currents to generate field: " << field.transpose() << endl;
  cout << currents.transpose() << endl;

  Eigen::JacobiSVD<ActuationMat> svd(act_mat, Eigen::ComputeThinU | Eigen::ComputeThinV);

  CurrentsVec currents_ls = svd.solve(field);

  cout << "least squares currents: " << endl;
  cout << currents_ls.transpose() << endl;

  FieldVec field_ = backward_model.getForwardModel().computeFieldFromCurrents(position, currents);
  EXPECT_TRUE(EIGEN_MATRIX_NEAR_ABS(field, field_, 1e-5));
}

TEST(setInitialCurrents, ones) {
  ForwardModel::UPtr p_forward_model(new ForwardModelMPEM);
  string cal_file = ros::package::getPath("mag_manip") + "/test/OctoMag_Calibration.yaml";
  p_forward_model->setCalibrationFile(cal_file);
  BackwardModelNLS backward_model;
  backward_model.setForwardModel(std::move(p_forward_model));
  FieldVec field(20e-3, 0, 0);
  PositionVec position(0, 0, 0);
  CurrentsVec init_currents(backward_model.getNumCoils());
  init_currents << 1, 1, 1, 1, 1, 1, 1, 1;
  backward_model.setInitialCurrents(init_currents);

  CurrentsVec currents = backward_model.computeCurrentsFromField(position, field);
  FieldVec field_ = backward_model.getForwardModel().computeFieldFromCurrents(position, currents);
  EXPECT_TRUE(EIGEN_MATRIX_NEAR_ABS(field, field_, 1e-6));
}

TEST(setDoPrintSummary, print_on) {
  ForwardModel::UPtr p_forward_model(new ForwardModelMPEM);
  string cal_file = ros::package::getPath("mag_manip") + "/test/OctoMag_Calibration.yaml";
  p_forward_model->setCalibrationFile(cal_file);
  BackwardModelNLS backward_model;
  backward_model.setForwardModel(std::move(p_forward_model));
  backward_model.setDoPrintSummary(true);
  FieldVec field(20e-3, 0, 0);
  PositionVec position(0, 0, 0);
  CurrentsVec init_currents(backward_model.getNumCoils());
  init_currents << 1, 1, 1, 1, 1, 1, 1, 1;
  backward_model.setInitialCurrents(init_currents);

  CurrentsVec currents = backward_model.computeCurrentsFromField(position, field);
  FieldVec field_ = backward_model.getForwardModel().computeFieldFromCurrents(position, currents);
  EXPECT_TRUE(EIGEN_MATRIX_NEAR_ABS(field, field_, 1e-6));
}

TEST(computeCurrentsFromFieldGradient5, def) {
  ForwardModel::UPtr p_forward_model(new ForwardModelMPEM);
  string cal_file = ros::package::getPath("mag_manip") + "/test/OctoMag_Calibration.yaml";
  p_forward_model->setCalibrationFile(cal_file);
  BackwardModelNLS backward_model;
  backward_model.setForwardModel(std::move(p_forward_model));
  FieldVec field(20e-3, 0, 0);
  Gradient5Vec gradient;
  gradient << 0.4, 0, 0, 0, 0;
  PositionVec position(0, 0, 0);
  CurrentsVec currents =
      backward_model.computeCurrentsFromFieldGradient5(position, field, gradient);
  FieldVec field_ = backward_model.getForwardModel().computeFieldFromCurrents(position, currents);
  Gradient5Vec gradient_ =
      backward_model.getForwardModel().computeGradient5FromCurrents(position, currents);
  EXPECT_TRUE(EIGEN_MATRIX_NEAR_ABS(field, field_, 1e-6));
  EXPECT_TRUE(EIGEN_MATRIX_NEAR_ABS(gradient, gradient_, 1e-6));
}

TEST(computeCurrentsFromFieldGradient5, success) {
  ForwardModel::UPtr p_forward_model(new ForwardModelMPEM);
  string cal_file = ros::package::getPath("mag_manip") + "/test/OctoMag_Calibration.yaml";
  p_forward_model->setCalibrationFile(cal_file);
  BackwardModelNLS backward_model;
  backward_model.setForwardModel(std::move(p_forward_model));
  FieldVec field(20e-3, 0, 0);
  Gradient5Vec gradient;
  gradient << 0.4, 0, 0, 0, 0;
  PositionVec position(0, 0, 0);
  auto ret_vals = backward_model.computeCurrentsFromFieldGradient5Ret(position, field, gradient);
  EXPECT_TRUE(ret_vals.first);
}

TEST(computeCurrentsFromFieldGradient5, failure) {
  ForwardModel::UPtr p_forward_model(new ForwardModelMPEM);
  string cal_file = ros::package::getPath("mag_manip") + "/test/OctoMag_Calibration.yaml";
  p_forward_model->setCalibrationFile(cal_file);
  BackwardModelNLS backward_model;
  backward_model.setForwardModel(std::move(p_forward_model));
  FieldVec field(NAN, 0, 0);
  Gradient5Vec gradient;
  gradient << 0.4, 0, 0, 0, 0;
  PositionVec position(0, 0, 0);
  auto ret_vals = backward_model.computeCurrentsFromFieldGradient5Ret(position, field, gradient);
  EXPECT_FALSE(ret_vals.first);
}

TEST(computeCurrentsFromFieldDipoleGradient3, def) {
  ForwardModel::UPtr p_forward_model(new ForwardModelMPEM);
  string cal_file = ros::package::getPath("mag_manip") + "/test/OctoMag_Calibration.yaml";
  p_forward_model->setCalibrationFile(cal_file);
  BackwardModelNLS backward_model;
  backward_model.setForwardModel(std::move(p_forward_model));
  FieldVec field(20e-3, 0, 0);
  Gradient3Vec gradient(0.3, 0, 0);
  DipoleVec dipole(1, 0, 0);
  PositionVec position(0, 0, 0);
  CurrentsVec currents =
      backward_model.computeCurrentsFromFieldDipoleGradient3(position, field, dipole, gradient);
  FieldVec field_ = backward_model.getForwardModel().computeFieldFromCurrents(position, currents);
  Gradient5Vec gradient5 =
      backward_model.getForwardModel().computeGradient5FromCurrents(position, currents);

  Gradient3Vec gradient_ = directedGradient3Mat(dipole) * gradient5;

  EXPECT_TRUE(EIGEN_MATRIX_NEAR_ABS(field, field_, 1e-6));
  EXPECT_TRUE(EIGEN_MATRIX_NEAR_ABS(gradient, gradient_, 1e-6));
}

TEST(computeCurrentsFromFieldDipoleGradient3, success) {
  ForwardModel::UPtr p_forward_model(new ForwardModelMPEM);
  string cal_file = ros::package::getPath("mag_manip") + "/test/OctoMag_Calibration.yaml";
  p_forward_model->setCalibrationFile(cal_file);
  BackwardModelNLS backward_model;
  backward_model.setForwardModel(std::move(p_forward_model));
  FieldVec field(20e-3, 0, 0);
  Gradient3Vec gradient(0.3, 0, 0);
  DipoleVec dipole(1, 0, 0);
  PositionVec position(0, 0, 0);
  auto ret_vals =
      backward_model.computeCurrentsFromFieldDipoleGradient3Ret(position, field, dipole, gradient);
  EXPECT_TRUE(ret_vals.first);
}

TEST(computeCurrentsFromFieldDipoleGradient3, failure) {
  ForwardModel::UPtr p_forward_model(new ForwardModelMPEM);
  string cal_file = ros::package::getPath("mag_manip") + "/test/OctoMag_Calibration.yaml";
  p_forward_model->setCalibrationFile(cal_file);
  BackwardModelNLS backward_model;
  backward_model.setForwardModel(std::move(p_forward_model));
  FieldVec field(NAN, 0, 0);
  Gradient3Vec gradient(0.3, 0, 0);
  DipoleVec dipole(1, 0, 0);
  PositionVec position(0, 0, 0);
  CurrentsVec currents;
  auto ret_vals =
      backward_model.computeCurrentsFromFieldDipoleGradient3Ret(position, field, dipole, gradient);
  EXPECT_FALSE(ret_vals.first);
}

TEST(computeCurrentsFromFieldRet, numerical_jacobians) {
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
  backward_model.setDoPrintSummary(true);

  FieldVec field(20e-3, 0, 0);
  PositionVec position(0, 0, 0);

  auto ret_vals = backward_model.computeCurrentsFromFieldRet(position, field);

  EXPECT_TRUE(ret_vals.first);
  auto field_ = p_fmls->computeFieldFromCurrents(position, ret_vals.second);
  EXPECT_TRUE(EIGEN_MATRIX_NEAR_ABS(field, field_, 1e-6));
}

TEST(computeCurrentsFromFieldRet, analytical_jacobians) {
  ForwardModelLinearSaturation::Ptr p_fmls(new ForwardModelLinearSaturation);
  string cal_file = ros::package::getPath("mag_manip") + "/models/cmag_fmls_v1/params.yaml";
  ForwardModelLinear::Ptr p_fml = make_shared<ForwardModelMPEM>();
  p_fml->setCalibrationFile(ros::package::getPath("mag_manip") +
                            "/models/cmag_fmls_v1/CardioMag_CalibrationCube_03-04-19.yaml");
  p_fmls->setLinearModel(p_fml);
  p_fmls->setSaturationFunctionsFile(ros::package::getPath("mag_manip") +
                                     "/models/cmag_fmls_v1/cmag_sat_erf_v1.yaml");
  const int Ne = p_fmls->getNumCoils();

  auto p_jac = make_shared<ForwardModelLinearSaturationCurrentsJacobianFunctor>();
  p_jac->setForwardModelLinearSaturation(p_fmls);

  BackwardModelNLS backward_model;
  backward_model.setForwardModel(p_fmls);
  backward_model.setCurrentsJacobianFunctor(p_jac);
  backward_model.setUseAnalyticalGradients(true);
  backward_model.setDoPrintSummary(true);

  FieldVec field(20e-3, 0, 0);
  PositionVec position(0, 0, 0);

  auto ret_vals = backward_model.computeCurrentsFromFieldRet(position, field);

  EXPECT_TRUE(ret_vals.first);
  auto field_ = p_fmls->computeFieldFromCurrents(position, ret_vals.second);
  EXPECT_TRUE(EIGEN_MATRIX_NEAR_ABS(field, field_, 1e-6));
}

TEST(computeCurrentsFromFieldGradient5Ret, analytical_jacobians) {
  ForwardModelLinearSaturation::Ptr p_fmls(new ForwardModelLinearSaturation);
  string cal_file = ros::package::getPath("mag_manip") + "/models/cmag_fmls_v1/params.yaml";
  ForwardModelLinear::Ptr p_fml = make_shared<ForwardModelMPEM>();
  p_fml->setCalibrationFile(ros::package::getPath("mag_manip") +
                            "/models/cmag_fmls_v1/CardioMag_CalibrationCube_03-04-19.yaml");
  p_fmls->setLinearModel(p_fml);
  p_fmls->setSaturationFunctionsFile(ros::package::getPath("mag_manip") +
                                     "/models/cmag_fmls_v1/cmag_sat_erf_v1.yaml");
  const int Ne = p_fmls->getNumCoils();

  auto p_jac = make_shared<ForwardModelLinearSaturationCurrentsJacobianFunctor>();
  p_jac->setForwardModelLinearSaturation(p_fmls);

  BackwardModelNLS backward_model;
  backward_model.setForwardModel(p_fmls);
  backward_model.setCurrentsJacobianFunctor(p_jac);
  backward_model.setDoPrintSummary(true);
  backward_model.setUseAnalyticalGradients(true);
  backward_model.setMaximumFieldDistance(200e-6);

  FieldVec field(10e-3, 0, 0);
  Gradient5Vec gradient;
  gradient << 0.1, 0, 0, 0, 0;
  PositionVec position(0, 0, 0);
  auto ret_vals = backward_model.computeCurrentsFromFieldGradient5Ret(position, field, gradient);

  EXPECT_TRUE(ret_vals.first);
  auto field_gradient_ = p_fmls->computeFieldGradient5FromCurrents(position, ret_vals.second);
  EXPECT_TRUE(EIGEN_MATRIX_NEAR_ABS(field, field_gradient_.head(3), 1e-3));
  EXPECT_TRUE(EIGEN_MATRIX_NEAR_ABS(gradient, field_gradient_.tail(5), 1e-3));
}

TEST(computeCurrentsFromFieldDipoleGradient3Ret, analytical_jacobians) {
  ForwardModelLinearSaturation::Ptr p_fmls(new ForwardModelLinearSaturation);
  string cal_file = ros::package::getPath("mag_manip") + "/models/cmag_fmls_v1/params.yaml";
  ForwardModelLinear::Ptr p_fml = make_shared<ForwardModelMPEM>();
  p_fml->setCalibrationFile(ros::package::getPath("mag_manip") +
                            "/models/cmag_fmls_v1/CardioMag_CalibrationCube_03-04-19.yaml");
  p_fmls->setLinearModel(p_fml);
  p_fmls->setSaturationFunctionsFile(ros::package::getPath("mag_manip") +
                                     "/models/cmag_fmls_v1/cmag_sat_erf_v1.yaml");
  const int Ne = p_fmls->getNumCoils();

  auto p_jac = make_shared<ForwardModelLinearSaturationCurrentsJacobianFunctor>();
  p_jac->setForwardModelLinearSaturation(p_fmls);

  BackwardModelNLS backward_model;
  backward_model.setForwardModel(p_fmls);
  backward_model.setCurrentsJacobianFunctor(p_jac);
  backward_model.setUseAnalyticalGradients(true);

  FieldVec field(10e-3, 0, 0);
  DipoleVec dipole(1, 0, 0);
  Gradient3Vec gradient;
  gradient << 0.1, 0, 0;
  PositionVec position(0, 0, 0);
  auto ret_vals =
      backward_model.computeCurrentsFromFieldDipoleGradient3Ret(position, field, dipole, gradient);

  EXPECT_TRUE(ret_vals.first);
  auto field_gradient_ = p_fmls->computeFieldGradient5FromCurrents(position, ret_vals.second);
  Gradient5Vec gradient5_ = field_gradient_.tail(5);
  Gradient3Vec gradient_ = directedGradient3Mat(dipole) * gradient5_;
  EXPECT_TRUE(EIGEN_MATRIX_NEAR_ABS(field, field_gradient_.head(3), 1e-3));
  EXPECT_TRUE(EIGEN_MATRIX_NEAR_ABS(gradient, gradient_, 1e-3));
}

TEST(computeCurrentsFromFieldRet, cached) {
  ForwardModelLinearSaturation::Ptr p_fmls(new ForwardModelLinearSaturation);
  string cal_file = ros::package::getPath("mag_manip") + "/models/cmag_fmls_v1/params.yaml";
  ForwardModelLinear::Ptr p_fml = make_shared<ForwardModelMPEM>();
  p_fml->setCalibrationFile(ros::package::getPath("mag_manip") +
                            "/models/cmag_fmls_v1/CardioMag_CalibrationCube_03-04-19.yaml");
  p_fmls->setLinearModel(p_fml);
  p_fmls->setSaturationFunctionsFile(ros::package::getPath("mag_manip") +
                                     "/models/cmag_fmls_v1/cmag_sat_erf_v1.yaml");
  const int Ne = p_fmls->getNumCoils();

  // auto p_jac =
  //   make_shared<ForwardModelLinearSaturationCurrentsJacobianFunctor>();
  // p_jac->setForwardModelLinearSaturation(p_fmls);

  BackwardModelNLS backward_model;
  backward_model.setForwardModel(p_fmls);
  // backward_model.setCurrentsJacobianFunctor(p_jac);
  backward_model.setUseAnalyticalGradients(false);
  backward_model.setDoPrintSummary(true);
  backward_model.setDoCachePosition(true);

  FieldVec field(20e-3, 0, 0);
  PositionVec position(0, 0, 0);

  auto ret_vals = backward_model.computeCurrentsFromFieldRet(position, field);

  EXPECT_TRUE(ret_vals.first);
  auto field_ = p_fmls->computeFieldFromCurrents(position, ret_vals.second);
  EXPECT_TRUE(EIGEN_MATRIX_NEAR_ABS(field, field_, 1e-6));
}

TEST(computeCurrentsFromFieldRet, analytical_jacobians_cached) {
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
  backward_model.setDoPrintSummary(true);
  backward_model.setDoCachePosition(true);

  auto ret_vals = backward_model.computeCurrentsFromFieldRet(position, field);

  EXPECT_TRUE(ret_vals.first);
  auto field_ = p_fmls->computeFieldFromCurrents(position, ret_vals.second);
  EXPECT_TRUE(EIGEN_MATRIX_NEAR_ABS(field, field_, 1e-6));
}

TEST(computeCurrentsFromFieldGradient5Ret, analytical_jacobians_cached) {
  ForwardModelLinearSaturation::Ptr p_fmls(new ForwardModelLinearSaturation);
  string cal_file = ros::package::getPath("mag_manip") + "/models/cmag_fmls_v1/params.yaml";
  ForwardModelLinear::Ptr p_fml = make_shared<ForwardModelMPEM>();
  p_fml->setCalibrationFile(ros::package::getPath("mag_manip") +
                            "/models/cmag_fmls_v1/CardioMag_CalibrationCube_03-04-19.yaml");
  p_fmls->setLinearModel(p_fml);
  p_fmls->setSaturationFunctionsFile(ros::package::getPath("mag_manip") +
                                     "/models/cmag_fmls_v1/cmag_sat_erf_v1.yaml");

  FieldVec field(10e-3, 0, 0);
  Gradient5Vec gradient;
  gradient << 0.1, 0, 0, 0, 0;
  PositionVec position(0, 0, 0);

  auto p_jac = make_shared<ForwardModelLinearSaturationCurrentsJacobianFunctor>();
  p_jac->setForwardModelLinearSaturation(p_fmls);
  p_jac->setCachedPosition(position);

  BackwardModelNLS backward_model;
  backward_model.setForwardModel(p_fmls);
  backward_model.setCurrentsJacobianFunctor(p_jac);
  backward_model.setUseAnalyticalGradients(true);
  backward_model.setDoPrintSummary(true);
  backward_model.setDoCachePosition(true);
  backward_model.setMaximumFieldDistance(200e-6);

  auto ret_vals = backward_model.computeCurrentsFromFieldGradient5Ret(position, field, gradient);

  EXPECT_TRUE(ret_vals.first);
  auto field_gradient_ = p_fmls->computeFieldGradient5FromCurrents(position, ret_vals.second);
  EXPECT_TRUE(EIGEN_MATRIX_NEAR_ABS(field, field_gradient_.head(3), 1e-3));
  EXPECT_TRUE(EIGEN_MATRIX_NEAR_ABS(gradient, field_gradient_.tail(5), 1e-3));
}

TEST(computeCurrentsFromFieldDipoleGradient3Ret, analytical_jacobians_cached) {
  ForwardModelLinearSaturation::Ptr p_fmls(new ForwardModelLinearSaturation);
  string cal_file = ros::package::getPath("mag_manip") + "/models/cmag_fmls_v1/params.yaml";
  ForwardModelLinear::Ptr p_fml = make_shared<ForwardModelMPEM>();
  p_fml->setCalibrationFile(ros::package::getPath("mag_manip") +
                            "/models/cmag_fmls_v1/CardioMag_CalibrationCube_03-04-19.yaml");
  p_fmls->setLinearModel(p_fml);
  p_fmls->setSaturationFunctionsFile(ros::package::getPath("mag_manip") +
                                     "/models/cmag_fmls_v1/cmag_sat_erf_v1.yaml");

  FieldVec field(20e-3, 0, 0);
  DipoleVec dipole(1, 0, 0);
  Gradient3Vec gradient(0.1, 0, 0);
  PositionVec position(0, 0, 0);

  auto p_jac = make_shared<ForwardModelLinearSaturationCurrentsJacobianFunctor>();
  p_jac->setForwardModelLinearSaturation(p_fmls);

  BackwardModelNLS backward_model;
  backward_model.setForwardModel(p_fmls);
  backward_model.setCurrentsJacobianFunctor(p_jac);
  backward_model.setUseAnalyticalGradients(true);
  backward_model.setDoPrintSummary(true);
  backward_model.setDoCachePosition(true);

  auto ret_vals =
      backward_model.computeCurrentsFromFieldDipoleGradient3Ret(position, field, dipole, gradient);

  EXPECT_TRUE(ret_vals.first);
  auto field_gradient_ = p_fmls->computeFieldGradient5FromCurrents(position, ret_vals.second);
  Gradient5Vec gradient5_ = field_gradient_.tail(5);
  Gradient3Vec gradient_ = directedGradient3Mat(dipole) * gradient5_;
  EXPECT_TRUE(EIGEN_MATRIX_NEAR_ABS(field, field_gradient_.head(3), 1e-3));
  EXPECT_TRUE(EIGEN_MATRIX_NEAR_ABS(gradient, gradient_, 1e-3));
}

TEST(computeCurrentsFromField, field_not_reached) {
  ForwardModel::UPtr p_forward_model(new ForwardModelMPEM);
  string cal_file = ros::package::getPath("mag_manip") + "/test/OctoMag_Calibration.yaml";
  p_forward_model->setCalibrationFile(cal_file);
  BackwardModelNLS backward_model;
  backward_model.setForwardModel(std::move(p_forward_model));
  FieldVec field(0.04, 0, 0);
  PositionVec position(0, 0, 0);

  backward_model.setMaximumFieldDistance(1e-16);
  auto ret_vals = backward_model.computeCurrentsFromFieldRet(position, field);
  EXPECT_FALSE(ret_vals.first);
}

TEST(setCalibrationFile, lin_analytical_gradients) {
  BackwardModelNLS model;
  string cal_file =
      ros::package::getPath("mag_manip") + "/test/cmag_bmnlsmpem_analytical_v1/params.yaml";
  model.setCalibrationFile(cal_file);
  EXPECT_TRUE(model.isValid());
  EXPECT_TRUE(model.getDoMinimizeCurrents());
  EXPECT_TRUE(model.getUseAnalyticalGradients());

  FieldVec field(20e-3, 0, 0);
  PositionVec position(0, 0, 0);
  auto ret_vals = model.computeCurrentsFromFieldRet(position, field);
  EXPECT_TRUE(ret_vals.first);
}

TEST(setCalibrationFile, linsat_analytical_gradients) {
  BackwardModelNLS model;
  string cal_file =
      ros::package::getPath("mag_manip") + "/test/cmag_bmnlslinsat_analytical_v1/params.yaml";
  model.setCalibrationFile(cal_file);
  EXPECT_TRUE(model.isValid());
  EXPECT_TRUE(model.getDoMinimizeCurrents());
  EXPECT_TRUE(model.getUseAnalyticalGradients());

  FieldVec field(20e-3, 0, 0);
  PositionVec position(0, 0, 0);
  auto ret_vals = model.computeCurrentsFromFieldRet(position, field);
  EXPECT_TRUE(ret_vals.first);
}

TEST(computeCurrentsFromField, throw_exception) {
  ForwardModel::UPtr p_forward_model(new ForwardModelMPEM);
  string cal_file = ros::package::getPath("mag_manip") + "/test/OctoMag_Calibration.yaml";
  p_forward_model->setCalibrationFile(cal_file);
  BackwardModelNLS backward_model;
  backward_model.setForwardModel(std::move(p_forward_model));
  FieldVec field(0.04, 0, 0);
  PositionVec position(0, 0, 0);

  backward_model.setMaximumFieldDistance(1e-16);
  backward_model.setDoThroughExceptions(true);
  EXPECT_THROW(backward_model.computeCurrentsFromField(position, field),
               FailedComputeCurrentsException);
}

TEST(computeCurrentsFromField, output_steps_to_console) {
  ForwardModel::UPtr p_forward_model(new ForwardModelMPEM);
  string cal_file = ros::package::getPath("mag_manip") + "/test/OctoMag_Calibration.yaml";
  p_forward_model->setCalibrationFile(cal_file);
  BackwardModelNLS backward_model;
  backward_model.setForwardModel(std::move(p_forward_model));
  backward_model.setDoLogIterationsToConsole(true);
  FieldVec field(0.04, 0, 0);
  PositionVec position(0, 0, 0);

  CurrentsVec currents = backward_model.computeCurrentsFromField(position, field);

  cout << "turning off logging" << endl;
  backward_model.setDoLogIterationsToConsole(false);
  currents = backward_model.computeCurrentsFromField(position, field);
}

TEST(computeCurrentsFromField, bound_max_current) {
  ForwardModel::UPtr p_forward_model(new ForwardModelMPEM);
  string cal_file = ros::package::getPath("mag_manip") + "/test/OctoMag_Calibration.yaml";
  p_forward_model->setCalibrationFile(cal_file);
  BackwardModelNLS backward_model;
  backward_model.setForwardModel(std::move(p_forward_model));

  FieldVec field(0.04, 0, 0);
  PositionVec position(0, 0, 0);

  // setting maximum current to 4A
  const CurrentsVec max_currents = CurrentsVec::Ones(backward_model.getNumCoils()) * 4;
  backward_model.setMaxCurrents(max_currents);

  backward_model.setDoThroughExceptions(false);
  const CurrentsVec currents_calc = backward_model.computeCurrentsFromField(position, field);
  EXPECT_TRUE(((max_currents - currents_calc).array() >= 0).all());
}

TEST(computeCurrentsFromField, bound_min_current) {
  ForwardModel::UPtr p_forward_model(new ForwardModelMPEM);
  string cal_file = ros::package::getPath("mag_manip") + "/test/OctoMag_Calibration.yaml";
  p_forward_model->setCalibrationFile(cal_file);
  BackwardModelNLS backward_model;
  backward_model.setForwardModel(std::move(p_forward_model));
  FieldVec field(0.04, 0, 0);
  PositionVec position(0, 0, 0);

  // setting maximum current to 4A
  const CurrentsVec min_currents = CurrentsVec::Ones(backward_model.getNumCoils()) * -4;
  backward_model.setMinCurrents(min_currents);

  backward_model.setDoThroughExceptions(false);
  const CurrentsVec currents_calc = backward_model.computeCurrentsFromField(position, field);
  EXPECT_TRUE(((min_currents - currents_calc).array() <= 0).all());
}

TEST(computeCurrentsFromField, bound_min_max_current) {
  ForwardModel::UPtr p_forward_model(new ForwardModelMPEM);
  string cal_file = ros::package::getPath("mag_manip") + "/test/OctoMag_Calibration.yaml";
  p_forward_model->setCalibrationFile(cal_file);
  BackwardModelNLS backward_model;
  backward_model.setForwardModel(std::move(p_forward_model));
  FieldVec field(0.04, 0, 0);
  PositionVec position(0, 0, 0);

  // setting maximum current to 4A
  const CurrentsVec min_currents = CurrentsVec::Ones(backward_model.getNumCoils()) * -4;
  backward_model.setMinCurrents(min_currents);

  const CurrentsVec max_currents = CurrentsVec::Ones(backward_model.getNumCoils()) * 4;
  backward_model.setMaxCurrents(max_currents);

  backward_model.setDoThroughExceptions(false);
  const CurrentsVec currents_calc = backward_model.computeCurrentsFromField(position, field);
  EXPECT_TRUE(((max_currents - currents_calc).array() >= 0).all());
  EXPECT_TRUE(((min_currents - currents_calc).array() <= 0).all());
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
