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

#include <iostream>

#include <gtest/gtest.h>
#include <ros/package.h>
#include <tsc_utils/eigen_matrix_compare.h>
#include "mag_manip/backward_model_mpem_L2.h"
#include "mag_manip/backward_model_saturation.h"
#include "mag_manip/exceptions.h"
#include "mag_manip/forward_model_mpem.h"
#include "mag_manip/forward_model_saturation.h"
#include "mag_manip/helpers.h"
#include "mag_manip/saturation_tanh.h"
#include "mag_manip/utils.h"

using namespace mag_manip;
using namespace std;

TEST(isValid, def) {
  string cal_file = ros::package::getPath("mag_manip") + "/test/OctoMag_Calibration.yaml";

  BackwardModelLinearL2::Ptr p_lin_model(new BackwardModelMPEML2);
  p_lin_model->setCalibrationFile(cal_file);

  BackwardModelSaturation model;
  EXPECT_FALSE(model.isValid());
  model.setModel(p_lin_model);
  EXPECT_FALSE(model.isValid());

  Eigen::Vector2d params(16.0, 1 / 16.0);
  vector<SaturationFunction::Ptr> sat_functions;
  for (int i = 0; i < model.getNumCoils(); i++) {
    sat_functions.push_back(make_unique<SaturationTanh>(params));
  }
  model.setSaturationFunctions(sat_functions);

  EXPECT_TRUE(model.isValid());
}

TEST(getNumCoils, def) {
  string cal_file = ros::package::getPath("mag_manip") + "/test/OctoMag_Calibration.yaml";

  BackwardModelLinearL2::Ptr p_lin_model(new BackwardModelMPEML2);
  p_lin_model->setCalibrationFile(cal_file);

  BackwardModelSaturation model;
  model.setModel(p_lin_model);

  EXPECT_EQ(model.getNumCoils(), 8);
}

TEST(setModelCalibrationFile, def) {
  string cal_file = ros::package::getPath("mag_manip") + "/test/OctoMag_Calibration.yaml";
  BackwardModelLinearL2::Ptr p_lin_model(new BackwardModelMPEML2);
  BackwardModelSaturation model;
  model.setModel(p_lin_model);
  model.setModelCalibrationFile(cal_file);
  Eigen::Vector2d params(16.0, 1 / 16.);
  vector<SaturationFunction::Ptr> sat_functions;
  for (int i = 0; i < model.getNumCoils(); i++) {
    sat_functions.push_back(make_unique<SaturationTanh>(params));
  }
  model.setSaturationFunctions(sat_functions);
  EXPECT_TRUE(model.isValid());
}

TEST(setCalibrationFile, def) {
  string cal_file = ros::package::getPath("mag_manip") + "/test/cmag_bml2s_v1/params.yaml";

  BackwardModelSaturation model;
  model.setCalibrationFile(cal_file);
  EXPECT_TRUE(model.isValid());
}

TEST(getName, name) {
  BackwardModelSaturation model;
  EXPECT_EQ(model.getName(), "");
  model.setName("name");
  EXPECT_EQ(model.getName(), "name");
}

TEST(computeCurrentsFromField, too_high) {
  string cal_file_f = ros::package::getPath("mag_manip") + "/test/cmag_fms_v1/params.yaml";

  ForwardModelSaturation fmodel;
  fmodel.setCalibrationFile(cal_file_f);
  ASSERT_TRUE(fmodel.isValid());

  auto currents = CurrentsVec(8);
  auto position = PositionVec::Zero();
  currents << 35., 35., -35., 0., -35., 0., -35., 35.;
  auto field = fmodel.computeFieldFromCurrents(position, currents);

  string cal_file_b = ros::package::getPath("mag_manip") + "/test/cmag_bml2s_v1/params.yaml";
  BackwardModelSaturation bmodel;
  bmodel.setCalibrationFile(cal_file_b);
  ASSERT_TRUE(bmodel.isValid());
  EXPECT_THROW(bmodel.computeCurrentsFromField(position, field), OverSaturationException);
}

TEST(computeCurrentsFromField, backwardForward) {
  string cal_file = ros::package::getPath("mag_manip") + "/test/OctoMag_Calibration.yaml";
  BackwardModelLinearL2::Ptr p_lin_model(new BackwardModelMPEML2);
  BackwardModelSaturation model;
  model.setModel(p_lin_model);
  model.setModelCalibrationFile(cal_file);
  Eigen::Vector2d params(8.0, 1 / 8.);
  vector<SaturationFunction::Ptr> sat_functions;
  for (int i = 0; i < model.getNumCoils(); i++) {
    sat_functions.push_back(make_unique<SaturationTanh>(params));
  }
  model.setSaturationFunctions(sat_functions);

  ForwardModelMPEM::Ptr p_f_lin_model(new ForwardModelMPEM);
  p_f_lin_model->setCalibrationFile(cal_file);
  ForwardModelSaturation f_model;
  f_model.setModel(p_f_lin_model);
  vector<SaturationFunction::Ptr> sat_functions_f;
  for (int i = 0; i < model.getNumCoils(); i++) {
    sat_functions_f.push_back(make_unique<SaturationTanh>(params));
  }
  f_model.setSaturationFunctions(sat_functions_f);

  PositionVec position = PositionVec::Zero();

  FieldVec field(0.01, 0, 0);
  CurrentsVec currents = model.computeCurrentsFromField(position, field);

  FieldVec field_ = f_model.computeFieldFromCurrents(position, currents);

  // cout << field.transpose() << endl;
  // cout << field_.transpose() << endl;

  EXPECT_TRUE(EIGEN_MATRIX_NEAR_ABS(field, field_, 1e-4));
}

TEST(computeCurrentsFromFieldGradient5, backwardForward) {
  string cal_file = ros::package::getPath("mag_manip") + "/test/OctoMag_Calibration.yaml";
  BackwardModelLinearL2::Ptr p_lin_model(new BackwardModelMPEML2);
  BackwardModelSaturation model;
  model.setModel(p_lin_model);
  model.setModelCalibrationFile(cal_file);
  Eigen::Vector2d params(8.0, 1 / 8.);
  vector<SaturationFunction::Ptr> sat_functions;
  for (int i = 0; i < model.getNumCoils(); i++) {
    sat_functions.push_back(make_unique<SaturationTanh>(params));
  }
  model.setSaturationFunctions(sat_functions);

  ForwardModelMPEM::Ptr p_f_lin_model(new ForwardModelMPEM);
  p_f_lin_model->setCalibrationFile(cal_file);
  ForwardModelSaturation f_model;
  f_model.setModel(p_f_lin_model);
  vector<SaturationFunction::Ptr> sat_functions_f;
  for (int i = 0; i < model.getNumCoils(); i++) {
    sat_functions_f.push_back(make_unique<SaturationTanh>(params));
  }
  f_model.setSaturationFunctions(sat_functions_f);

  PositionVec position = PositionVec::Zero();

  FieldVec field(0.01, 0, 0);
  Gradient5Vec gradient;
  gradient << 0.3, 0, 0, 0, 0;
  FieldGradient5Vec field_gradient;
  field_gradient << field, gradient;
  CurrentsVec currents = model.computeCurrentsFromFieldGradient5(position, field, gradient);

  FieldGradient5Vec field_gradient_ = f_model.computeFieldGradient5FromCurrents(position, currents);

  EXPECT_TRUE(EIGEN_MATRIX_NEAR_ABS(field_gradient, field_gradient_, 1e-4));
}

TEST(computeCurrentsFromFieldDipoleGradient3, backwardForward) {
  string cal_file = ros::package::getPath("mag_manip") + "/test/OctoMag_Calibration.yaml";
  BackwardModelLinearL2::Ptr p_lin_model(new BackwardModelMPEML2);
  BackwardModelSaturation model;
  model.setModel(p_lin_model);
  model.setModelCalibrationFile(cal_file);
  Eigen::Vector2d params(8.0, 1 / 8.);
  vector<SaturationFunction::Ptr> sat_functions;
  for (int i = 0; i < model.getNumCoils(); i++) {
    sat_functions.push_back(make_unique<SaturationTanh>(params));
  }
  model.setSaturationFunctions(sat_functions);

  ForwardModelMPEM::Ptr p_f_lin_model(new ForwardModelMPEM);
  p_f_lin_model->setCalibrationFile(cal_file);
  ForwardModelSaturation f_model;
  f_model.setModel(p_f_lin_model);
  vector<SaturationFunction::Ptr> sat_functions_f;
  for (int i = 0; i < model.getNumCoils(); i++) {
    sat_functions_f.push_back(make_unique<SaturationTanh>(params));
  }
  f_model.setSaturationFunctions(sat_functions_f);

  PositionVec position = PositionVec::Zero();

  FieldVec field(0.01, 0, 0);
  DipoleVec dipole(1, 0, 0);
  Gradient3Vec gradient;
  gradient << 0.3, 0, 0;
  CurrentsVec currents =
      model.computeCurrentsFromFieldDipoleGradient3(position, field, dipole, gradient);

  FieldGradient5Vec field_gradient = f_model.computeFieldGradient5FromCurrents(position, currents);

  Gradient3Vec gradient_ = gradient5VecToGradientMat(field_gradient.tail<5>()) * dipole;
  FieldVec field_ = field_gradient.head<3>();

  EXPECT_TRUE(EIGEN_MATRIX_NEAR_ABS(field, field_, 1e-4));
  EXPECT_TRUE(EIGEN_MATRIX_NEAR_ABS(gradient, gradient_, 1e-4));
}

TEST(getSaturationFunctions, check_equal) {
  string cal_file = ros::package::getPath("mag_manip") + "/test/OctoMag_Calibration.yaml";
  BackwardModelMPEML2::Ptr p_lin_model(new BackwardModelMPEML2);
  p_lin_model->setCalibrationFile(cal_file);
  BackwardModelSaturation model;
  model.setModel(p_lin_model);
  Eigen::Vector2d params(16.0, 1 / 16.0);
  vector<SaturationFunction::Ptr> sat_functions;
  for (int i = 0; i < model.getNumCoils(); i++) {
    sat_functions.push_back(make_shared<SaturationTanh>(params));
  }
  model.setSaturationFunctions(sat_functions);
  vector<SaturationFunction::Ptr> sat_functions_ = model.getSaturationFunctions();

  bool are_equal = std::equal(begin(sat_functions), end(sat_functions), begin(sat_functions_),
                              [](const SaturationFunction::Ptr lhs,
                                 const SaturationFunction::Ptr rhs) { return &*lhs == &*rhs; });

  EXPECT_TRUE(are_equal);
}

TEST(getSaturationFunction, check_equal) {
  string cal_file = ros::package::getPath("mag_manip") + "/test/OctoMag_Calibration.yaml";
  BackwardModelMPEML2::Ptr p_lin_model(new BackwardModelMPEML2);
  p_lin_model->setCalibrationFile(cal_file);
  BackwardModelSaturation model;
  model.setModel(p_lin_model);
  Eigen::Vector2d params(16.0, 1 / 16.0);
  vector<SaturationFunction::Ptr> sat_functions;
  for (int i = 0; i < model.getNumCoils(); i++) {
    sat_functions.push_back(make_shared<SaturationTanh>(params));
  }
  model.setSaturationFunctions(sat_functions);
  for (int i = 0; i < sat_functions.size(); i++) {
    SaturationFunction::Ptr p_sat_function_ = model.getSaturationFunction(i);
    EXPECT_EQ(&*sat_functions[i], &*p_sat_function_);
  }
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
