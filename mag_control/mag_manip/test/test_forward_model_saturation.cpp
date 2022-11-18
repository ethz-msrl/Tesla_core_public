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

#include <algorithm>
#include <iostream>

#include <gtest/gtest.h>
#include <ros/package.h>
#include <tsc_utils/eigen_matrix_compare.h>
#include "mag_manip/exceptions.h"
#include "mag_manip/forward_model_mpem.h"
#include "mag_manip/forward_model_saturation.h"
#include "mag_manip/saturation_tanh.h"
#include "mag_manip/utils.h"

using namespace mag_manip;
using namespace std;

TEST(isValid, def) {
  string cal_file = ros::package::getPath("mag_manip") + "/test/OctoMag_Calibration.yaml";

  ForwardModelLinear::Ptr p_lin_model(new ForwardModelMPEM);
  p_lin_model->setCalibrationFile(cal_file);

  ForwardModelSaturation model;
  EXPECT_FALSE(model.isValid());
  model.setModel(p_lin_model);
  EXPECT_FALSE(model.isValid());

  Eigen::Vector2d params(16.0, 1 / 16.0);
  vector<SaturationFunction::Ptr> sat_functions;
  for (int i = 0; i < model.getNumCoils(); i++) {
    sat_functions.push_back(make_shared<SaturationTanh>(params));
  }
  model.setSaturationFunctions(sat_functions);

  EXPECT_TRUE(model.isValid());
}

TEST(getNumCoils, def) {
  string cal_file = ros::package::getPath("mag_manip") + "/test/OctoMag_Calibration.yaml";

  ForwardModelLinear::Ptr p_lin_model(new ForwardModelMPEM);
  p_lin_model->setCalibrationFile(cal_file);

  ForwardModelSaturation model;
  model.setModel(p_lin_model);

  EXPECT_EQ(model.getNumCoils(), 8);
}

TEST(setModelCalibrationFile, def) {
  string cal_file = ros::package::getPath("mag_manip") + "/test/OctoMag_Calibration.yaml";
  ForwardModelLinear::Ptr p_lin_model(new ForwardModelMPEM);
  ForwardModelSaturation model;
  model.setModel(p_lin_model);
  model.setModelCalibrationFile(cal_file);
  Eigen::Vector2d params(16.0, 1 / 16.);
  vector<SaturationFunction::Ptr> sat_functions;
  for (int i = 0; i < model.getNumCoils(); i++) {
    sat_functions.push_back(make_shared<SaturationTanh>(params));
  }
  model.setSaturationFunctions(sat_functions);
  EXPECT_TRUE(model.isValid());
}

TEST(setCalibrationFile, def) {
  string cal_file = ros::package::getPath("mag_manip") + "/test/cmag_fms_v1/params.yaml";

  ForwardModelSaturation model;
  model.setCalibrationFile(cal_file);
  EXPECT_TRUE(model.isValid());
}

TEST(getName, name) {
  ForwardModelSaturation model;
  EXPECT_EQ(model.getName(), "");
  model.setName("name");
  EXPECT_EQ(model.getName(), "name");
}

TEST(computeFieldFromCurrents, zeros) {
  string cal_file = ros::package::getPath("mag_manip") + "/test/OctoMag_Calibration.yaml";
  ForwardModelLinear::Ptr p_lin_model(new ForwardModelMPEM);
  ForwardModelSaturation model;
  model.setModel(p_lin_model);
  model.setModelCalibrationFile(cal_file);
  Eigen::Vector2d params(16.0, 1 / 16.0);
  vector<SaturationFunction::Ptr> sat_functions;
  for (int i = 0; i < model.getNumCoils(); i++) {
    sat_functions.push_back(make_shared<SaturationTanh>(params));
  }
  model.setSaturationFunctions(sat_functions);
  CurrentsVec currents = CurrentsVec::Zero(8);
  PositionVec position = PositionVec::Zero();
  FieldVec field = model.computeFieldFromCurrents(position, currents);
  EXPECT_TRUE(EIGEN_MATRIX_NEAR_ABS(field, Eigen::Vector3d::Zero(), 1e-3));
}

TEST(computeFieldFromCurrents, ones) {
  string cal_file = ros::package::getPath("mag_manip") + "/test/OctoMag_Calibration.yaml";
  ForwardModelLinear::Ptr p_lin_model(new ForwardModelMPEM);
  ForwardModelSaturation model;
  model.setModel(p_lin_model);
  model.setModelCalibrationFile(cal_file);
  Eigen::Vector2d params(16.0, 1 / 16.0);
  vector<SaturationFunction::Ptr> sat_functions;
  for (int i = 0; i < model.getNumCoils(); i++) {
    sat_functions.push_back(make_shared<SaturationTanh>(params));
  }
  model.setSaturationFunctions(sat_functions);
  CurrentsVec currents = 2 * CurrentsVec::Ones(8);
  PositionVec position = PositionVec::Zero();
  FieldVec field = model.computeFieldFromCurrents(position, currents);
  EXPECT_FALSE(EIGEN_MATRIX_NEAR_ABS(field, Eigen::Vector3d::Zero(), 1e-4));
}

TEST(computeGradient5FromCurrents, ones) {
  string cal_file = ros::package::getPath("mag_manip") + "/test/OctoMag_Calibration.yaml";
  ForwardModelLinear::Ptr p_lin_model(new ForwardModelMPEM);
  ForwardModelSaturation model;
  p_lin_model->setCalibrationFile(cal_file);
  model.setModel(p_lin_model);
  Eigen::Vector2d params(16.0, 1 / 16.0);
  vector<SaturationFunction::Ptr> sat_functions;
  for (int i = 0; i < model.getNumCoils(); i++) {
    sat_functions.push_back(make_shared<SaturationTanh>(params));
  }
  model.setSaturationFunctions(sat_functions);
  CurrentsVec currents = 8 * CurrentsVec::Ones(8);
  PositionVec position = PositionVec::Zero();
  Gradient5Vec gradient_s = model.computeGradient5FromCurrents(position, currents);
  Gradient5Vec gradient = model.getModel()->computeGradient5FromCurrents(position, currents);

  EXPECT_GT((gradient).norm() - (gradient_s).norm(), 0);
}

TEST(computeFieldGradient5FromCurrents, ones) {
  string cal_file = ros::package::getPath("mag_manip") + "/test/OctoMag_Calibration.yaml";
  ForwardModelLinear::Ptr p_lin_model(new ForwardModelMPEM);
  p_lin_model->setCalibrationFile(cal_file);
  ForwardModelSaturation model;
  model.setModel(p_lin_model);
  Eigen::Vector2d params(16.0, 1 / 16.0);
  vector<SaturationFunction::Ptr> sat_functions;
  for (int i = 0; i < model.getNumCoils(); i++) {
    sat_functions.push_back(make_shared<SaturationTanh>(params));
  }
  model.setSaturationFunctions(sat_functions);
  CurrentsVec currents = 8 * CurrentsVec::Ones(8);
  PositionVec position = PositionVec::Zero();
  Eigen::VectorXd field_gradient =
      model.getModel()->computeFieldGradient5FromCurrents(position, currents);
  Eigen::VectorXd field_gradient_s = model.computeFieldGradient5FromCurrents(position, currents);
  EXPECT_GT((field_gradient).norm() - (field_gradient_s).norm(), 0);
}

TEST(getSaturationFunctions, check_equal) {
  string cal_file = ros::package::getPath("mag_manip") + "/test/OctoMag_Calibration.yaml";
  ForwardModelLinear::Ptr p_lin_model(new ForwardModelMPEM);
  p_lin_model->setCalibrationFile(cal_file);
  ForwardModelSaturation model;
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
  ForwardModelLinear::Ptr p_lin_model(new ForwardModelMPEM);
  p_lin_model->setCalibrationFile(cal_file);
  ForwardModelSaturation model;
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

TEST(computeCurrentsFromFieldCached, ones) {
  string cal_file = ros::package::getPath("mag_manip") + "/test/OctoMag_Calibration.yaml";
  ForwardModelLinear::Ptr p_lin_model(new ForwardModelMPEM);
  ForwardModelSaturation model;
  model.setModel(p_lin_model);
  model.setModelCalibrationFile(cal_file);
  Eigen::Vector2d params(16.0, 1 / 16.0);
  vector<SaturationFunction::Ptr> sat_functions;
  for (int i = 0; i < model.getNumCoils(); i++) {
    sat_functions.push_back(make_shared<SaturationTanh>(params));
  }
  model.setSaturationFunctions(sat_functions);
  CurrentsVec currents = 2 * CurrentsVec::Ones(8);
  PositionVec position = PositionVec::Zero();
  model.setCachedPosition(position);
  FieldVec field = model.computeFieldFromCurrentsCached(currents);
  EXPECT_FALSE(EIGEN_MATRIX_NEAR_ABS(field, Eigen::Vector3d::Zero(), 1e-4));
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
