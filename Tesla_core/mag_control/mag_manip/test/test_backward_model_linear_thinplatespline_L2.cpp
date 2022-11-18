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

#include "mag_manip/backward_model_factory.h"
#include "mag_manip/backward_model_linear_thinplatespline_L2.h"
#include "mag_manip/exceptions.h"
#include "mag_manip/forward_model_linear_thinplatespline.h"
#include "mag_manip/helpers.h"

using namespace std;
using namespace mag_manip;

TEST(Constructor, defaultConstructor) {
  BackwardModelLinearThinPlateSplineL2 model;
  EXPECT_FALSE(model.isValid());
}

TEST(setCalibrationFile, Valid) {
  string filename = ros::package::getPath("mag_manip") + "/test/rbf_valid.yaml";
  BackwardModelLinearThinPlateSplineL2 model;
  model.setCalibrationFile(filename);
  EXPECT_TRUE(model.isValid());
  EXPECT_EQ(model.getNumCoils(), 3);
}

TEST(getName, def) {
  string cal_file = ros::package::getPath("mag_manip") + "/test/rbf_valid.yaml";
  BackwardModelLinearThinPlateSplineL2 model;
  model.setCalibrationFile(cal_file);
  EXPECT_EQ(model.getName(), "Dummy");
}

TEST(setCalibrationFile, invalidFile) {
  string cal_file = ros::package::getPath("mag_manip") + "/test/invalid.yaml";
  BackwardModelLinearThinPlateSplineL2 model;
  EXPECT_THROW(model.setCalibrationFile(cal_file), InvalidFile);
}

TEST(getActuationMatrix, def) {
  string cal_file = ros::package::getPath("mag_manip") + "/test/rbf_valid.yaml";
  BackwardModelLinearThinPlateSplineL2 model;
  model.setCalibrationFile(cal_file);
  PositionVec position(0, 0, 0);
  ActuationMat act_mat = model.getActuationMatrix(position);
  EXPECT_GT(act_mat.norm(), 0);
}

TEST(getFieldActuationMatrix, def) {
  string cal_file = ros::package::getPath("mag_manip") + "/test/rbf_valid.yaml";
  BackwardModelLinearThinPlateSplineL2 model;
  model.setCalibrationFile(cal_file);
  PositionVec position(0, 0, 0);
  ActuationMat field_act_mat = model.getFieldActuationMatrix(position);
  EXPECT_GT(field_act_mat.norm(), 0);
}

TEST(getActuationMatrixInverse, def) {
  string cal_file = ros::package::getPath("mag_manip") + "/test/rbf_valid.yaml";
  BackwardModelLinearThinPlateSplineL2 model;
  model.setCalibrationFile(cal_file);
  PositionVec position(0, 0, 0);
  ActuationMat act_mat = model.getActuationMatrix(position);
  ActuationMat act_mat_inv = model.getActuationMatrixInverse(position);
  // since act_mat has full rank, we can check the following identity
  ActuationMat m = act_mat_inv * act_mat;
  EXPECT_TRUE(m.isIdentity());
}

TEST(getFieldActuationMatrixInverse, def) {
  string cal_file = ros::package::getPath("mag_manip") + "/test/rbf_valid.yaml";
  BackwardModelLinearThinPlateSplineL2 model;
  model.setCalibrationFile(cal_file);
  PositionVec position(0, 0, 0);
  ActuationMat act_mat = model.getFieldActuationMatrix(position);
  ActuationMat act_mat_inv = model.getFieldActuationMatrixInverse(position);
  // act_mat does not have full rank, use the first 2 Moore-Penrose conditions
  ActuationMat m1 = act_mat * act_mat_inv * act_mat;
  ActuationMat m2 = act_mat_inv * act_mat * act_mat_inv;
  EXPECT_TRUE(EIGEN_MATRIX_NEAR_ABS(m1, act_mat, 1e-6));
  EXPECT_TRUE(EIGEN_MATRIX_NEAR_ABS(m2, act_mat_inv, 1e-4));
}

TEST(computeCurrentsFromField, def) {
  string cal_file = ros::package::getPath("mag_manip") + "/test/rbf_gaussian_octomag.yaml";
  BackwardModelLinearThinPlateSplineL2 model;
  model.setCalibrationFile(cal_file);
  PositionVec position(1e-12, 0, 0);
  FieldVec field(30e-3, 0, 0);
  CurrentsVec currents = model.computeCurrentsFromField(position, field);
  // We expect the currents to be larger than 2 A
  EXPECT_GT(currents.norm(), 2);

  ForwardModelLinearThinPlateSpline forward_model;
  forward_model.setCalibrationFile(cal_file);
  FieldVec field_calc = forward_model.computeFieldFromCurrents(position, currents);

  EXPECT_TRUE(EIGEN_MATRIX_NEAR_ABS(field, field_calc, 1e-4));
}

TEST(computeCurrentsFromFieldGradient5, no_field_no_grad) {
  string cal_file = ros::package::getPath("mag_manip") + "/test/rbf_valid.yaml";
  BackwardModelLinearThinPlateSplineL2 model;
  model.setCalibrationFile(cal_file);
  PositionVec position(1e-12, 0, 0);
  FieldVec field(0, 0, 0);
  Gradient5Vec gradient;
  gradient << 0, 0, 0, 0, 0;
  FieldGradient5Vec field_gradient;
  field_gradient << field, gradient;
  CurrentsVec currents = model.computeCurrentsFromFieldGradient5(position, field, gradient);
  EXPECT_NEAR(currents.norm(), 0, 1e-6);
}

TEST(computeCurrentsFromFieldGradient5, field_x_no_grad) {
  string cal_file = ros::package::getPath("mag_manip") + "/test/rbf_valid.yaml";
  BackwardModelLinearThinPlateSplineL2 model;
  model.setCalibrationFile(cal_file);
  PositionVec position(1e-12, 0, 0);
  FieldVec field(30e-3, 0, 0);
  Gradient5Vec gradient;
  gradient << 0, 0, 0, 0, 0;
  FieldGradient5Vec field_gradient;
  field_gradient << field, gradient;
  CurrentsVec currents = model.computeCurrentsFromFieldGradient5(position, field, gradient);
  EXPECT_GT(currents.norm(), 0);
  ActuationMat act_mat_b = model.getActuationMatrix(position);

  ForwardModelLinearThinPlateSpline forward_model;
  forward_model.setCalibrationFile(cal_file);
  FieldGradient5Vec field_gradient_calc =
      forward_model.computeFieldGradient5FromCurrents(position, currents);
  ActuationMat act_mat_f = forward_model.getActuationMatrix(position);
  ASSERT_TRUE(EIGEN_MATRIX_EQUAL(act_mat_b, act_mat_f));

  FieldVec field_calc = field_gradient_calc.head(3);
  Gradient5Vec gradient_calc = field_gradient_calc.tail(5);

  EXPECT_TRUE(EIGEN_MATRIX_NEAR_ABS(field, field_calc, 1e-3));
  EXPECT_TRUE(EIGEN_MATRIX_NEAR_ABS(gradient, gradient_calc, 1e-1));
}

TEST(computeCurrentsFromFieldDipoleGradient3, field_x_grad_x) {
  string cal_file = ros::package::getPath("mag_manip") + "/test/rbf_gaussian_octomag.yaml";
  BackwardModelLinearThinPlateSplineL2 model;
  model.setCalibrationFile(cal_file);
  EXPECT_TRUE(model.isValid());
  EXPECT_EQ(model.getNumCoils(), 8);
  PositionVec position(1e-3, -1e-3, 1e-3);
  FieldVec field(30e-3, 0, 0);
  Gradient3Vec gradient3(0.2, 0, 0);
  DipoleVec dipole(1, 0, 0);
  CurrentsVec currents =
      model.computeCurrentsFromFieldDipoleGradient3(position, field, dipole, gradient3);
  EXPECT_GT(currents.norm(), 0);

  ForwardModelLinearThinPlateSpline forward_model;
  forward_model.setCalibrationFile(cal_file);
  FieldGradient5Vec field_gradient_calc =
      forward_model.computeFieldGradient5FromCurrents(position, currents);
  FieldVec field_calc = field_gradient_calc.head(3);
  Gradient5Vec gradient_calc = field_gradient_calc.tail(5);
  Gradient3Vec gradient3_calc = gradient5VecToGradientMat(gradient_calc) * dipole;

  EXPECT_TRUE(EIGEN_MATRIX_NEAR_ABS(field, field_calc, 1e-3));
  EXPECT_TRUE(EIGEN_MATRIX_NEAR_ABS(gradient3, gradient3_calc, 1e-1));
}

TEST(computeCurrentsFromFieldDipoleGradient3, no_field_no_grad) {
  string cal_file = ros::package::getPath("mag_manip") + "/test/rbf_gaussian_octomag.yaml";
  BackwardModelLinearThinPlateSplineL2 model;
  model.setCalibrationFile(cal_file);
  PositionVec position(1e-12, 0, 0);
  FieldVec field(0, 0, 0);
  Gradient3Vec gradient3(0, 0, 0);
  DipoleVec dipole(1, 0, 0);
  CurrentsVec currents =
      model.computeCurrentsFromFieldDipoleGradient3(position, field, dipole, gradient3);
  EXPECT_NEAR(currents.norm(), 0, 1e-6);
}

TEST(computeCurrentsFromFieldDipoleGradient3, no_field_no_grad_no_dipole) {
  string cal_file = ros::package::getPath("mag_manip") + "/test/rbf_gaussian_octomag.yaml";
  BackwardModelLinearThinPlateSplineL2 model;
  model.setCalibrationFile(cal_file);
  PositionVec position(1e-12, 0, 0);
  FieldVec field(0, 0, 0);
  Gradient3Vec gradient3(0, 0, 0);
  DipoleVec dipole(0, 0, 0);
  CurrentsVec currents =
      model.computeCurrentsFromFieldDipoleGradient3(position, field, dipole, gradient3);
  EXPECT_NEAR(currents.norm(), 0, 1e-6);
}

TEST(factoryCreate, def) {
  BackwardModelFactory f;
  string cal_file = ros::package::getPath("mag_manip") + "/test/rbf_valid.yaml";
  BackwardModel::Ptr p_model = f.create("linear_thinplatespline_L2", cal_file);
  EXPECT_TRUE(p_model->isValid());
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
