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

#include "mag_manip/exceptions.h"
#include "mag_manip/forward_model_factory.h"
#include "mag_manip/forward_model_linear_rbf.h"
#include "mag_manip/helpers.h"

using namespace std;
using namespace Eigen;
using namespace mag_manip;

TEST(Constructor, defaultConstructor) {
  ForwardModelLinearRBF model;
  EXPECT_FALSE(model.isValid());
}

TEST(setCalibrationFile, Valid) {
  string filename = ros::package::getPath("mag_manip") + "/test/rbf_valid.yaml";
  ForwardModelLinearRBF model;
  model.setCalibrationFile(filename);
  EXPECT_TRUE(model.isValid());
  EXPECT_EQ(model.getNumCoils(), 3);
}

TEST(getName, def) {
  string cal_file = ros::package::getPath("mag_manip") + "/test/rbf_valid.yaml";
  ForwardModelLinearRBF model;
  model.setCalibrationFile(cal_file);
  EXPECT_EQ(model.getName(), "Dummy");
}

TEST(setCalibrationFile, invalidFile) {
  string cal_file = ros::package::getPath("mag_manip") + "/test/invalid.yaml";
  ForwardModelLinearRBF model;
  EXPECT_THROW(model.setCalibrationFile(cal_file), InvalidFile);
}

TEST(getActuationMatrix, def) {
  string cal_file = ros::package::getPath("mag_manip") + "/test/rbf_valid.yaml";
  ForwardModelLinearRBF model;
  model.setCalibrationFile(cal_file);
  PositionVec position(0, 0, 0);
  ActuationMat act_mat = model.getActuationMatrix(position);
  EXPECT_GT(act_mat.norm(), 0);
}

TEST(computeFieldFromCurrents, zeros) {
  string cal_file = ros::package::getPath("mag_manip") + "/test/rbf_valid.yaml";
  ForwardModelLinearRBF model;
  model.setCalibrationFile(cal_file);
  PositionVec position(0, 0, 0);
  CurrentsVec currents = CurrentsVec(model.getNumCoils());
  currents.setZero();
  ASSERT_TRUE(model.isValid());
  FieldVec field = model.computeFieldFromCurrents(position, currents);
  EXPECT_LT(field.norm(), 1e-3);
}

TEST(computeGradient5FromCurrents, ones) {
  string cal_file = ros::package::getPath("mag_manip") + "/test/rbf_valid.yaml";
  ForwardModelLinearRBF model;
  model.setCalibrationFile(cal_file);
  PositionVec position(0, 0, 0);
  CurrentsVec currents = CurrentsVec(model.getNumCoils());
  currents.setOnes();
  Gradient5Vec gradient = model.computeGradient5FromCurrents(position, currents);
  EXPECT_GT(gradient.norm(), 0);
}

TEST(computeFieldGradient5FromCurrents, ones) {
  string cal_file = ros::package::getPath("mag_manip") + "/test/rbf_valid.yaml";
  ForwardModelLinearRBF model;
  model.setCalibrationFile(cal_file);
  PositionVec position(0, 0, 0);
  CurrentsVec currents = CurrentsVec(model.getNumCoils());
  currents.setOnes();
  FieldGradient5Vec field_gradient = model.computeFieldGradient5FromCurrents(position, currents);
  FieldVec field = field_gradient.head(3);
  Gradient5Vec gradient = field_gradient.tail(5);
  EXPECT_GT(field.norm(), 0);
  EXPECT_GT(gradient.norm(), 0);
}

TEST(factoryCreate, def) {
  ForwardModelFactory f;
  string cal_file = ros::package::getPath("mag_manip") + "/test/rbf_valid.yaml";
  ForwardModel::Ptr p_model = f.create("linear_rbf", cal_file);
  EXPECT_TRUE(p_model->isValid());
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
