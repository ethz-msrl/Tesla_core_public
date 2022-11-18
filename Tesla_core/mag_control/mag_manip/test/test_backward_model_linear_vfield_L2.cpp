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
#include "mag_manip/backward_model_linear_vfield_L2.h"
#include "mag_manip/forward_model_linear_vfield.h"
#include "mag_manip/helpers.h"

using namespace std;
using namespace Eigen;
using namespace mag_manip;

TEST(Constructor, defaultConstructor) {
  BackwardModelLinearVFieldL2 model;
  ASSERT_FALSE(model.isValid());
}

TEST(setInterpolationType, Tricubic) {
  BackwardModelLinearVFieldL2 model;
  string filename = ros::package::getPath("mag_manip") + "/test/vfield_valid.yaml";
  model.setCalibrationFile(filename);
  model.setInterpolationType(InterpolateRegular::Type::TRICUBIC);
  EXPECT_EQ(model.getInterpolationType(), InterpolateRegular::Type::TRICUBIC);

  // we expect the current to be calculated after
  PositionVec position = PositionVec::Zero();
  FieldVec field = FieldVec(20e-3, 0, 0);
  CurrentsVec currents = model.computeCurrentsFromField(position, field);
  EXPECT_GT(currents.norm(), 0);
}

TEST(setInterpolationType, TricubicScalarField) {
  BackwardModelLinearVFieldL2 model;
  string filename = ros::package::getPath("mag_manip") + "/test/vfield_valid.yaml";
  model.setCalibrationFile(filename);
  model.setInterpolationType(InterpolateRegular::Type::TRICUBIC_SCALAR_FIELD);
  EXPECT_EQ(model.getInterpolationType(), InterpolateRegular::Type::TRICUBIC_SCALAR_FIELD);

  // we expect the current to be calculated after
  PositionVec position = PositionVec::Zero();
  FieldVec field = FieldVec(20e-3, 0, 0);
  CurrentsVec currents = model.computeCurrentsFromField(position, field);
  EXPECT_GT(currents.norm(), 0);
}

TEST(setInterpolationType, Trilinear) {
  BackwardModelLinearVFieldL2 model;
  string filename = ros::package::getPath("mag_manip") + "/test/vfield_valid.yaml";
  model.setCalibrationFile(filename);
  model.setInterpolationType(InterpolateRegular::Type::TRILINEAR);
  EXPECT_EQ(model.getInterpolationType(), InterpolateRegular::Type::TRILINEAR);

  // we expect the field to be calculated after
  PositionVec position = PositionVec::Zero();
  FieldVec field = FieldVec(20e-3, 0, 0);
  CurrentsVec currents = model.computeCurrentsFromField(position, field);
  EXPECT_GT(currents.norm(), 0);
}

TEST(setCalibrationFile, Invalid) {
  string filename = ros::package::getPath("mag_manip") + "/test/vfield_valid.yaml";
  BackwardModelLinearVFieldL2 model;
  model.setCalibrationFile(filename);
  EXPECT_TRUE(model.isValid());
  EXPECT_EQ(model.getNumCoils(), 8);
}

TEST(setVFieldFiles, def) {
  BackwardModelLinearVFieldL2 model;

  vector<string> filenames;
  for (int i = 0; i < 8; i++) {
    ostringstream ss;
    ss << ros::package::getPath("mag_manip") << "/test/vfield_0" << i << ".txt";
    filenames.push_back(ss.str());
  }

  model.setVFieldFiles(filenames);

  EXPECT_TRUE(model.isValid());
  EXPECT_EQ(model.getNumCoils(), 8);
  vector<VFieldGridProperties> v_props = model.getVFieldGridProperties();
  ASSERT_EQ(v_props.size(), 8);
  EXPECT_NEAR(v_props[0].min_x, -0.005f, 1e-8);
  EXPECT_NEAR(v_props[0].max_x, 0.005f, 1e-8);
  EXPECT_NEAR(v_props[0].min_y, -0.005f, 1e-8);
  EXPECT_NEAR(v_props[0].max_y, 0.005f, 1e-8);
  EXPECT_NEAR(v_props[0].min_z, -0.005f, 1e-8);
  EXPECT_NEAR(v_props[0].max_z, 0.005f, 1e-8);
  EXPECT_EQ(v_props[0].dim_x, 11);
  EXPECT_EQ(v_props[0].dim_y, 11);
  EXPECT_EQ(v_props[0].dim_z, 11);
  EXPECT_NEAR(v_props[0].step_x, 0.001f, 1e-8);
}

TEST(pointInWorkspace, def) {
  BackwardModelLinearVFieldL2 model;

  vector<string> filenames;
  for (int i = 0; i < 8; i++) {
    ostringstream ss;
    ss << ros::package::getPath("mag_manip") << "/test/vfield_0" << i << ".txt";
    filenames.push_back(ss.str());
  }

  model.setVFieldFiles(filenames);
  PositionVec position(0.0, 0.0, 0.0);
  EXPECT_TRUE(model.pointInWorkspace(position));
}

TEST(getActuationMatrix, def) {
  BackwardModelLinearVFieldL2 model;

  vector<string> filenames;
  for (int i = 0; i < 8; i++) {
    ostringstream ss;
    ss << ros::package::getPath("mag_manip") << "/test/vfield_0" << i << ".txt";
    filenames.push_back(ss.str());
  }

  model.setVFieldFiles(filenames);
  PositionVec position(0, 0, 0);
  ActuationMat act_mat = model.getActuationMatrix(position);
  EXPECT_GT(act_mat.norm(), 0);
}

TEST(computeCurrentsFromField, zeros) {
  // string cal_file = ros::package::getPath("mag_manip") +
  // "/test/C_Mag_Calibration_06-25-2015.yaml";
  BackwardModelLinearVFieldL2 model;

  vector<string> filenames;
  for (int i = 0; i < 8; i++) {
    ostringstream ss;
    ss << ros::package::getPath("mag_manip") << "/test/vfield_0" << i << ".txt";
    filenames.push_back(ss.str());
  }

  model.setVFieldFiles(filenames);
  PositionVec position(0, 0, 0);
  FieldVec field(0, 0, 0);
  CurrentsVec currents = model.computeCurrentsFromField(position, field);
  EXPECT_NEAR(currents.norm(), 0., 1e-6);
}

TEST(computeCurrentsFromField, field_x) {
  BackwardModelLinearVFieldL2 model;

  vector<string> filenames;
  for (int i = 0; i < 8; i++) {
    ostringstream ss;
    ss << ros::package::getPath("mag_manip") << "/test/vfield_0" << i << ".txt";
    filenames.push_back(ss.str());
  }

  model.setVFieldFiles(filenames);
  PositionVec position(0, 0, 0);
  FieldVec field(30e-3, 0, 0);
  CurrentsVec currents = model.computeCurrentsFromField(position, field);
  EXPECT_GT(currents.norm(), 0);

  ForwardModelLinearVField forward_model;
  forward_model.setVFieldFiles(filenames);
  FieldVec field_calc = forward_model.computeFieldFromCurrents(position, currents);

  EXPECT_TRUE(EIGEN_MATRIX_NEAR_ABS(field, field_calc, 1e-3));
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
