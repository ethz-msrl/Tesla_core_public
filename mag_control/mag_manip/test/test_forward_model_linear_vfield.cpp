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
#include <sstream>

#include <gtest/gtest.h>
#include <ros/package.h>
#include <tsc_utils/eigen_matrix_compare.h>

#include "mag_manip/exceptions.h"
#include "mag_manip/forward_model_factory.h"
#include "mag_manip/forward_model_linear_vfield.h"
#include "mag_manip/helpers.h"

using namespace std;
using namespace Eigen;
using namespace mag_manip;

TEST(setInterpolationType, Tricubic) {
  ForwardModelLinearVField model;
  string filename = ros::package::getPath("mag_manip") + "/test/vfield_valid.yaml";
  model.setCalibrationFile(filename);
  model.setInterpolationType(InterpolateRegular::Type::TRICUBIC);
  EXPECT_EQ(model.getInterpolationType(), InterpolateRegular::Type::TRICUBIC);

  // we expect the field to be calculated after
  PositionVec position = PositionVec::Zero();
  CurrentsVec currents = CurrentsVec::Ones(8);
  FieldVec field = model.computeFieldFromCurrents(position, currents);
  EXPECT_GT(field.norm(), 0);
}

TEST(setInterpolationType, TricubicScalarField) {
  ForwardModelLinearVField model;
  string filename = ros::package::getPath("mag_manip") + "/test/vfield_valid.yaml";
  model.setCalibrationFile(filename);
  model.setInterpolationType(InterpolateRegular::Type::TRICUBIC_SCALAR_FIELD);
  EXPECT_EQ(model.getInterpolationType(), InterpolateRegular::Type::TRICUBIC_SCALAR_FIELD);

  // we expect the field to be calculated after
  PositionVec position = PositionVec::Zero();
  CurrentsVec currents = CurrentsVec::Ones(8);
  FieldVec field = model.computeFieldFromCurrents(position, currents);
  EXPECT_GT(field.norm(), 0);
}

TEST(setInterpolationType, Trilinear) {
  string filename = ros::package::getPath("mag_manip") + "/test/vfield_valid.yaml";
  ForwardModelLinearVField model;
  model.setCalibrationFile(filename);
  model.setInterpolationType(InterpolateRegular::Type::TRILINEAR);

  // we expect the field to be calculated after
  PositionVec position = PositionVec::Zero();
  CurrentsVec currents = CurrentsVec::Ones(8);
  FieldVec field = model.computeFieldFromCurrents(position, currents);
  EXPECT_GT(field.norm(), 0);
}

TEST(setCalibrationFile, Valid) {
  string filename = ros::package::getPath("mag_manip") + "/test/vfield_valid.yaml";
  ForwardModelLinearVField model;
  model.setCalibrationFile(filename);
  EXPECT_TRUE(model.isValid());
  EXPECT_EQ(model.getNumCoils(), 8);
}

TEST(setVFieldFiles, Valid) {
  ForwardModelLinearVField model;

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

TEST(computeFieldFromCurrents, 1A) {
  ForwardModelLinearVField model;

  vector<string> filenames;
  for (int i = 0; i < 8; i++) {
    ostringstream ss;
    ss << ros::package::getPath("mag_manip") << "/test/vfield_0" << i << ".txt";
    filenames.push_back(ss.str());
  }

  model.setVFieldFiles(filenames);

  CurrentsVec currents = CurrentsVec::Ones(8);
  PositionVec position = PositionVec::Zero();

  FieldVec field = model.computeFieldFromCurrents(position, currents);
  EXPECT_GT(field.norm(), 0);
}

TEST(computeFieldFromCurrentsCached, 1A) {
  ForwardModelLinearVField model;

  vector<string> filenames;
  for (int i = 0; i < 8; i++) {
    ostringstream ss;
    ss << ros::package::getPath("mag_manip") << "/test/vfield_0" << i << ".txt";
    filenames.push_back(ss.str());
  }

  model.setVFieldFiles(filenames);

  CurrentsVec currents = CurrentsVec::Ones(8);
  PositionVec position = PositionVec::Zero();
  model.setCachedPosition(position);
  FieldVec field = model.computeFieldFromCurrentsCached(currents);
  EXPECT_GT(field.norm(), 0);
}

TEST(computeGradientMatFromCurrents, 1A) {
  ForwardModelLinearVField model;

  vector<string> filenames;
  for (int i = 0; i < 8; i++) {
    ostringstream ss;
    ss << ros::package::getPath("mag_manip") << "/test/vfield_0" << i << ".txt";
    filenames.push_back(ss.str());
  }

  model.setVFieldFiles(filenames);

  const float MAX_CURRENT = 10;

  CurrentsVec currents = MAX_CURRENT * CurrentsVec::Random(8);
  PositionVec position = PositionVec::Zero();

  GradientMat gradient = model.computeGradientMatFromCurrents(position, currents);
  cout << currents.transpose() << endl;
  cout << gradient << endl;
}

TEST(computeGradient5FromCurrentsCached, 1A) {
  ForwardModelLinearVField model;

  vector<string> filenames;
  for (int i = 0; i < 8; i++) {
    ostringstream ss;
    ss << ros::package::getPath("mag_manip") << "/test/vfield_0" << i << ".txt";
    filenames.push_back(ss.str());
  }

  model.setVFieldFiles(filenames);

  CurrentsVec currents = CurrentsVec::Ones(8);
  PositionVec position = PositionVec::Zero();
  model.setCachedPosition(position);
  Gradient5Vec gradient = model.computeGradient5FromCurrentsCached(currents);
  EXPECT_GT(gradient.norm(), 0);
}

TEST(computeFieldGradient5FromCurrentsCached, 1A) {
  ForwardModelLinearVField model;

  vector<string> filenames;
  for (int i = 0; i < 8; i++) {
    ostringstream ss;
    ss << ros::package::getPath("mag_manip") << "/test/vfield_0" << i << ".txt";
    filenames.push_back(ss.str());
  }

  model.setVFieldFiles(filenames);

  CurrentsVec currents = CurrentsVec::Ones(8);
  PositionVec position = PositionVec::Zero();
  model.setCachedPosition(position);
  FieldGradient5Vec field_gradient = model.computeFieldGradient5FromCurrentsCached(currents);
  EXPECT_GT(field_gradient.norm(), 0);
}

TEST(Factory, def) {
  ForwardModelFactory f;
  string filename = ros::package::getPath("mag_manip") + "/test/vfield_valid.yaml";
  ForwardModel::Ptr p_model = f.create("linear_vfield", filename);
  EXPECT_TRUE(p_model->isValid());
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
