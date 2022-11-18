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
#include "mag_manip/forward_model_tensorflow.h"
#include "mag_manip/vfield.h"

#include <iostream>

using namespace mag_manip;
using namespace std;

TEST(Constructor, defaultConstructor) {
  ForwardModelTensorFlow model;
  ASSERT_FALSE(model.isValid());
}

TEST(setCalibrationFile, cmag_file) {
  ForwardModelTensorFlow model;
  string filename = ros::package::getPath("mag_tensorflow") + "/models/cmag_cnn_v1/params.yaml";
  model.setCalibrationFile(filename);
  EXPECT_TRUE(model.isValid());

  EXPECT_EQ(model.getNumCoils(), 8);
  EXPECT_EQ(model.getName(), "cmag_cnn_v1");
  EXPECT_EQ(model.getOutputGridProperties().dim_x, 16);
  EXPECT_EQ(model.getOutputGridProperties().dim_y, 16);
  EXPECT_EQ(model.getOutputGridProperties().dim_z, 16);
}

TEST(computeFieldFromCurrents, zeros) {
  ForwardModelTensorFlow model;
  string filename = ros::package::getPath("mag_tensorflow") + "/models/cmag_cnn_v1/params.yaml";
  model.setCalibrationFile(filename);
  VFieldGridProperties grid_props = model.getOutputGridProperties();
  // picking a position in the middle of the workspace
  PositionVec position((grid_props.min_x + 7 * grid_props.step_x),
                       (grid_props.min_y + 7 * grid_props.step_y),
                       (grid_props.min_z + 7 * grid_props.step_z));
  CurrentsVec currents = CurrentsVec::Zero(8);

  FieldVec field = model.computeFieldFromCurrents(position, currents);
  std::cout << field.transpose() << std::endl;
}

TEST(computeFieldsFromCurrents, random_currents) {
  ForwardModelTensorFlow model;
  string filename = ros::package::getPath("mag_tensorflow") + "/models/cmag_cnn_v1/params.yaml";
  model.setCalibrationFile(filename);
  VFieldGridProperties grid_props = model.getOutputGridProperties();
  VFieldGridProperties grid_props_eval(grid_props.min_x + 0.01, grid_props.max_x - 0.01,
                                       grid_props.min_y + 0.01, grid_props.max_y - 0.01,
                                       grid_props.min_z + 0.01, grid_props.max_z - 0.01, 3, 3, 3);
  PositionVecs positions = getGridPositions(grid_props_eval);
  // picking positions equally spaced over the workspace
  CurrentsVec currents = CurrentsVec::Random(8) * 8;
  FieldVecs fields = model.computeFieldsFromCurrents(positions, currents);

  FieldVecs fields_(3, positions.cols());
  for (int i = 0; i < positions.cols(); i++) {
    fields_.col(i) = model.computeFieldFromCurrents(positions.col(i), currents);
  }

  EXPECT_TRUE(EIGEN_MATRIX_NEAR_ABS(fields, fields_, 1e-6));
}

// TEST(computeFieldFromCurrents, zerosRBF) {
//  ForwardModelTensorFlow model;
//  string filename = ros::package::getPath("mag_tensorflow") + "/models/cmag_cnn_v1/params.yaml";
//  model.setCalibrationFile(filename);
//  VFieldGridProperties grid_props = model.getOutputGridProperties();
//  // picking a position in the middle of the workspace
//  PositionVec position((grid_props.min_x + grid_props.max_x)/2, (grid_props.min_y +
//  grid_props.max_y)/2, (grid_props.min_z + grid_props.max_z)/2); CurrentsVec currents =
//  CurrentsVec::Zero(8);
//
//  FieldVec field = model.computeFieldFromCurrentsRBF(position, currents);
//  std::cout << field.transpose() << std::endl;
//}

TEST(computeFieldFromCurrents, max_current) {
  ForwardModelTensorFlow model;
  string filename = ros::package::getPath("mag_tensorflow") + "/models/cmag_cnn_v1/params.yaml";
  model.setCalibrationFile(filename);
  VFieldGridProperties grid_props = model.getOutputGridProperties();
  // picking a position in the middle of the workspace
  PositionVec position((grid_props.min_x + grid_props.max_x) / 2,
                       (grid_props.min_y + grid_props.max_y) / 2,
                       (grid_props.min_z + grid_props.max_z) / 2);
  CurrentsVec currents = CurrentsVec::Zero(8);
  currents(3) = 35;

  FieldVec field = model.computeFieldFromCurrents(position, currents);
  std::cout << field.transpose() << std::endl;
}

TEST(computeGradient5FromCurrents, zeros) {
  ForwardModelTensorFlow model;
  string filename = ros::package::getPath("mag_tensorflow") + "/models/cmag_cnn_v1/params.yaml";
  model.setCalibrationFile(filename);
  VFieldGridProperties grid_props = model.getOutputGridProperties();
  // picking a position in the middle of the workspace
  PositionVec position((grid_props.min_x + 7 * grid_props.step_x),
                       (grid_props.min_y + 7 * grid_props.step_y),
                       (grid_props.min_z + 7 * grid_props.step_z));
  // PositionVec position((grid_props.min_x + grid_props.max_x)/2, (grid_props.min_y +
  // grid_props.max_y)/2, (grid_props.min_z + grid_props.max_z)/2);
  CurrentsVec currents = CurrentsVec::Zero(8);

  Gradient5Vec gradient = model.computeGradient5FromCurrents(position, currents);
  std::cout << gradient.transpose() << std::endl;
}

TEST(computeGradient5sFromCurrents, random_currents) {
  ForwardModelTensorFlow model;
  string filename = ros::package::getPath("mag_tensorflow") + "/models/cmag_cnn_v1/params.yaml";
  model.setCalibrationFile(filename);
  VFieldGridProperties grid_props = model.getOutputGridProperties();
  VFieldGridProperties grid_props_eval(grid_props.min_x + 0.01, grid_props.max_x - 0.01,
                                       grid_props.min_y + 0.01, grid_props.max_y - 0.01,
                                       grid_props.min_z + 0.01, grid_props.max_z - 0.01, 3, 3, 3);
  PositionVecs positions = getGridPositions(grid_props_eval);
  // picking positions equally spaced over the workspace
  CurrentsVec currents = CurrentsVec::Random(8) * 8;
  Gradient5Vecs gradients = model.computeGradient5sFromCurrents(positions, currents);

  Gradient5Vecs gradients_(5, positions.cols());
  for (int i = 0; i < positions.cols(); i++) {
    gradients_.col(i) = model.computeGradient5FromCurrents(positions.col(i), currents);
  }

  EXPECT_TRUE(EIGEN_MATRIX_NEAR_ABS(gradients, gradients_, 1e-6));
}

TEST(computeFieldGradient5FromCurrents, zeros) {
  ForwardModelTensorFlow model;
  string filename = ros::package::getPath("mag_tensorflow") + "/models/cmag_cnn_v1/params.yaml";
  model.setCalibrationFile(filename);
  VFieldGridProperties grid_props = model.getOutputGridProperties();
  // picking a position in the middle of the workspace
  PositionVec position((grid_props.min_x + 7 * grid_props.step_x),
                       (grid_props.min_y + 7 * grid_props.step_y),
                       (grid_props.min_z + 7 * grid_props.step_z));
  // PositionVec position((grid_props.min_x + grid_props.max_x)/2, (grid_props.min_y +
  // grid_props.max_y)/2, (grid_props.min_z + grid_props.max_z)/2);
  CurrentsVec currents = CurrentsVec::Zero(8);

  FieldGradient5Vec field_gradient = model.computeFieldGradient5FromCurrents(position, currents);
  std::cout << field_gradient.transpose() << std::endl;
}

TEST(computeFieldGradient5FromCurrents, max_current) {
  ForwardModelTensorFlow model;
  string filename = ros::package::getPath("mag_tensorflow") + "/models/cmag_cnn_v1/params.yaml";
  model.setCalibrationFile(filename);
  VFieldGridProperties grid_props = model.getOutputGridProperties();
  // picking a position in the middle of the workspace
  PositionVec position((grid_props.min_x + grid_props.max_x) / 2,
                       (grid_props.min_y + grid_props.max_y) / 2,
                       (grid_props.min_z + grid_props.max_z) / 2);
  CurrentsVec currents = CurrentsVec::Zero(8);
  currents(3) = 35;

  FieldGradient5Vec field_gradient = model.computeFieldGradient5FromCurrents(position, currents);
  std::cout << field_gradient.transpose() << std::endl;
}

TEST(computeFieldGradient5sFromCurrents, random_currents) {
  ForwardModelTensorFlow model;
  string filename = ros::package::getPath("mag_tensorflow") + "/models/cmag_cnn_v1/params.yaml";
  model.setCalibrationFile(filename);
  VFieldGridProperties grid_props = model.getOutputGridProperties();
  VFieldGridProperties grid_props_eval(grid_props.min_x + 0.01, grid_props.max_x - 0.01,
                                       grid_props.min_y + 0.01, grid_props.max_y - 0.01,
                                       grid_props.min_z + 0.01, grid_props.max_z - 0.01, 3, 3, 3);
  PositionVecs positions = getGridPositions(grid_props_eval);
  // picking positions equally spaced over the workspace
  CurrentsVec currents = CurrentsVec::Random(8) * 8;
  FieldGradient5Vecs field_gradients =
      model.computeFieldGradient5sFromCurrents(positions, currents);

  FieldGradient5Vecs field_gradients_(8, positions.cols());
  for (int i = 0; i < positions.cols(); i++) {
    field_gradients_.col(i) = model.computeFieldGradient5sFromCurrents(positions.col(i), currents);
  }

  EXPECT_TRUE(EIGEN_MATRIX_NEAR_ABS(field_gradients, field_gradients_, 1e-6));
}

TEST(factoryCreate, def) {
  ForwardModelFactory f;
  string filename = ros::package::getPath("mag_tensorflow") + "/models/cmag_cnn_v1/params.yaml";
  ForwardModel::Ptr p_model = f.create("tensorflow", filename);
  EXPECT_TRUE(p_model->isValid());
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
