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
#include <yaml-cpp/yaml.h>
#include <iostream>

#include "mag_manip/vfield.h"

using namespace mag_manip;
using namespace std;
using namespace Eigen;

TEST(parseVFieldFile, validFile) {
  string filename = ros::package::getPath("mag_manip") + "/test/vfield_valid.txt";
  pair<DataMat, VFieldGridProperties> ret = parseVFieldFile(filename);

  EXPECT_EQ(get<1>(ret).dim_x, 11);
}

TEST(pointInVFieldWorkspace, Inside) {
  const float min_x = -0.1;
  const float max_x = 0.1;
  const float min_y = -0.1;
  const float max_y = 0.1;
  const float min_z = -0.1;
  const float max_z = 0.1;
  const int dim_x = 5;
  const int dim_y = 5;
  const int dim_z = 5;
  VFieldGridProperties props1(min_x, max_x, min_y, max_y, min_z, max_z, dim_x, dim_y, dim_z);

  const float min_x_2 = -0.2;
  const float max_x_2 = 0.2;
  const float min_y_2 = -0.2;
  const float max_y_2 = 0.2;
  const float min_z_2 = -0.2;
  const float max_z_2 = 0.2;
  const int dim_x_2 = 5;
  const int dim_y_2 = 5;
  const int dim_z_2 = 5;
  VFieldGridProperties props2(min_x_2, max_x_2, min_y_2, max_y_2, min_z_2, max_z_2, dim_x_2,
                              dim_y_2, dim_z_2);

  vector<VFieldGridProperties> v_props = {props1, props2};

  EXPECT_TRUE(pointInVFieldWorkspace(PositionVec::Zero(), v_props));
}

TEST(pointInVFieldWorkspace, Outside) {
  const float min_x = -0.1;
  const float max_x = 0.1;
  const float min_y = -0.1;
  const float max_y = 0.1;
  const float min_z = -0.1;
  const float max_z = 0.1;
  const int dim_x = 5;
  const int dim_y = 5;
  const int dim_z = 5;
  VFieldGridProperties props1(min_x, max_x, min_y, max_y, min_z, max_z, dim_x, dim_y, dim_z);

  const float min_x_2 = -0.2;
  const float max_x_2 = 0.2;
  const float min_y_2 = -0.2;
  const float max_y_2 = 0.2;
  const float min_z_2 = -0.2;
  const float max_z_2 = 0.2;
  const int dim_x_2 = 5;
  const int dim_y_2 = 5;
  const int dim_z_2 = 5;
  VFieldGridProperties props2(min_x_2, max_x_2, min_y_2, max_y_2, min_z_2, max_z_2, dim_x_2,
                              dim_y_2, dim_z_2);

  vector<VFieldGridProperties> v_props = {props1, props2};

  EXPECT_FALSE(pointInVFieldWorkspace(PositionVec(0.15, 0.15, 0.15), v_props));
}

TEST(parseVFieldYAML, Valid) {
  string filename = ros::package::getPath("mag_manip") + "/test/vfield_single.yaml";
  YAML::Node root = YAML::LoadFile(filename);
  pair<DataMat, VFieldGridProperties> ret = parseVFieldYAML(root);
  DataMat data(get<0>(ret));
  VFieldGridProperties props(get<1>(ret));
  EXPECT_EQ(data.cols(), props.dim_x * props.dim_y * props.dim_z);
  DataMat data_true(3, 8);
  data_true.setOnes();
  EXPECT_TRUE(EIGEN_MATRIX_EQUAL(data, data_true));
}

TEST(getGridPositions, def) {
  const float min_x = -0.1;
  const float max_x = 0.1;
  const float min_y = -0.1;
  const float max_y = 0.1;
  const float min_z = -0.1;
  const float max_z = 0.1;
  const int dim_x = 3;
  const int dim_y = 3;
  const int dim_z = 3;
  VFieldGridProperties props(min_x, max_x, min_y, max_y, min_z, max_z, dim_x, dim_y, dim_z);
  DataMat positions = getGridPositions(props);
  EXPECT_EQ(positions(0, 0), min_x);
  EXPECT_EQ(positions(1, 0), min_y);
  EXPECT_EQ(positions(2, 0), min_z);
  EXPECT_EQ(positions(0, 1), min_x);
  EXPECT_EQ(positions(1, 1), min_y);
  EXPECT_EQ(positions(2, 1), min_z + props.step_z);
  EXPECT_EQ(positions(0, positions.cols() - 1), max_x);
  EXPECT_EQ(positions(1, positions.cols() - 1), max_y);
  EXPECT_EQ(positions(2, positions.cols() - 1), max_z);
}

TEST(printGridProps, def) {
  const float min_x = -0.1;
  const float max_x = 0.1;
  const float min_y = -0.1;
  const float max_y = 0.1;
  const float min_z = -0.1;
  const float max_z = 0.1;
  const int dim_x = 3;
  const int dim_y = 3;
  const int dim_z = 3;
  VFieldGridProperties props(min_x, max_x, min_y, max_y, min_z, max_z, dim_x, dim_y, dim_z);
  std::cout << props << std::endl;
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
