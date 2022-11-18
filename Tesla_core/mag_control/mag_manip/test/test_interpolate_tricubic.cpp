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
#include <fstream>

#include "mag_manip/interpolate_tricubic.h"
#include "tsc_utils/eigen_matrix_compare.h"

using namespace mag_manip;
using namespace std;
using namespace Eigen;

TEST(interpolate, outsideBounds) {
  const int NX = 11;
  const int NY = 11;
  const int NZ = 11;
  const float SPACING = 0.01;

  DataMat data(3, NX * NY * NZ);
  data.setOnes();

  const float min_x = 0;
  const float min_y = 0;
  const float min_z = 0;
  const float max_x = (NX - 1) * SPACING;
  const float max_y = (NY - 1) * SPACING;
  const float max_z = (NZ - 1) * SPACING;
  const int dim_x = NX;
  const int dim_y = NY;
  const int dim_z = NZ;
  VFieldGridProperties props(min_x, max_x, min_y, max_y, min_z, max_z, dim_x, dim_y, dim_z);

  InterpolateTricubic interp(data, props);
  PositionVec position(10, 10, 10);
  EXPECT_THROW(interp.interpolate(position), mag_manip::OutsideBounds);
}

TEST(interpolate, constant) {
  const int NX = 11;
  const int NY = 11;
  const int NZ = 11;
  const float SPACING = 0.01;

  DataMat data(3, NX * NY * NZ);
  data.setOnes();

  const float min_x = 0;
  const float min_y = 0;
  const float min_z = 0;
  const float max_x = (NX - 1) * SPACING;
  const float max_y = (NY - 1) * SPACING;
  const float max_z = (NZ - 1) * SPACING;
  const int dim_x = NX;
  const int dim_y = NY;
  const int dim_z = NZ;
  VFieldGridProperties props(min_x, max_x, min_y, max_y, min_z, max_z, dim_x, dim_y, dim_z);

  InterpolateTricubic interp(data, props);
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      for (int k = 0; k < 4; k++) {
        PositionVec position(i * SPACING, j * SPACING, k * SPACING);
        FieldVec field = interp.interpolate(position);
        EXPECT_TRUE(EIGEN_MATRIX_NEAR_ABS(field, FieldVec(1, 1, 1), 1e-6));
      }
    }
  }
}

TEST(interpolate, SingleCube) {
  const float min_x = 0;
  const float min_y = 0;
  const float min_z = 0;
  const float max_x = 0.2;
  const float max_y = 0.2;
  const float max_z = 0.2;
  const int dim_x = 3;
  const int dim_y = 3;
  const int dim_z = 3;
  VFieldGridProperties props(min_x, max_x, min_y, max_y, min_z, max_z, dim_x, dim_y, dim_z);

  DataMat mag_fields = DataMat::Zero(3, props.dim_x * props.dim_y * props.dim_z);
  mag_fields.rightCols<1>() << 1, 1, 1;

  InterpolateTricubic interp(mag_fields, props);

  PositionVec p(0.15, 0.15, 0.15);
  FieldVec field = interp.interpolate(p);
  FieldVec field_true(0.0837, 0.0837, 0.0837);
  EXPECT_TRUE(EIGEN_MATRIX_NEAR_ABS(field, field_true, 1e-4));
}

TEST(interpolate, CompareWithSaved) {
  const float min_x = -0.1;
  const float max_x = 0.1;
  const float min_y = -0.1;
  const float max_y = 0.1;
  const float min_z = -0.1;
  const float max_z = 0.1;
  const int dim_x = 5;
  const int dim_y = 5;
  const int dim_z = 5;
  VFieldGridProperties props(min_x, max_x, min_y, max_y, min_z, max_z, dim_x, dim_y, dim_z);

  DataMat data(3, props.dim_x * props.dim_y * props.dim_z);
  {
    string filename_fields = ros::package::getPath("mag_manip") + "/test/interp_Ng16_0_fields.txt";
    ifstream is(filename_fields, std::ios::in);
    EXPECT_TRUE(is.is_open());
    int idx = 0;
    std::string line;
    while (std::getline(is, line)) {
      std::istringstream ss(line);
      double bx, by, bz;
      ss >> bx;
      ss >> by;
      ss >> bz;
      data.col(idx) = FieldVec(bx, by, bz);
      idx++;
    }
    is.close();
  }

  InterpolateTricubic interp(data, props);

  DataMat pos_ev(3, 16 * 16 * 16);
  {
    string filename_pos_ev = ros::package::getPath("mag_manip") + "/test/interp_Ng16_0_pos_ev.txt";
    ifstream is(filename_pos_ev, std::ios::in);
    EXPECT_TRUE(is.is_open());
    int idx = 0;
    string line;
    while (std::getline(is, line)) {
      std::istringstream ss(line);
      double px, py, pz;
      ss >> px;
      ss >> py;
      ss >> pz;
      pos_ev.col(idx) = PositionVec(px, py, pz);
      idx++;
    }
    is.close();
  }

  DataMat field_true(3, 16 * 16 * 16);
  {
    string filename_pos_ev =
        ros::package::getPath("mag_manip") + "/test/interp_Ng16_0_tri3d_ev.txt";
    ifstream is(filename_pos_ev, std::ios::in);
    EXPECT_TRUE(is.is_open());
    string line;
    int idx = 0;
    while (std::getline(is, line)) {
      std::istringstream ss(line);
      double bx, by, bz;
      ss >> bx;
      ss >> by;
      ss >> bz;
      field_true.col(idx) = FieldVec(bx, by, bz);
      idx++;
    }
  }

  DataMat field_interp(3, field_true.cols());
  for (int i = 0; i < pos_ev.cols(); i++) {
    field_interp.col(i) = interp.interpolate(pos_ev.col(i));
  }

  EXPECT_TRUE(EIGEN_MATRIX_NEAR_ABS(field_interp, field_true, 1e-4));
}

TEST(getGradient, outsideBounds) {
  const float min_x = 0;
  const float min_y = 0;
  const float min_z = 0;
  const float max_x = 0.2;
  const float max_y = 0.2;
  const float max_z = 0.2;
  const int dim_x = 3;
  const int dim_y = 3;
  const int dim_z = 3;
  VFieldGridProperties props(min_x, max_x, min_y, max_y, min_z, max_z, dim_x, dim_y, dim_z);

  DataMat mag_fields = DataMat::Zero(3, props.dim_x * props.dim_y * props.dim_z);
  mag_fields.rightCols<1>() << 1, 1, 1;

  InterpolateTricubic interp(mag_fields, props);
  EXPECT_THROW(interp.getGradient(PositionVec(10, 10, 10)), mag_manip::OutsideBounds);
}

TEST(getGradient, FiniteDiff) {
  const float min_x = 0;
  const float min_y = 0;
  const float min_z = 0;
  const float max_x = 0.2;
  const float max_y = 0.2;
  const float max_z = 0.2;
  const int dim_x = 3;
  const int dim_y = 3;
  const int dim_z = 3;
  VFieldGridProperties props(min_x, max_x, min_y, max_y, min_z, max_z, dim_x, dim_y, dim_z);

  DataMat mag_fields = DataMat::Zero(3, props.dim_x * props.dim_y * props.dim_z);
  mag_fields.rightCols<1>() << 1, 1, 1;

  InterpolateTricubic interp(mag_fields, props);
  const int num_examples = 10;
  PositionVec mid_point((props.max_x - props.min_x) / 2, (props.max_y - props.min_y) / 2,
                        (props.max_z - props.min_z) / 2);

  for (int e = 0; e < num_examples; e++) {
    PositionVec p = mid_point.array() + mid_point.array() * PositionVec::Random().array();
    FieldVec field = interp.interpolate(p);
    GradientMat gradient = interp.getGradient(p);
    GradientMat gradient_fd;

    const float eps = 1e-6;
    GradientMat del_m = eps * GradientMat::Identity();
    for (int i = 0; i < 3; i++) {
      PositionVec p_ = del_m.col(i) + p;
      FieldVec field_ = interp.interpolate(p_);
      gradient_fd.col(i) = (field_ - field) / eps;
    }
    EXPECT_TRUE(EIGEN_MATRIX_NEAR_ABS(gradient, gradient_fd, 1e-3));
  }
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
