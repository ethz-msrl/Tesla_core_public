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

#include "mag_manip/interpolate_trilinear.h"
#include "tsc_utils/eigen_matrix_compare.h"

using namespace mag_manip;
using namespace std;
using namespace Eigen;

TEST(interpolate, single) {
  DataMat mag_fields = DataMat::Zero(3, 27);
  mag_fields.row(0) << 0.950129, 0.444703, 0.410270, 0.485982, 0.921813, 0.352868, 0.456468,
      0.405706, 0.138891, 0.231139, 0.615432, 0.893650, 0.891299, 0.738207, 0.813166, 0.018504,
      0.935470, 0.202765, 0.606843, 0.791937, 0.057891, 0.762097, 0.176266, 0.009861, 0.821407,
      0.916904, 0.198722;
  mag_fields.row(1) << 0.952782, 0.375886, 0.034470, 0.598159, 0.199571, 0.059516, 0.836820,
      0.910235, 0.312344, 0.704062, 0.898596, 0.715335, 0.840743, 0.303095, 0.627099, 0.518703,
      0.525293, 0.522695, 0.953877, 0.429001, 0.768716, 0.442819, 0.538300, 0.265189, 0.022210,
      0.306830, 0.408626;
  mag_fields.row(2) << 0.875158, 0.243687, 0.971638, 0.676502, 0.356813, 0.560920, 0.529079,
      0.992665, 0.287519, 0.317899, 0.842915, 0.218536, 0.071171, 0.232397, 0.733430, 0.171756,
      0.384554, 0.998395, 0.273234, 0.557659, 0.607524, 0.196591, 0.647602, 0.615582, 0.869963,
      0.147839, 0.001178;

  const float min_x = -2;
  const float max_x = 2;
  const float min_y = -2;
  const float max_y = 2;
  const float min_z = -2;
  const float max_z = 2;
  const int dim_x = 3;
  const int dim_y = 3;
  const int dim_z = 3;

  VFieldGridProperties props(min_x, max_x, min_y, max_y, min_z, max_z, dim_x, dim_y, dim_z);
  InterpolateTrilinear interp(mag_fields, props);

  PositionVec p1(0.4962, -1.8490, -0.9525);
  FieldVec field = interp.interpolate(p1);
  FieldVec field_true(0.5166, 0.7570, 0.5259);
  EXPECT_TRUE(EIGEN_MATRIX_NEAR_ABS(field, field_true, 1e-4));
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

  InterpolateTrilinear interp(mag_fields, props);

  PositionVec p(0.15, 0.15, 0.15);
  FieldVec field = interp.interpolate(p);
  FieldVec field_true(0.1250, 0.1250, 0.1250);
  EXPECT_TRUE(EIGEN_MATRIX_NEAR_ABS(field, field_true, 1e-4));
}

TEST(getGradient, single) {
  DataMat mag_fields = DataMat::Zero(3, 27);
  mag_fields.row(0) << 0.950129, 0.444703, 0.410270, 0.485982, 0.921813, 0.352868, 0.456468,
      0.405706, 0.138891, 0.231139, 0.615432, 0.893650, 0.891299, 0.738207, 0.813166, 0.018504,
      0.935470, 0.202765, 0.606843, 0.791937, 0.057891, 0.762097, 0.176266, 0.009861, 0.821407,
      0.916904, 0.198722;
  mag_fields.row(1) << 0.952782, 0.375886, 0.034470, 0.598159, 0.199571, 0.059516, 0.836820,
      0.910235, 0.312344, 0.704062, 0.898596, 0.715335, 0.840743, 0.303095, 0.627099, 0.518703,
      0.525293, 0.522695, 0.953877, 0.429001, 0.768716, 0.442819, 0.538300, 0.265189, 0.022210,
      0.306830, 0.408626;
  mag_fields.row(2) << 0.875158, 0.243687, 0.971638, 0.676502, 0.356813, 0.560920, 0.529079,
      0.992665, 0.287519, 0.317899, 0.842915, 0.218536, 0.071171, 0.232397, 0.733430, 0.171756,
      0.384554, 0.998395, 0.273234, 0.557659, 0.607524, 0.196591, 0.647602, 0.615582, 0.869963,
      0.147839, 0.001178;

  const float min_x = -2;
  const float max_x = 2;
  const float min_y = -2;
  const float max_y = 2;
  const float min_z = -2;
  const float max_z = 2;
  const int dim_x = 3;
  const int dim_y = 3;
  const int dim_z = 3;

  VFieldGridProperties props(min_x, max_x, min_y, max_y, min_z, max_z, dim_x, dim_y, dim_z);
  InterpolateTrilinear interp(mag_fields, props);
  PositionVec p1(0.5, 0.1, 0.1);
  GradientMat gradient = interp.getGradient(p1);
  GradientMat gradient_true;
  gradient_true << -0.3595, -0.2321, -0.2527, -0.1244, -0.1773, -0.2884, -0.2786, -0.0993, -0.3157;
  EXPECT_TRUE(EIGEN_MATRIX_NEAR_ABS(gradient, gradient_true, 1e-4));
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
