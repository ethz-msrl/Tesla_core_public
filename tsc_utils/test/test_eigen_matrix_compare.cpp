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

#include <gtest/gtest-spi.h>
#include <gtest/gtest.h>
#include <cmath>
#include "tsc_utils/eigen_matrix_compare.h"

using namespace tesla;

TEST(assertMatricesNear, areEqual) {
  Eigen::Matrix2d A, B;
  A << 1., 2., 3., 4.;
  B << 1., 2., 3., 4.;

  EXPECT_TRUE(EIGEN_MATRIX_EQUAL(A, B));
}

TEST(assertMatricesNear, areDifferent) {
  static Eigen::Matrix2d A, B;
  A << 1., 2., 3., 4.;
  B << 4., 3., 2., 1.;

  EXPECT_FALSE(assertMatricesNear(A, B));
}

TEST(assertMatricesNear, areNear) {
  Eigen::Matrix2d A, B;
  A << 1.000000001, 2., 3., 4.;

  B << 1., 2., 3., 4.;

  EXPECT_TRUE(EIGEN_MATRIX_NEAR_ABS(A, B, 1e-4));
}

TEST(assertMatricesNear, hasNan) {
  Eigen::Matrix2d A, B;
  A << NAN, 2., 3., 4.;
  B << 1., 2., 3., 4.;

  EXPECT_FALSE(EIGEN_MATRIX_NEAR_ABS(A, B, 1e-4));
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
