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
#include "tsc_utils/eigen_tensor_compare.h"

using namespace tesla;
using Tensor3d = Eigen::Tensor<double, 3>;
using Tensor2d = Eigen::Tensor<double, 2>;

TEST(assertTensors2Near, areEqual) {
  Tensor2d t_a(2, 3);
  Tensor2d t_b(2, 3);

  t_a.setConstant(2.0);
  t_b.setConstant(2.0);

  EXPECT_TRUE(assertTensors2Near(t_a, t_b));
}

TEST(assertTensors2Near, areDifferent) {
  Tensor2d t_a(2, 3);
  Tensor2d t_b(2, 3);

  t_a.setConstant(2.0);
  t_b.setConstant(1.0);

  EXPECT_FALSE(assertTensors2Near(t_a, t_b));
}

TEST(assertTensors2Near, areNear) {
  Tensor2d t_a(2, 3);
  Tensor2d t_b(2, 3);

  Tensor2d d(2, 3);
  t_b = t_a + d.setRandom() * 1e-6;

  EXPECT_TRUE(EIGEN_TENSOR2_NEAR_ABS(t_a, t_b, 1e-4));
}

TEST(assertTensors2Near, hasNan) {
  Tensor2d t_a(2, 3);
  Tensor2d t_b(2, 3);

  t_a.setConstant(NAN);
  t_b.setConstant(1.0);

  EXPECT_FALSE(EIGEN_TENSOR2_NEAR_ABS(t_a, t_b, 1e-4));
}

TEST(assertTensors3Near, areEqual) {
  Tensor3d t_a(2, 3, 4);
  Tensor3d t_b(2, 3, 4);

  t_a.setConstant(2.0);
  t_b.setConstant(2.0);

  EXPECT_TRUE(assertTensors3Near(t_a, t_b));
}

TEST(assertTensors3Near, areDifferent) {
  Tensor3d t_a(2, 3, 4);
  Tensor3d t_b(2, 3, 4);

  t_a.setConstant(2.0);
  t_b.setConstant(1.0);

  EXPECT_FALSE(assertTensors3Near(t_a, t_b));
}

TEST(assertTensors3Near, areNear) {
  Tensor3d t_a(2, 3, 4);
  Tensor3d t_b(2, 3, 4);

  t_a.setConstant(2.0);

  Tensor3d d(2, 3, 4);

  t_b = t_a + d.setRandom() * 1e-6;

  EXPECT_TRUE(EIGEN_TENSOR3_NEAR_ABS(t_a, t_b, 1e-4));
}

TEST(assertTensors3Near, hasNan) {
  Tensor3d t_a(2, 3, 4);
  Tensor3d t_b(2, 3, 4);

  t_a.setConstant(NAN);
  t_b.setConstant(1.0);

  EXPECT_FALSE(EIGEN_TENSOR3_NEAR_ABS(t_a, t_b, 1e-4));
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
