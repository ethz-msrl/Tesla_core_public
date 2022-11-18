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
#include "mag_manip/saturation_tanh.h"
#include "tsc_utils/eigen_matrix_compare.h"

using namespace mag_manip;

TEST(evaluate, basic) {
  Eigen::Vector2d params(1.0, 2.0);
  SaturationTanh sat(params);
  double y = sat.evaluate(0.5);
  EXPECT_NEAR(0.761594, y, 1e-4);
}

TEST(jacobian, basic) {
  Eigen::Vector2d params(1.0, 2.0);
  SaturationTanh sat(params);
  const double x = 0.5;
  Eigen::Vector3d jacobian = sat.jacobian(x);
  double eps = 1e-6;
  double ds_dx = (sat.evaluate(x + eps) - sat.evaluate(x)) / eps;
  EXPECT_NEAR(ds_dx, jacobian(0), 1e-4);

  SaturationTanh sat_d1(Eigen::Vector2d(1.0 + eps, 2.0));
  double ds_dp1 = (sat_d1.evaluate(x) - sat.evaluate(x)) / eps;
  EXPECT_NEAR(ds_dp1, jacobian(1), 1e-4);

  SaturationTanh sat_d2(Eigen::Vector2d(1.0, 2.0 + eps));
  double ds_dp2 = (sat_d2.evaluate(x) - sat.evaluate(x)) / eps;
  EXPECT_NEAR(ds_dp2, jacobian(2), 1e-4);
}

TEST(inverse, basic) {
  Eigen::Vector2d params(1.0, 2.0);
  SaturationTanh sat(params);
  double y = sat.evaluate(0.5);
  EXPECT_NEAR(0.5, sat.inverse(y), 1e-6);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
