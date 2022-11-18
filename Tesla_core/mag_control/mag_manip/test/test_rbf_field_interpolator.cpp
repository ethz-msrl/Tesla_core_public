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
#include <tsc_utils/eigen_matrix_compare.h>
#include <iostream>
#include "mag_manip/rbf_3d_field_interpolator.h"

using namespace mag_manip;
using namespace std;
using namespace Eigen;

template <typename Interpolater>
struct Tester {
  static void getFieldTest() {
    const int N = 125;
    const double eps = 3.0;
    Eigen::MatrixXd nodes(3, N);
    nodes.setRandom();
    Eigen::MatrixXd values(3, N);
    values.setRandom();
    PositionVec position = nodes.col(1);

    Interpolater interp(nodes, values, eps);
    FieldVec field = interp.getField(position);
    EXPECT_TRUE(EIGEN_MATRIX_NEAR_ABS(field, values.col(1), 1e-6));
  }

  static void getGradientTest() {
    const int N = 125;
    const double eps = 3.0;
    Eigen::MatrixXd nodes(3, N);
    nodes.setRandom();
    Eigen::MatrixXd values(3, N);
    values.setRandom();
    // PositionVec position = nodes.col(1).array() + 0.1;
    PositionVec position = nodes.col(1);

    Interpolater interp(nodes, values, eps);
    FieldVec field = interp.getField(position);
    GradientMat gradient = interp.getGradient(position);

    const double delp = 1e-4;
    GradientMat gradient_;
    for (int i = 0; i < 3; i++) {
      PositionVec position_ = position + (Eigen::Matrix3d::Identity() * delp).col(i);
      FieldVec field_ = interp.getField(position_);
      gradient_.col(i) = (field_ - field).array() / delp;
    }

    EXPECT_FALSE(gradient.hasNaN());
    EXPECT_TRUE(EIGEN_MATRIX_NEAR_REL(gradient, gradient_, 1e-1));
  }
};

TEST(RBF3DFieldGaussianInterpolator, getField) {
  Tester<RBF3DFieldGaussianInterpolator>::getFieldTest();
}

TEST(RBF3DFieldGaussianInterpolator, getGradient) {
  Tester<RBF3DFieldGaussianInterpolator>::getGradientTest();
}

TEST(RBF3DFieldMultiquadricInterpolator, getField) {
  Tester<RBF3DFieldMultiquadricInterpolator>::getFieldTest();
}

TEST(RBF3DFieldMultiquadricInterpolator, getGradient) {
  Tester<RBF3DFieldMultiquadricInterpolator>::getGradientTest();
}

TEST(RBF3DFieldInverseMultiquadricInterpolator, getField) {
  Tester<RBF3DFieldInverseMultiquadricInterpolator>::getFieldTest();
}

TEST(RBF3DFieldInverseMultiquadricInterpolator, getGradient) {
  Tester<RBF3DFieldInverseMultiquadricInterpolator>::getGradientTest();
}

TEST(RBF3DFieldCubicInterpolator, getField) { Tester<RBF3DFieldCubicInterpolator>::getFieldTest(); }

TEST(RBF3DFieldCubicInterpolator, getGradient) {
  Tester<RBF3DFieldCubicInterpolator>::getGradientTest();
}

TEST(RBF3DFieldThinPlateSplineInterpolator, getField) {
  Tester<RBF3DFieldThinPlateSplineInterpolator>::getFieldTest();
}

TEST(RBF3DFieldThinPlateSplineInterpolator, getGradient) {
  Tester<RBF3DFieldThinPlateSplineInterpolator>::getGradientTest();
}

TEST(RBF3DFieldInterpolator, factoryCreate) {
  const int N = 125;
  const double eps = 3.0;
  Eigen::MatrixXd nodes(3, N);
  nodes.setRandom();
  Eigen::MatrixXd values(3, N);
  values.setRandom();
  PositionVec position = nodes.col(1);

  auto interp_gaussian = RBF3DFieldInterpolator::create("gaussian", nodes, values, eps);
  EXPECT_NE(dynamic_cast<RBF3DFieldGaussianInterpolator*>(interp_gaussian.get()), nullptr);

  auto interp_mq = RBF3DFieldInterpolator::create("multiquadric", nodes, values, eps);
  EXPECT_NE(dynamic_cast<RBF3DFieldMultiquadricInterpolator*>(interp_mq.get()), nullptr);

  auto interp_imq = RBF3DFieldInterpolator::create("inverse_multiquadric", nodes, values, eps);
  EXPECT_NE(dynamic_cast<RBF3DFieldInverseMultiquadricInterpolator*>(interp_imq.get()), nullptr);

  auto interp_cubic = RBF3DFieldInterpolator::create("cubic", nodes, values, eps);
  EXPECT_NE(dynamic_cast<RBF3DFieldCubicInterpolator*>(interp_cubic.get()), nullptr);

  auto interp_tp = RBF3DFieldInterpolator::create("thinplate", nodes, values, eps);
  EXPECT_NE(dynamic_cast<RBF3DFieldThinPlateSplineInterpolator*>(interp_tp.get()), nullptr);

  EXPECT_ANY_THROW(RBF3DFieldInterpolator::create("invalid", nodes, values, eps));
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
