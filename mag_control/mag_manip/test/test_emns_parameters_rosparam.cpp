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
#include <ros/node_handle.h>
#include <ros/package.h>
#include <tsc_utils/eigen_matrix_compare.h>
#include "mag_manip/emns_parameters_rosparam.h"
#include "mag_manip/exceptions.h"

using namespace mag_manip;
using namespace std;
using namespace Eigen;

class RosparamTest : public ::testing::Test {
 public:
  RosparamTest() : nh_(ros::NodeHandle()) {}

  ros::NodeHandle nh_;
};

TEST_F(RosparamTest, getSystemNameValid) {
  EMNSParametersRosparam params(nh_);
  EXPECT_EQ(params.getSystemName(), "dummy");
}

TEST_F(RosparamTest, getNumElectromagnets) {
  EMNSParametersRosparam params(nh_);
  EXPECT_EQ(params.getNumElectromagnets(), 3);
}

TEST_F(RosparamTest, getMaxCurrents) {
  EMNSParametersRosparam params(nh_);
  EXPECT_TRUE(EIGEN_MATRIX_EQUAL(params.getMaxCurrents(), Eigen::Vector3d(8, 8, 8)));
}

TEST_F(RosparamTest, getMaxPower) {
  EMNSParametersRosparam params(nh_);
  EXPECT_EQ(params.getMaxPower(), 15000);
}

TEST_F(RosparamTest, getCoilResistances) {
  EMNSParametersRosparam params(nh_);
  EXPECT_TRUE(EIGEN_MATRIX_EQUAL(params.getCoilResistances(), Eigen::Vector3d(8, 8, 8)));
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "test_emns_parameters_rosparam");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
  ros::shutdown();
}
