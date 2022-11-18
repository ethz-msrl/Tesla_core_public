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
#include <yaml-cpp/yaml.h>
#include "./emns_parameters_mock.h"
#include "mag_manip/emns_parameters_conversions.h"
#include "mag_manip/emns_parameters_rosparam.h"
#include "mag_manip/emns_parameters_yaml.h"
#include "mag_manip/exceptions.h"

using namespace mag_manip;
using namespace std;
using namespace Eigen;

TEST(EMNSParametersToYAML, valid) {
  EMNSParametersMock params;
  auto yaml_str = eMNSParametersToYAML(params);
  EMNSParametersYAML params_(yaml_str);
  EXPECT_EQ(params.getSystemName(), params_.getSystemName());
  EXPECT_EQ(params.getNumElectromagnets(), params_.getNumElectromagnets());
  EXPECT_EQ(params.getMaxPower(), params_.getMaxPower());
  EXPECT_TRUE(EIGEN_MATRIX_EQUAL(params.getMaxCurrents(), params_.getMaxCurrents()));
  EXPECT_TRUE(EIGEN_MATRIX_EQUAL(params.getCoilResistances(), params_.getCoilResistances()));
}

TEST(EMNSParametersToRosparam, valid) {
  EMNSParametersMock params;
  ros::NodeHandle nh;
  eMNSParametersToRosparam(params, nh);
  EMNSParametersRosparam params_(nh);

  EXPECT_EQ(params.getSystemName(), params_.getSystemName());
  EXPECT_EQ(params.getNumElectromagnets(), params_.getNumElectromagnets());
  EXPECT_EQ(params.getMaxPower(), params_.getMaxPower());
  EXPECT_TRUE(EIGEN_MATRIX_EQUAL(params.getMaxCurrents(), params_.getMaxCurrents()));
  EXPECT_TRUE(EIGEN_MATRIX_EQUAL(params.getCoilResistances(), params_.getCoilResistances()));
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "test_emns_parameters_rosparam");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
