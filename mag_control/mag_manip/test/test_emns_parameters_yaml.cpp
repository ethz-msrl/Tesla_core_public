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
#include "mag_manip/emns_parameters_yaml.h"
#include "mag_manip/exceptions.h"

using namespace mag_manip;
using namespace std;
using namespace Eigen;

TEST(fromFile, valid) {
  string filename = ros::package::getPath("mag_manip") + "/test/emns_params_valid.yaml";
  auto params = EMNSParametersYAML::fromFile(filename);

  EXPECT_EQ(params.getSystemName(), "dummy");
  EXPECT_EQ(params.getNumElectromagnets(), 3);
  EXPECT_TRUE(EIGEN_MATRIX_EQUAL(params.getMaxCurrents(), Eigen::Vector3d(8, 8, 8)));
  EXPECT_EQ(params.getMaxPower(), 15000);
  EXPECT_EQ(params.getCoilResistances(), Eigen::Vector3d(8, 8, 8));
}

TEST(ptrFromFile, valid) {
  string filename = ros::package::getPath("mag_manip") + "/test/emns_params_valid.yaml";
  EMNSParametersYAML::Ptr p_params = EMNSParametersYAML::ptrFromFile(filename);

  EXPECT_EQ(p_params->getNumElectromagnets(), 3);
  EXPECT_TRUE(EIGEN_MATRIX_EQUAL(p_params->getMaxCurrents(), Eigen::Vector3d(8, 8, 8)));
  EXPECT_EQ(p_params->getMaxPower(), 15000);
  EXPECT_EQ(p_params->getCoilResistances(), Eigen::Vector3d(8, 8, 8));
}

TEST(load, valid) {
  std::string config_s =
      " { system_name: dummy, "
      "num_electromagnets: 3, "
      "max_currents: [8, 8, 8], "
      "max_power: 15000, "
      "coil_resistances: [8, 8, 8] } ";

  EMNSParametersYAML params(config_s);

  EXPECT_EQ(params.getSystemName(), "dummy");
  EXPECT_EQ(params.getNumElectromagnets(), 3);
  EXPECT_TRUE(EIGEN_MATRIX_EQUAL(params.getMaxCurrents(), Eigen::Vector3d(8, 8, 8)));
  EXPECT_EQ(params.getMaxPower(), 15000);
  EXPECT_EQ(params.getCoilResistances(), Eigen::Vector3d(8, 8, 8));
}

TEST(load, noSystemName) {
  std::string config_s =
      " { "
      "num_electromagnets: 3, "
      "max_currents: [8, 8, 8], "
      "max_power: 15000, "
      "coil_resistances: [8, 8, 8] } ";

  EXPECT_THROW(EMNSParametersYAML params(config_s), InvalidCalibration);
}

TEST(load, noNumElectromagnets) {
  std::string config_s =
      " { system_name: dummy, "
      "max_currents: [8, 8, 8], "
      "max_power: 15000, "
      "coil_resistances: [8, 8, 8] } ";

  EXPECT_THROW(EMNSParametersYAML params(config_s), InvalidCalibration);
}

TEST(load, noMaxCurrents) {
  std::string config_s =
      " { system_name: dummy, "
      "num_electromagnets: 3, "
      "max_power: 15000, "
      "coil_resistances: [8, 8, 8] } ";

  EXPECT_THROW(EMNSParametersYAML params(config_s), InvalidCalibration);
}

TEST(load, maxCurrentsWrongNumber) {
  std::string config_s =
      " { system_name: dummy, "
      "num_electromagnets: 3, "
      "max_power: 15000, "
      "max_currents: [8, 8, 8, 8], "
      "coil_resistances: [8, 8, 8] } ";

  EXPECT_THROW(EMNSParametersYAML params(config_s), InvalidCalibration);
}

TEST(load, maxCurrentsNegative) {
  std::string config_s =
      " { system_name: dummy, "
      "num_electromagnets: 3, "
      "max_power: 15000, "
      "max_currents: [8, 8, -8], "
      "coil_resistances: [8, 8, 8] } ";

  EXPECT_THROW(EMNSParametersYAML params(config_s), InvalidCalibration);
}

TEST(load, noMaxPower) {
  std::string config_s =
      " { system_name: dummy, "
      "num_electromagnets: 3, "
      "max_currents: [8, 8, 8], "
      "coil_resistances: [8, 8, 8] } ";

  EXPECT_THROW(EMNSParametersYAML params(config_s), InvalidCalibration);
}

TEST(load, max_power) {
  std::string config_s =
      " { system_name: dummy, "
      "num_electromagnets: 3, "
      "max_power: -15000, "
      "max_currents: [8, 8, 8], "
      "coil_resistances: [8, 8, 8] } ";

  EXPECT_THROW(EMNSParametersYAML params(config_s), InvalidCalibration);
}

TEST(load, noCoilResistances) {
  std::string config_s =
      " { system_name: dummy, "
      "num_electromagnets: 3, "
      "max_currents: [8, 8, 8], "
      "max_power: 15000 } ";

  EXPECT_THROW(EMNSParametersYAML params(config_s), InvalidCalibration);
}

TEST(load, coilResistancesWrongNumber) {
  std::string config_s =
      " { system_name: dummy, "
      "num_electromagnets: 3, "
      "max_currents: [8, 8, 8], "
      "max_power: 15000, "
      "coilResistances: [8, 8, 8, 8] } ";

  EXPECT_THROW(EMNSParametersYAML params(config_s), InvalidCalibration);
}

TEST(load, coilResistancesNegative) {
  std::string config_s =
      " { system_name: dummy, "
      "num_electromagnets: 3, "
      "max_currents: [8, 8, 8], "
      "max_power: 15000, "
      "coilResistances: [8, 8, -8] } ";

  EXPECT_THROW(EMNSParametersYAML params(config_s), InvalidCalibration);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
