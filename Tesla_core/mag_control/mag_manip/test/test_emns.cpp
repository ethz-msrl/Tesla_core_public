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

#include <iostream>

#include "mag_manip/backward_model_mpem_L2.h"
#include "mag_manip/emns.h"
#include "mag_manip/forward_model_mpem.h"

using namespace std;
using namespace mag_manip;

TEST(getMaxFieldAlignedWithTargetField, mpemNoPowerLim) {
  // string cal_file = ros::package::getPath("mag_manip") + "/test/OctoMag_Calibration.yaml";
  string cal_file = ros::package::getPath("mpem") + "/cal/Navion_2_Calibration_24-02-2020.yaml";
  const int NUM_CURRENTS = 3;
  const double MAX_CURRENT = 35.;

  BackwardModelMPEML2 bmodel;
  bmodel.setCalibrationFile(cal_file);

  auto position = PositionVec::Zero();

  auto target_field = FieldVec(0, 0, -20e-3);

  cout << "field pre scaling: " << target_field.transpose() << endl;
  auto currents_target = bmodel.computeCurrentsFromField(position, target_field);
  cout << "currents pre scaling: " << currents_target.transpose() << endl;

  CurrentsVec max_currents = MAX_CURRENT * CurrentsVec::Ones(NUM_CURRENTS);

  auto scaled_field =
      getMaxFieldAlignedWithTargetField(bmodel, position, target_field, max_currents);

  cout << "field post scaling: " << scaled_field.transpose() << endl;

  auto scaled_field_same_dir =
      getMaxFieldAlignedWithTargetField(bmodel, position, FieldVec(0, 0, -10e-3), max_currents);

  // We expect that two parallel field values will get the same result when scaling for the maximum
  // current in that direction
  EXPECT_TRUE(EIGEN_MATRIX_NEAR_ABS(scaled_field, scaled_field_same_dir, 1e-6));

  auto scaled_field_opposite_dir =
      getMaxFieldAlignedWithTargetField(bmodel, position, FieldVec(0, 0, 10e-3), max_currents);

  // We expect that two parallel field values will get the same result when scaling for the maximum
  // current in that direction
  EXPECT_TRUE(EIGEN_MATRIX_NEAR_ABS(scaled_field, -scaled_field_opposite_dir, 1e-6));
}

TEST(getMaxFieldAlignedWithTargetField, mpemPowerLim) {
  // string cal_file = ros::package::getPath("mag_manip") + "/test/OctoMag_Calibration.yaml";
  string cal_file = ros::package::getPath("mpem") + "/cal/Navion_2_Calibration_24-02-2020.yaml";
  const int NUM_CURRENTS = 3;
  const double MAX_CURRENT = 35.;
  const double COIL_RESISTANCES = 7.5;
  double MAX_POWER = 10000;

  BackwardModelMPEML2 bmodel;
  bmodel.setCalibrationFile(cal_file);

  auto position = PositionVec::Zero();

  auto target_field = FieldVec(0, 0, -20e-3);

  cout << "field pre scaling: " << target_field.transpose() << endl;
  auto currents_target = bmodel.computeCurrentsFromField(position, target_field);
  cout << "currents pre scaling: " << currents_target.transpose() << endl;

  CurrentsVec max_currents = MAX_CURRENT * CurrentsVec::Ones(NUM_CURRENTS);
  ResistancesVec coil_resistances = COIL_RESISTANCES * CurrentsVec::Ones(NUM_CURRENTS);

  auto scaled_field = getMaxFieldAlignedWithTargetField(bmodel, position, target_field,
                                                        max_currents, coil_resistances, MAX_POWER);

  cout << "field post scaling: " << scaled_field.transpose() << endl;

  auto scaled_field_same_dir = getMaxFieldAlignedWithTargetField(
      bmodel, position, FieldVec(0, 0, -10e-3), max_currents, coil_resistances, MAX_POWER);

  cout << "field post scaling same dir: " << scaled_field_same_dir.transpose() << endl;

  // We expect that two parallel field values will get the same result when scaling for the maximum
  // current in that direction
  EXPECT_TRUE(EIGEN_MATRIX_NEAR_ABS(scaled_field, scaled_field_same_dir, 1e-6));

  auto scaled_field_opposite_dir = getMaxFieldAlignedWithTargetField(
      bmodel, position, FieldVec(0, 0, 10e-3), max_currents, coil_resistances, MAX_POWER);

  // We expect that two parallel field values will get the same result when scaling for the maximum
  // current in that direction
  EXPECT_TRUE(EIGEN_MATRIX_NEAR_ABS(scaled_field, -scaled_field_opposite_dir, 1e-6));
}

TEST(getMaxFieldMagnitudeAlignedWithTargetField, mpemNoPowerLim) {
  // string cal_file = ros::package::getPath("mag_manip") + "/test/OctoMag_Calibration.yaml";
  string cal_file = ros::package::getPath("mpem") + "/cal/Navion_2_Calibration_24-02-2020.yaml";
  const int NUM_CURRENTS = 3;
  const double MAX_CURRENT = 35.;

  BackwardModelMPEML2 bmodel;
  bmodel.setCalibrationFile(cal_file);

  auto position = PositionVec::Zero();

  auto target_field = FieldVec(0, 0, 20e-3);

  CurrentsVec max_currents = MAX_CURRENT * CurrentsVec::Ones(NUM_CURRENTS);

  auto scaled_field_abs =
      getMaxFieldMagnitudeAlignedWithTargetField(bmodel, position, target_field, max_currents);

  cout << "field magnitude post scaling: " << scaled_field_abs << endl;
}

TEST(getMaxFieldMagnitudeAlignedWithTargetField, mpemPowerLim) {
  // string cal_file = ros::package::getPath("mag_manip") + "/test/OctoMag_Calibration.yaml";
  string cal_file = ros::package::getPath("mpem") + "/cal/Navion_2_Calibration_24-02-2020.yaml";
  const int NUM_CURRENTS = 3;
  const double MAX_CURRENT = 35.;
  const double COIL_RESISTANCES = 7.5;
  double MAX_POWER = 10000;

  BackwardModelMPEML2 bmodel;
  bmodel.setCalibrationFile(cal_file);

  auto position = PositionVec::Zero();

  auto target_field = FieldVec(0, 0, 20e-3);

  CurrentsVec max_currents = MAX_CURRENT * CurrentsVec::Ones(NUM_CURRENTS);
  ResistancesVec coil_resistances = COIL_RESISTANCES * CurrentsVec::Ones(NUM_CURRENTS);

  auto scaled_field_abs = getMaxFieldMagnitudeAlignedWithTargetField(
      bmodel, position, target_field, max_currents, coil_resistances, MAX_POWER);

  cout << "field magnitude post scaling: " << scaled_field_abs << endl;
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
