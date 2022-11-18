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
#include <vector>
#include "mag_manip/forward_model_linear_saturation.h"
#include "mag_manip/forward_model_mpem.h"
#include "mag_manip/saturation_tanh.h"

using namespace mag_manip;
using namespace std;

TEST(setCalibrationFile, def) {
  string cal_file = ros::package::getPath("mag_manip") + "/test/cmag_fmls_v1/params.yaml";
  ForwardModelLinearSaturation model;
  model.setCalibrationFile(cal_file);
  EXPECT_TRUE(model.isValid());
}

TEST(setLinearModel, def) {
  string cal_file = ros::package::getPath("mag_manip") + "/test/OctoMag_Calibration.yaml";

  ForwardModelLinear::Ptr p_lin_model(new ForwardModelMPEM);
  p_lin_model->setCalibrationFile(cal_file);
  ForwardModelLinearSaturation model;
  model.setLinearModel(p_lin_model);

  Eigen::Vector2d params(16.0, 1 / 16.0);
  vector<SaturationFunction::Ptr> sat_functions;
  for (int i = 0; i < model.getNumCoils(); i++) {
    sat_functions.push_back(make_shared<SaturationTanh>(params));
  }
  model.setSaturationFunctions(sat_functions);

  EXPECT_TRUE(model.isValid());
}

TEST(getLinearModel, def) {
  string cal_file = ros::package::getPath("mag_manip") + "/test/OctoMag_Calibration.yaml";

  ForwardModelLinear::Ptr p_lin_model(new ForwardModelMPEM);
  p_lin_model->setCalibrationFile(cal_file);
  ForwardModelLinearSaturation model;
  model.setLinearModel(p_lin_model);

  Eigen::Vector2d params(16.0, 1 / 16.0);
  vector<SaturationFunction::Ptr> sat_functions;
  for (int i = 0; i < model.getNumCoils(); i++) {
    sat_functions.push_back(make_shared<SaturationTanh>(params));
  }
  model.setSaturationFunctions(sat_functions);
  auto p_lin_model_ = model.getLinearModel();

  EXPECT_EQ(&*p_lin_model_, &*p_lin_model);
}

TEST(setLinearModel, isLinear) {
  string cal_file = ros::package::getPath("mag_manip") + "/test/OctoMag_Calibration.yaml";

  ForwardModelLinear::Ptr p_lin_model(new ForwardModelMPEM);
  p_lin_model->setCalibrationFile(cal_file);
  ForwardModelLinearSaturation model;
  model.setModel(static_pointer_cast<ForwardModel>(p_lin_model));

  Eigen::Vector2d params(16.0, 1 / 16.0);
  vector<SaturationFunction::Ptr> sat_functions;
  for (int i = 0; i < model.getNumCoils(); i++) {
    sat_functions.push_back(make_shared<SaturationTanh>(params));
  }
  model.setSaturationFunctions(sat_functions);
  EXPECT_TRUE(model.isValid());
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
