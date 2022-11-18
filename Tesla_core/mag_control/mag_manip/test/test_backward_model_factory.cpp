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

#include "mag_manip/backward_model_factory.h"

using namespace mag_manip;
using namespace std;

TEST(create, mpem_L2) {
  string cal_file = ros::package::getPath("mag_manip") + "/test/OctoMag_Calibration.yaml";
  BackwardModelFactory f;
  auto model = f.create("mpem_L2", cal_file);
  EXPECT_TRUE(model->isValid());
}

TEST(create, linear_vfield_L2) {
  string cal_file = ros::package::getPath("mag_manip") + "/test/vfield_valid.yaml";
  BackwardModelFactory f;
  auto model = f.create("linear_vfield_L2", cal_file);
  EXPECT_TRUE(model->isValid());
}

TEST(create, empty_type) {
  string cal_file = ros::package::getPath("mag_manip") + "/test/vfield_valid.yaml";
  BackwardModelFactory f;
  EXPECT_THROW(f.create("", cal_file), std::invalid_argument);
}

TEST(create, saturation) {
  string cal_file = ros::package::getPath("mag_manip") + "/test/cmag_bml2s_v1/params.yaml";
  BackwardModelFactory f;
  auto model = f.create("saturation", cal_file);
  EXPECT_TRUE(model->isValid());
}

TEST(create, nls) {
  string cal_file = ros::package::getPath("mag_manip") + "/test/cmag_bmnlsmpem_v1/params.yaml";
  BackwardModelFactory f;
  auto model = f.create("nls", cal_file);
  EXPECT_TRUE(model->isValid());
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
