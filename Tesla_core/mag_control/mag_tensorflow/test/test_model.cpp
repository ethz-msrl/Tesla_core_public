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
#include <fstream>
#include "mag_tensorflow/model.h"

using namespace std;
using namespace mag_tensorflow;

class ModelTest : public ::testing::Test {
 protected:
  // You can remove any or all of the following functions if their bodies would
  // be empty.

  ModelTest() {
    string model_dir = ros::package::getPath("mag_tensorflow") + "/models/cmag_cnn_v1";
    string signature_tag = "serve";
    string input_op_name = "Placeholder";
    vector<int64_t> input_dims = {1, 8};
    string output_op_name = "G/13_conv/BiasAdd";
    // vector<int64_t> output_dims = {16, 16, 16, 3};
    p_model_ = unique_ptr<Model>(
        new Model(model_dir, signature_tag, input_op_name, input_dims, output_op_name));
  }

  ~ModelTest() override {
    // You can do clean-up work that doesn't throw exceptions here.
  }

  // Class members declared here can be used by all tests in the test suite
  // for Foo.
  std::unique_ptr<Model> p_model_;
};

TEST(Model, creation) {
  string model_dir = ros::package::getPath("mag_tensorflow") + "/models/cmag_cnn_v1";
  string signature_tag = "serve";
  string input_op_name = "Placeholder";
  vector<int64_t> input_dims = {1, 8};
  string output_op_name = "G/13_conv/BiasAdd";

  EXPECT_NO_THROW(Model(model_dir, signature_tag, input_op_name, input_dims, output_op_name));
}

TEST_F(ModelTest, runInputZeros) {
  std::vector<float> input = {0., 0., 0., 0., 0., 0., 0., 0.};
  std::pair<std::vector<int>, std::vector<float> > output = p_model_->runInput(input);
  std::vector<int> out_size = {1, 16, 16, 16, 3};
  EXPECT_EQ(output.first, out_size);
  EXPECT_EQ(output.second.size(), 16 * 16 * 16 * 3);
}

TEST_F(ModelTest, runInputCompareWithPython) {
  std::ifstream is(ros::package::getPath("mag_tensorflow") + "/test/currents.txt", std::ios::in);
  ASSERT_TRUE(is.is_open());

  std::vector<float> currents;
  std::string s;
  while (std::getline(is, s)) {
    currents.push_back(std::stof(s));
  }
  is.close();

  EXPECT_EQ(currents.size(), 8);

  is.open(ros::package::getPath("mag_tensorflow") + "/test/fields.txt", std::ios::in);
  ASSERT_TRUE(is.is_open());

  std::vector<float> fields;
  while (std::getline(is, s)) {
    fields.push_back(std::stof(s));
  }
  is.close();

  ASSERT_EQ(fields.size(), 16 * 16 * 16 * 3);

  auto output = p_model_->runInput(currents);
  ASSERT_EQ(output.second.size(), 16 * 16 * 16 * 3);

  for (int i = 0; i < fields.size(); i++) {
    EXPECT_NEAR(output.second[i], fields[i], 1e-4);
  }
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
