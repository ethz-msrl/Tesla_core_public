//
// Tesla - A ROS-based framework for performing magnetic manipulation
//
// Software License Agreement (BSD License)
//
// ©2022 ETH Zurich, Samuel Charreyron, D-​MAVT; Multi-Scale Robotics Lab (MSRL) ; Prof Bradley
// J. Nelson All rights reserved.
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

#pragma once

#include <tensorflow/c/c_api.h>

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace mag_tensorflow {

class Model {
 public:
  typedef std::unique_ptr<TF_Status, void (*)(TF_Status*)> StatusPtr;

  Model(const std::string& model_dir, const std::string& signature_tag,
        const std::string& input_op_name, const std::vector<int64_t>& input_dims,
        const std::string& output_op_name);

  Model(const Model&) = delete;
  void operator=(const Model&) = delete;

  virtual ~Model();

  /**
   * @brief
   *
   * @param input_data: data of the input tensor. Beware that the size must match the input
   * dimensions of the model
   *
   * @return a pair with first element being the dimensions of the tensor output
   * and the second element being the data
   */
  std::pair<std::vector<int>, std::vector<float> > runInput(const std::vector<float>& input_data);

 private:
  static void closedAndDeleteTfSession(TF_Session* p_sess);

  static void checkStatus(TF_Status* p_status, const std::string& doing_what);

  static void deallocator(void* ddata, size_t len, void* arg);

  std::string model_dir_;
  std::string signature_tag_;
  std::string input_op_name_;
  std::vector<int64_t> input_dims_;
  std::string output_op_name_;

  std::unique_ptr<TF_Buffer, void (*)(TF_Buffer*)> p_metagraph_pb_;
  std::unique_ptr<TF_Graph, void (*)(TF_Graph*)> p_graph_;
  std::unique_ptr<TF_SessionOptions, void (*)(TF_SessionOptions*)> p_sess_opts_;

  TF_Session* p_sess_;
};
}  // namespace mag_tensorflow
