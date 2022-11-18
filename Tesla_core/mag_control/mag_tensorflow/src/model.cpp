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

#include "mag_tensorflow/model.h"
#include <algorithm>
#include <cstring>
#include <exception>
#include <iostream>
#include <iterator>
#include <numeric>
#include <sstream>

using namespace mag_tensorflow;

Model::Model(const std::string& model_dir, const std::string& signature_tag,
             const std::string& input_op_name, const std::vector<int64_t>& input_dims,
             const std::string& output_op_name)
    : model_dir_(model_dir),
      signature_tag_(signature_tag),
      input_op_name_(input_op_name),
      input_dims_(input_dims),
      output_op_name_(output_op_name),
      p_metagraph_pb_(TF_NewBuffer(), &TF_DeleteBuffer),
      p_graph_(TF_NewGraph(), &TF_DeleteGraph),
      p_sess_opts_(TF_NewSessionOptions(), &TF_DeleteSessionOptions) {
  auto p_status = StatusPtr(TF_NewStatus(), &TF_DeleteStatus);

  // Right now we support a model with a single tag
  const char* const tags[] = {signature_tag_.c_str()};
  p_sess_ = TF_LoadSessionFromSavedModel(p_sess_opts_.get(), NULL, model_dir_.c_str(), tags, 1,
                                         p_graph_.get(), p_metagraph_pb_.get(), p_status.get());

  checkStatus(p_status.get(), "loading saved model");

  // checking that the input and output operations exist in graph
  if (!TF_GraphOperationByName(p_graph_.get(), input_op_name.c_str())) {
    std::stringstream ss;
    ss << "input operation " << input_op_name << " not found in graph";
    throw std::runtime_error(ss.str());
  }

  if (!TF_GraphOperationByName(p_graph_.get(), output_op_name.c_str())) {
    std::stringstream ss;
    ss << "output operation " << output_op_name << " not found in graph";
    throw std::runtime_error(ss.str());
  }
}

Model::~Model() { closedAndDeleteTfSession(p_sess_); }

void Model::deallocator(void* ddata, size_t len, void* arg) {
  std::free(static_cast<float*>(ddata));
}

std::pair<std::vector<int>, std::vector<float> > Model::runInput(
    const std::vector<float>& input_data) {
  // For now we only support single input and single output
  const int num_in = 1;
  const int num_targets = 1;
  const int num_out = 1;

  // total number of elements
  int num_elements =
      std::accumulate(input_dims_.begin(), input_dims_.end(), 1, std::multiplies<int>());
  if (input_data.size() != num_elements) {
    throw std::runtime_error(
        "The number of elements in the input_data does not match the expected input dimensions");
  }

  void* p_data = malloc(sizeof(float) * num_elements);
  memcpy(p_data, input_data.data(), sizeof(float) * num_elements);

  TF_Tensor* p_input_tensor =
      TF_NewTensor(TF_FLOAT, input_dims_.data(), input_dims_.size(), p_data,
                   num_elements * sizeof(float), &Model::deallocator, nullptr);

  TF_Tensor* in_tensors[1] = {p_input_tensor};

  TF_Output in_op;
  // existance of input operation is already checked at construction
  in_op.oper = TF_GraphOperationByName(p_graph_.get(), input_op_name_.c_str());
  in_op.index = 0;

  TF_Output out_op;
  // existance of output operation is already checked at construction
  out_op.oper = TF_GraphOperationByName(p_graph_.get(), output_op_name_.c_str());
  out_op.index = 0;

  TF_Tensor** p_output_tensors = new TF_Tensor*[1];

  const TF_Operation* out_opers[1] = {out_op.oper};

  auto p_status = StatusPtr(TF_NewStatus(), &TF_DeleteStatus);

  TF_SessionRun(p_sess_, NULL, &in_op, in_tensors, num_in, &out_op, p_output_tensors, num_out,
                out_opers, num_targets, nullptr, p_status.get());

  checkStatus(p_status.get(), "running sesion");

  if (num_out != 1) {
    throw std::runtime_error("Model invalid (cause multiple outputs)");
  }

  float* p_out_data = static_cast<float*>(TF_TensorData(p_output_tensors[0]));

  if (!p_out_data) {
    throw std::runtime_error("output data invalid");
  }

  if (TF_TensorType(p_output_tensors[0]) != TF_FLOAT) {
    throw std::runtime_error("output does not have TF_FLOAT type");
  }

  // get output dims
  int num_out_dims = TF_NumDims(p_output_tensors[0]);
  std::vector<int> out_dims(num_out_dims);

  for (int i = 0; i < num_out_dims; i++) {
    out_dims[i] = TF_Dim(p_output_tensors[0], i);
  }
  // total number of elements
  const int num_out_elements = TF_TensorElementCount(p_output_tensors[0]);

  std::vector<float> v_out(p_out_data, p_out_data + num_out_elements);

  // cleanup
  TF_DeleteTensor(p_input_tensor);
  TF_DeleteTensor(p_output_tensors[0]);

  return {out_dims, v_out};
}

void Model::closedAndDeleteTfSession(TF_Session* p_session) {
  auto p_status = StatusPtr(TF_NewStatus(), &TF_DeleteStatus);

  TF_CloseSession(p_session, p_status.get());
  checkStatus(p_status.get(), "closing session");
  TF_DeleteSession(p_session, p_status.get());
  checkStatus(p_status.get(), "deleting session");
}

void Model::checkStatus(TF_Status* p_status, const std::string& doing_what) {
  if (TF_GetCode(p_status) != TF_OK) {
    std::stringstream ss;
    ss << "Error " << doing_what << std::string(TF_Message(p_status)) << std::endl;
    ss << TF_Message(p_status) << std::endl;
    throw std::runtime_error(ss.str());
  }
}
