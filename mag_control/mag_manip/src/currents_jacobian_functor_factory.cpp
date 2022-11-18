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

#include "mag_manip/currents_jacobian_functor_factory.h"

#include <yaml-cpp/yaml.h>

#include <stdexcept>

#include "mag_manip/forward_model_linear_currents_jacobian_functor.h"
#include "mag_manip/forward_model_linear_factory.h"
#include "mag_manip/forward_model_linear_saturation_currents_jacobian_functor.h"
#include "mag_manip/utils.h"

using namespace std;
using namespace mag_manip;

CurrentsJacobianFunctor::Ptr CurrentsJacobianFunctorFactory::create(const std::string& type,
                                                                    const std::string& filename) {
  CurrentsJacobianFunctor::Ptr p_jac;
  if (type == "forward_model_linear_saturation") {
    auto p_fmls_jac = std::make_shared<ForwardModelLinearSaturationCurrentsJacobianFunctor>();
    auto p_fmls = std::make_shared<ForwardModelLinearSaturation>();
    // this will complain
    p_fmls->setCalibrationFile(filename);
    p_fmls_jac->setForwardModelLinearSaturation(p_fmls);
    p_jac = p_fmls_jac;
    return p_jac;
  } else if (type == "forward_model_linear") {
    auto p_fml_jac = std::make_shared<ForwardModelLinearCurrentsJacobianFunctor>();
    // here we need to retrieve the ForwardModelLinear type from the filename

    YAML::Node config;
    try {
      config = YAML::LoadFile(filename);
    } catch (YAML::BadFile& e) {
      throw InvalidFile(filename, e.what());
    }

    const string parent_dir = getFileDirectory(filename);

    YAML::Node n_fml = config["forward_model_linear"];

    if (!n_fml.IsMap()) {
      throw InvalidCalibration("forward_model_linear is not a map");
    }

    string fml_type;
    try {
      fml_type = n_fml["type"].as<string>();
    } catch (YAML::Exception& e) {
      throw InvalidCalibration("Unable to set forward_model_linear type because " +
                               string(e.what()));
    }

    string fml_fn;
    try {
      fml_fn = n_fml["filename"].as<string>();
    } catch (YAML::Exception& e) {
      throw InvalidCalibration("Unable to set forward_model_linear filename because " +
                               string(e.what()));
    }

    auto p_fml = ForwardModelLinearFactory::create(fml_type, pathAppend(parent_dir, fml_fn));
    p_fml_jac->setForwardModelLinear(p_fml);
    p_jac = p_fml_jac;
    return p_jac;
  } else {
    throw std::invalid_argument("Invalid CurrentsJacobianFunctor type");
  }
}
