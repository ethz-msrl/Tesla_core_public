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
#include <exception>
#include <memory>

#include "mag_manip/forward_model.h"
#include "mag_manip/forward_model_factory.h"
#include "mag_manip/forward_model_linear_rbf.h"
#include "mag_manip/forward_model_linear_saturation.h"
#include "mag_manip/forward_model_linear_thinplatespline.h"
#include "mag_manip/forward_model_linear_vfield.h"
#include "mag_manip/forward_model_mpem.h"
#include "mag_manip/forward_model_saturation.h"

#ifndef NO_TENSORFLOW
#include "mag_manip/forward_model_tensorflow.h"
#endif

namespace mag_manip {
ForwardModel::Ptr ForwardModelFactory::create(const std::string& type,
                                              const std::string& filename) {
  if (type == "mpem") {
    ForwardModel::Ptr p(new ForwardModelMPEM());
    p->setCalibrationFile(filename);
    return p;
  } else if (type == "linear_vfield") {
    ForwardModel::Ptr p(new ForwardModelLinearVField());
    p->setCalibrationFile(filename);
    return p;
  } else if (type == "linear_rbf") {
    ForwardModel::Ptr p(new ForwardModelLinearRBF());
    p->setCalibrationFile(filename);
    return p;
  } else if (type == "linear_thinplatespline") {
    ForwardModel::Ptr p(new ForwardModelLinearThinPlateSpline());
    p->setCalibrationFile(filename);
    return p;
  } else if (type == "saturation") {
    ForwardModel::Ptr p(new ForwardModelSaturation());
    p->setCalibrationFile(filename);
    return p;
  } else if (type == "linear_saturation") {
    ForwardModel::Ptr p(new ForwardModelLinearSaturation());
    p->setCalibrationFile(filename);
    return p;
#ifndef NO_TENSORFLOW
  } else if (type == "tensorflow") {
    ForwardModel::Ptr p(new ForwardModelTensorFlow());
    p->setCalibrationFile(filename);
    return p;
#endif
  } else {
    throw std::invalid_argument("Unrecognized forward model type");
  }
}  // namespace mag_manip
}  // namespace mag_manip
