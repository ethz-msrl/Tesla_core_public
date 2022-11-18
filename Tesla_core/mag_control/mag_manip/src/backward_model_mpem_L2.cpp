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

#include "mag_manip/backward_model_mpem_L2.h"
#include "mag_manip/exceptions.h"

namespace mag_manip {
BackwardModelMPEML2::BackwardModelMPEML2() : cal_filename_("") {}

bool BackwardModelMPEML2::isValid() const { return p_mpem_cal_.get() != nullptr; }

int BackwardModelMPEML2::getNumCoils() const {
  if (!p_mpem_cal_) {
    throw InvalidCalibration("Calibration not set");
  }

  return p_mpem_cal_->getNumberOfCoils();
}

std::string BackwardModelMPEML2::getName() const {
  if (!p_mpem_cal_) {
    throw InvalidCalibration("Calibration not set");
  }

  return p_mpem_cal_->getName();
}

bool BackwardModelMPEML2::pointInWorkspace(const PositionVec& position) const {
  if (!p_mpem_cal_) {
    throw InvalidCalibration("Calibration not set");
  }

  return p_mpem_cal_->pointInWorkspace(position);
}

void BackwardModelMPEML2::setCalibrationFile(const std::string& filename) {
  try {
    p_mpem_cal_.reset(new ElectromagnetCalibration(filename));
  } catch (std::exception& e) {
    throw InvalidFile(filename, e.what());
  }
  cal_filename_ = filename;
}

ActuationMat BackwardModelMPEML2::getActuationMatrix(const PositionVec& position) const {
  if (!p_mpem_cal_) {
    throw InvalidCalibration("Calibration not set");
  }

  return p_mpem_cal_->fieldAndGradientCurrentJacobian(position);
}

}  // namespace mag_manip
