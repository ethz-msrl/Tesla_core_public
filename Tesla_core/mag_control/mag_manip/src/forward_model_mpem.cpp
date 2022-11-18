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

#include "mag_manip/forward_model_mpem.h"

#include "mag_manip/exceptions.h"
#include "mag_manip/helpers.h"

namespace mag_manip {
ForwardModelMPEM::ForwardModelMPEM() : cal_filename_("") {}

bool ForwardModelMPEM::isValid() const { return p_mpem_cal_.get() != nullptr; }

int ForwardModelMPEM::getNumCoils() const {
  if (!p_mpem_cal_) {
    throw InvalidCalibration("Calibration not set");
  }

  return p_mpem_cal_->getNumberOfCoils();
}

std::string ForwardModelMPEM::getName() const {
  if (!p_mpem_cal_) {
    throw InvalidCalibration("Calibration not set");
  }

  return p_mpem_cal_->getName();
}

bool ForwardModelMPEM::pointInWorkspace(const PositionVec& position) const {
  if (!p_mpem_cal_) {
    throw InvalidCalibration("Calibration not set");
  }

  return p_mpem_cal_->pointInWorkspace(position);
}

void ForwardModelMPEM::setCalibrationFile(const std::string& filename) {
  try {
    p_mpem_cal_.reset(new ElectromagnetCalibration(filename));
  } catch (std::exception& e) {
    throw InvalidFile(filename, e.what());
  }
  cal_filename_ = filename;
}

ActuationMat ForwardModelMPEM::getActuationMatrix(const PositionVec& position) const {
  if (!p_mpem_cal_) {
    throw InvalidCalibration("Calibration not set");
  }

  return p_mpem_cal_->fieldAndGradientCurrentJacobian(position);
}

FieldVec ForwardModelMPEM::computeFieldFromCurrents(const PositionVec& position,
                                                    const CurrentsVec& currents) const {
  if (!p_mpem_cal_) {
    throw InvalidCalibration("Calibration not set");
  }

  return p_mpem_cal_->fieldAtPoint(currents, position);
}

Gradient5Vec ForwardModelMPEM::computeGradient5FromCurrents(const PositionVec& position,
                                                            const CurrentsVec& currents) const {
  if (!p_mpem_cal_) {
    throw InvalidCalibration("Calibration not set");
  }

  GradientMat grad_mat = p_mpem_cal_->gradientAtPoint(currents, position);
  return gradientMatToGradient5Vec(grad_mat);
}

FieldGradient5Vec ForwardModelMPEM::computeFieldGradient5FromCurrents(
    const PositionVec& position, const CurrentsVec& currents) const {
  if (!p_mpem_cal_) {
    throw InvalidCalibration("Calibration not set");
  }

  return p_mpem_cal_->fieldAndGradientAtPoint(currents, position);
}
}  // namespace mag_manip
