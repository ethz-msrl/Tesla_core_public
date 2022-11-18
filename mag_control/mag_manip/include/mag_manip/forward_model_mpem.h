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

#pragma once

#include <memory>
#include <string>

#include "mag_manip/forward_model_linear.h"
#include "mpem/electromagnet_calibration.h"

namespace mag_manip {

/**
 * @brief Implements the ForwardModel interface for the multipole electromagnet
 * model in the mpem package.
 *
 */
class ForwardModelMPEM : public ForwardModelLinear {
 public:
  /**
   * @brief Default constructor.
   *
   * You need to load a valid calibration file before calling any functions.
   */
  ForwardModelMPEM();

  /**
   * @brief Sets the model parameters from a YAML file.
   *
   * @param filename of the YAML model configuration file.
   */
  virtual void setCalibrationFile(const std::string& filename) override;

  /**
   * @brief Returns the name defined in the model calibration file.
   *
   */
  std::string getName() const;

  /**
   * @brief Checks if a position is inside the area that was calibrated.
   * If the position is far outside the calibrated area, the calculations provided by
   * the model may not be very accurate.
   *
   * @param position to be checked.
   *
   * @return true if position is in the calibrated area of the model.
   */
  bool pointInWorkspace(const PositionVec& position) const;

  // Interface functions

  virtual ActuationMat getActuationMatrix(const PositionVec& position) const override;

  virtual bool isValid() const override;

  virtual int getNumCoils() const override;

  virtual FieldVec computeFieldFromCurrents(const PositionVec& position,
                                            const CurrentsVec& currents) const override;

  virtual Gradient5Vec computeGradient5FromCurrents(const PositionVec& position,
                                                    const CurrentsVec& currents) const override;

  virtual FieldGradient5Vec computeFieldGradient5FromCurrents(
      const PositionVec& position, const CurrentsVec& currents) const override;

 private:
  std::string cal_filename_;
  std::unique_ptr<ElectromagnetCalibration> p_mpem_cal_;
};
}  // namespace mag_manip
