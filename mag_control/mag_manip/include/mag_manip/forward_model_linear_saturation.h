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

#include <memory>
#include <string>

#include "mag_manip/forward_model_linear.h"
#include "mag_manip/forward_model_saturation.h"

#pragma once

namespace mag_manip {

class ForwardModelLinearSaturation : public ForwardModelSaturation {
 public:
  using Ptr = std::shared_ptr<ForwardModelLinearSaturation>;
  using UPtr = std::unique_ptr<ForwardModelLinearSaturation>;

  /**
   * @brief Set the calibration from a configuration file
   *
   * This will set both the linear model and the saturation
   * params
   *
   * @param filename path to a YAML metafile containing
   * all the calibration info
   */
  void setCalibrationFile(const std::string& filename) override;

  /**
   * @brief Sets the linear part of the model
   *
   * No need to also call setModel
   *
   * @param p_lin_model
   */
  virtual void setLinearModel(ForwardModelLinear::Ptr p_lin_model);

  /**
   * @brief Gets the linear model
   *
   * @return linear model
   */
  virtual ForwardModelLinear::Ptr getLinearModel() const;

  /**
   * @brief Sets the linear part of the model by dynamic casting.
   *
   * Prefer to use setLinearModel
   *
   * @param p_model
   */
  virtual void setModel(ForwardModel::Ptr p_model) override;

  virtual void setCachedPosition(const PositionVec& position) override;

  FieldVec computeFieldFromCurrentsCached(const CurrentsVec& currents) const override;

 private:
  ForwardModelLinear::Ptr p_lin_model_;
};
}  // namespace mag_manip
