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

#include <string>
#include <vector>

#include "mag_manip/forward_model_linear.h"
#include "mag_manip/rbf.h"

namespace mag_manip {

/**
 * @brief Implements the ForwardModel interface for linear radial basis function (RBF) interpolation
 * based models
 */
class ForwardModelLinearThinPlateSpline : public ForwardModelLinear {
 public:
  /**
   * @brief Default constructor
   *
   * You need to load a valid calibration file before calling any functions.
   */
  ForwardModelLinearThinPlateSpline();

  /**
   * @brief Returns the name of the modeled system
   *
   * @return the name of the modeled system
   */
  std::string getName() const;

  /**
   * @brief Returns a vector of pointers to the interpolants
   *
   * The length of the vector is Ne, the number of electromagnets
   *
   * @return a vector of shared_ptrs to ThinplateSplineInterpolant
   */
  std::vector<ThinPlateSplineInterpolator::Ptr> getInterpolants() const;

  /**
   * @brief Sets the model parameters from a YAML file.
   *
   * @param filename of the YAML model configuration file.
   */
  virtual void setCalibrationFile(const std::string& filename) override;

  /**
   * @brief This override is faster than the base class version
   *
   * @param position: the position at which the compute the field actuation matrix
   *
   * @return a matrix of size 3 x Ne where Ne is the umber of electromagnets
   */
  virtual ActuationMat getFieldActuationMatrix(const PositionVec& position) const override;

  // Interface functions

  virtual ActuationMat getActuationMatrix(const PositionVec& position) const override;

  virtual bool isValid() const override;

  virtual int getNumCoils() const override;

 private:
  bool is_valid_;
  std::string name_;
  std::string cal_filename_;
  int num_coils_;
  std::vector<int> v_num_nodes_;
  std::vector<ThinPlateSplineInterpolator::Ptr> v_p_interpolants_;
};
}  // namespace mag_manip
