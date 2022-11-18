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
#include <vector>

#include "mag_manip/forward_model_linear.h"
#include "mag_manip/interpolate_regular.h"

namespace mag_manip {

class ForwardModelLinearVField : public ForwardModelLinear {
 public:
  /**
   * @brief Constructor for a forward model based on linearly interpolated vector fields
   *
   * The default interpolation type is tricubic.
   */
  ForwardModelLinearVField();

  /**
   * @brief Sets the interpolation type
   *
   * @param type: the interpolation type
   */
  void setInterpolationType(InterpolateRegular::Type type);

  /**
   * @brief Gets the interpolation type
   *
   * @return the interpolation type
   */
  InterpolateRegular::Type getInterpolationType() const { return interp_type_; }

  /**
   * @brief Checks if the position is in the calibrated workspace
   *
   * @param position: a 3D position vector
   *
   * @return true if position is in the calibrated workspace of the model
   */
  bool pointInWorkspace(const PositionVec& position) const;

  /**
   * @brief Sets the data from vector fields in libAeon's vfield file format
   *
   * This will define the number of coils and the properties of the grid for each coil
   *
   * The format is defined as follows:
   *
   * total_points # total number of points in the grid
   * dim_x dim_y dim_z # the size of the grid in the 3 directions
   * min_x max_x # the minimum and maximum position values on the grid
   * min_y max_y # the minimum and maximum position values on the grid
   * min_z max_z # the minimum and maximum position values on the grid
   * p_x p_y p_z b_x b_y b_z # the position of the measurement and the magnetic field for a unit
   * current 1A
   * this is repeated for total_points
   *
   * See the test folder for examples of valid files.
   *
   * @param vfield_filenames: each coil has a filename in Aeon's format
   */
  void setVFieldFiles(const std::vector<std::string>& vfield_filenames);

  /**
   * @brief Gets the grid properties for each coil
   *
   * @return a vector of grid properties of length Nc
   */
  std::vector<VFieldGridProperties> getVFieldGridProperties() const;

  /**
   * @brief Computes the interpolated 3x3 gradient matrix
   *
   * Note that interpolation doesn't ensure that the gradient is symmetric and zero trace.
   *
   * @param position: the 3D position at which to compute the gradient matrix
   * @param currents: the currents on the electromagnets
   *
   * @return a 3x3 matrix of interpolated gradient values
   */
  GradientMat computeGradientMatFromCurrents(const PositionVec& position,
                                             const CurrentsVec& currents) const;

  // Interface Functions
  //
  /**
   * @brief This function does not do anything at the moment
   *
   * @param filename
   */
  virtual void setCalibrationFile(const std::string& filename) override;

  virtual bool isValid() const override { return is_valid_; }

  virtual int getNumCoils() const override { return num_coils_; }

  virtual FieldVec computeFieldFromCurrents(const PositionVec& position,
                                            const CurrentsVec& currents) const override;

  virtual Gradient5Vec computeGradient5FromCurrents(const PositionVec& position,
                                                    const CurrentsVec& currents) const override;

  virtual FieldGradient5Vec computeFieldGradient5FromCurrents(
      const PositionVec& position, const CurrentsVec& currents) const override;

  virtual ActuationMat getActuationMatrix(const PositionVec& position) const override;

  virtual ActuationMat getFieldActuationMatrix(const PositionVec& position) const override;

 protected:
  int num_coils_;
  std::string name_;
  bool is_valid_;
  std::vector<std::unique_ptr<InterpolateRegular>> interpolants_;
  InterpolateRegular::Type interp_type_;
  std::string filename_;
};
}  // namespace mag_manip
