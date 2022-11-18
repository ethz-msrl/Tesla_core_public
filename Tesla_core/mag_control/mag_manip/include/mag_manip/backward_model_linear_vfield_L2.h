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
#include "mag_manip/backward_model_linear_L2.h"
#include "mag_manip/interpolate_regular.h"

namespace mag_manip {

class BackwardModelLinearVFieldL2 : public BackwardModelLinearL2 {
 public:
  /**
   * @brief Default constructor.
   *
   * You need to load a valid calibration file before calling any functions.
   */
  BackwardModelLinearVFieldL2();

  /**
   * @brief Loads the vfield calibration from a YAML file
   *
   * See scripts/convert_aeon_vfield.py to convert the Aeon file format
   * to YAML
   *
   * @param filename: path to the YAML file defining the calibration
   */
  virtual void setCalibrationFile(const std::string& filename) override;

  /**
   * @brief Checks if a position is inside the area that was calibrated.
   * If the position is far outside the calibrated area, the calculations
   * provided by the model may not be very accurate.
   *
   * @param position to be checked.
   *
   * @return true if position is in the calibrated area of the model.
   */
  bool pointInWorkspace(const PositionVec& position) const;

  /**
   * @brief Sets the interpolation type
   *
   * @param type: the interpolation type
   */
  void setInterpolationType(InterpolateRegular::Type type);

  /**
   * @brief Gets the interpolation type
   *
   * @return interpolation type
   */
  InterpolateRegular::Type getInterpolationType() const { return interp_type_; }

  /**
   * @brief gets the actuation matrix relating changes in current to changes in field
   *
   * The field actuation matrix is of size 3 x Ne where Ne is the number of electromagnets
   *
   * @param position: the position at which to calculate the actuation matrix
   *
   * @return the field actuation matrix
   */
  ActuationMat getFieldActuationMatrix(const PositionVec& position) const override;

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

  // Interface functions
  ActuationMat getActuationMatrix(const PositionVec& position) const override;

  /**
   * @brief Returns the name defined in the model calibration file.
   *
   */
  std::string getName() const { return name_; }

  bool isValid() const override { return is_valid_; }

  int getNumCoils() const override { return num_coils_; }

 private:
  InterpolateRegular::Type interp_type_;
  std::vector<InterpolateRegular::UPtr> interpolants_;
  std::string name_;
  std::string filename_;
  bool is_valid_;
  int num_coils_;
};
}  // namespace mag_manip
