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

#include "mag_manip/backward_model.h"
#include "mag_manip/backward_model_linear_L2.h"
#include "mag_manip/exceptions.h"
#include "mag_manip/saturation_function.h"

namespace mag_manip {

/**
 * @brief Backward models that are linear to current and have a simple 1D saturation function.
 *
 * The relationship between magnetic control variables and current is linear
 * \f[
 *    \mathbf{i} = h^{-1}\left(A^{\dagger}(\mathbf{p}) ~ \begin{bmatrix} \mathbf{b} \\ \mathbf{g}
 * \end{bmatrix} \right)
 *  \f]
 *
 * \f$\mathbf{A}(\mathbf{p})\f$ is the 8xNc actuation matrix.
 */
class BackwardModelSaturation : public BackwardModel {
 public:
  BackwardModelSaturation();

  virtual ~BackwardModelSaturation() {}

  virtual bool isValid() const override;

  virtual int getNumCoils() const override;

  /**
   * @brief Returns the name of the calibration
   *
   * @return the name of the calibration
   */
  std::string getName() const;

  /**
   * @brief Sets the name of the calibration
   *
   * @param name: new name of the calibration
   */
  void setName(const std::string& name);

  /**
   * @brief Sets the maximum checking
   *
   * @param enable: if true, will check for the maximum
   * of each saturation and throw a mag_manip::OverSaturationException
   * exception if  currents violate the saturation maximum
   */
  void setDoCheckMax(const bool enable);

  /**
   * @brief Load the calibration of the model
   *
   * The calibration should be contained in a folder with a metafile containing the params fo the
   * calibration. The folder should also contain a calibration file for the forward model and for
   * the saturation params at the same level as the meta file.
   *
   * @param filename: path to the metafile of the calibration
   */
  virtual void setCalibrationFile(const std::string& filename) override;

  /**
   * @brief Returns a pointer to the linear model that is saturated
   *
   * @return pointer to a linear model
   */
  virtual BackwardModel::Ptr getModel() const;

  /**
   * @brief Sets the linear model.
   *
   * WARNING: Must be done before computing.
   *
   * @param p_model
   */
  virtual void setModel(BackwardModel::Ptr p_model);

  /**
   * @brief Sets the calibration parameters for the linear model
   *
   * WARNING: this function assumes that a linear model is already loaded and just
   * changes the calibration parameters for that model. Loading the wrong type of calibration file
   * for the type of loaded model can result in an exception
   *
   * @param filename: path to the linear model calibration file
   */
  virtual void setModelCalibrationFile(const std::string& filename);

  /**
   * @brief Gets the univariate saturation function h(x)
   *
   * @return the saturation function for coil number i
   */
  virtual SaturationFunction::Ptr getSaturationFunction(const int i) const;

  /**
   * @brief Gets the saturation function for each electromagnet
   *
   * @return a vector containing all the saturation functions
   */
  virtual std::vector<SaturationFunction::Ptr> getSaturationFunctions() const;

  /**
   * @brief Sets the saturation functions h(x).
   *
   * WARNING: Must be done before performing computations and after setting linear model.
   *
   * @param sat_functions: a vector of length numCoils containing pointers to the saturation
   * functions
   */
  virtual void setSaturationFunctions(std::vector<SaturationFunction::Ptr> sat_functions);

  /**
   * @brief Sets the saturation functions from a YAML file
   * WARNING: Must be done before performing computations and after setting linear model.
   *
   * @param filename: path to saturation functions YAML file
   */
  virtual void setSaturationFunctionsFile(const std::string& filename);

  virtual CurrentsVec computeCurrentsFromField(const PositionVec& position,
                                               const FieldVec& field) const override;

  virtual CurrentsVec computeCurrentsFromFieldGradient5(
      const PositionVec& position, const FieldVec& field,
      const Gradient5Vec& gradient) const override;

  virtual CurrentsVec computeCurrentsFromFieldDipoleGradient3(
      const PositionVec& position, const FieldVec& field, const DipoleVec& dipole,
      const Gradient3Vec& gradient) const override;

  virtual void setCachedPosition(const PositionVec& position) override;
  PositionVec getCachedPosition() const override;

  virtual void setCachedPositionDipole(const PositionVec& position,
                                       const DipoleVec& dipole) override;

  DipoleVec getCachedDipole() const override;

  CurrentsVec computeCurrentsFromFieldCached(const FieldVec& field) const override;

  CurrentsVec computeCurrentsFromFieldGradient5Cached(const FieldVec& field,
                                                      const Gradient5Vec& gradient) const override;

  CurrentsVec computeCurrentsFromFieldDipoleGradient3Cached(
      const FieldVec& field, const Gradient3Vec& gradient) const override;

 protected:
  BackwardModel::Ptr p_bac_model_;
  std::vector<SaturationFunction::Ptr> sat_functions_;
  std::string cal_name_;
  bool do_check_max_;

  static void checkMax(SaturationFunction::Ptr p_sat, double current);
};
}  // namespace mag_manip
