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

#include "mag_manip/exceptions.h"
#include "mag_manip/forward_model.h"
#include "mag_manip/saturation_function.h"

namespace mag_manip {

/**
 * @brief Forward models that have a simple 1D saturation function.
 *
 * The relationship between magnetic control variables and current is linear
 * \f[
 *      \begin{bmatrix} \mathbf{b} \\ \mathbf{g} \end{bmatrix} =
 *      \mathbf{A}(\mathbf{p}) ~ h(\mathbf{i})
 * \f]
 *
 * \f$\mathbf{A}(\mathbf{p})\f$ is the 8xNc actuation matrix.
 *
 * Note that while called LinearSaturation, the class does not force you
 * to use a model that is explicitely linear. You could use a generic nonlinear forward model and
 * have it be saturated at the output.
 */
class ForwardModelSaturation : public ForwardModel {
 public:
  virtual ~ForwardModelSaturation() {}

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
   * @brief Sets the calibration parameters for the forward model
   *
   * WARNING: this function assumes that a forward model is already loaded and just
   * changes the calibration parameters for that model. Loading the wrong type of calibration file
   * for the type of loaded model can result in an exception
   *
   * @param filename: path to the forward model calibration file
   */
  virtual void setModelCalibrationFile(const std::string& filename);

  /**
   * @brief Returns a pointer to the forward model that is saturated
   *
   * @return pointer to a forward model
   */
  virtual ForwardModel::Ptr getModel() const;

  /**
   * @brief Sets the forward model.
   *
   * WARNING: Must be done before computing.
   *
   * Note:
   * You do not to use a model that is explicitely linear. You could use a generic nonlinear forward
   * model and have it be saturated at the output.
   *
   * @param p_lin_model
   */
  virtual void setModel(ForwardModel::Ptr p_lin_model);

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
   * @brief Sets the saturation functions from a YAML file
   * WARNING: Must be done before performing computations and after setting forward model.
   *
   * @param filename: path to saturation functions YAML file
   */
  virtual void setSaturationFunctionsFile(const std::string& filename);

  /**
   * @brief Sets the saturation functions h(x).
   *
   * WARNING: Must be done before performing computations and after setting forward model.
   *
   * @param sat_functions: a vector of length numCoils containing pointers to the saturation
   * functions
   */
  virtual void setSaturationFunctions(std::vector<SaturationFunction::Ptr> sat_functions);

  virtual FieldVec computeFieldFromCurrents(const PositionVec& position,
                                            const CurrentsVec& currents) const override;

  virtual Gradient5Vec computeGradient5FromCurrents(const PositionVec& position,
                                                    const CurrentsVec& currents) const override;

  virtual FieldGradient5Vec computeFieldGradient5FromCurrents(
      const PositionVec& position, const CurrentsVec& currents) const override;

  virtual void setCachedPosition(const PositionVec& position) override;

  virtual PositionVec getCachedPosition() const override;

  virtual FieldVec computeFieldFromCurrentsCached(const CurrentsVec& currents) const override;

  virtual Gradient5Vec computeGradient5FromCurrentsCached(
      const CurrentsVec& currents) const override;

  virtual FieldGradient5Vec computeFieldGradient5FromCurrentsCached(
      const CurrentsVec& currents) const override;

 protected:
  /**
   * @brief Pointer to the model that is saturated
   */
  ForwardModel::Ptr p_for_model_;
  std::vector<SaturationFunction::Ptr> sat_functions_;

  std::string cal_name_;
};

}  // namespace mag_manip
