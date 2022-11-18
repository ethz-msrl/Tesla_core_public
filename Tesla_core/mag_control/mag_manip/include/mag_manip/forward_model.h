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

#include "mag_manip/types.h"

namespace mag_manip {

/**
 * @brief Interface class for implementing magnetic manipulation system models.
 *
 * This handles the forward part of the model ie converting from electromagnet
 * currents to magnetic control properties such as magnetic fields and magnetic
 * field gradients.
 *
 */
class ForwardModel {
 public:
  typedef std::unique_ptr<ForwardModel> UPtr;
  typedef std::shared_ptr<ForwardModel> Ptr;

  virtual ~ForwardModel() {}

  /**
   * @brief Loads the parameters of the model from a file.
   *
   * There is no standard file type for magnetic calibrations yet so this is a catch all
   * that will be loaded whenever the model is created.
   *
   * @param filename: path to a calibration file.
   */
  virtual void setCalibrationFile(const std::string& filename) = 0;

  /**
   * @brief Returns true if the magnetic model is in a valid state.
   *
   * For example, if the model needs to load a file and that file is not yet
   * loaded, this function should return false.
   *
   * @return true if the magnetic model is in a valid state.
   */
  virtual bool isValid() const = 0;

  /**
   * @brief Returns the number of coils in the magnetic manipulation system
   * that is modelled
   *
   * @return the number of coils in the magnetic manipulation system
   */
  virtual int getNumCoils() const = 0;

  /**
   * @brief Computes the 3D magnetic field at a given position and given
   * electromagnet currents.
   *
   * The magnetic field is returned as the 3D magnetic flux density vector in
   * Tesla.
   *
   * @param position: 3D position vector with respect to the center of the
   * model's workspace in meters.
   * @param currents: A vector of length Nc, where Nc is the number of
   * electromagnets of current values in Amps.
   *
   * @return the magnetic field flux density in Tesla.
   */
  virtual FieldVec computeFieldFromCurrents(const PositionVec& position,
                                            const CurrentsVec& currents) const = 0;
  /**
   * @brief Computes the 3D magnetic field at a list of positions given electromagnet currents
   *
   * A reference implementation is provided which makes repeated calls to computeFieldFromCurrents.
   * Derived classes may provide an optimized version by overriding this function.
   *
   * currents can either be supplied as a vector, where the same current is used at all positions or
   * as a matrix with N * cols
   *
   * @param positions: a 3xN matrix containing the position values as columns
   * @param currents: a vector of length Ne, where Ne is the number of electromagnets
   * of current values in Amps
   *
   * @return a 3xN matrix where the columns are the magnetic field vectors in Tesla
   * corresponding to the positions
   */
  virtual FieldVecs computeFieldsFromCurrents(const PositionVecs& positions,
                                              const CurrentsVec& currents) const;

  /**
   * @brief Computes the 5D magnetic field gradient at a given position and
   * given electromagnet  currents.
   *
   * The gradient is returned as the 5D magnetic flux density vector in
   * Tesla/meter. G = [dBx/dx, dBx/dy, dBz/dz, dBy/dy. dBy/dz]
   *
   * @param position: 3D position vector with respect to the center of the
   * model's workspace in meters.
   * @param currents: A vector of length Nc, where Nc is the number of
   * electromagnets of current values in Amps.
   *
   * @return the magnetic gradient vector.
   */
  virtual Gradient5Vec computeGradient5FromCurrents(const PositionVec& position,
                                                    const CurrentsVec& currents) const = 0;
  /**
   * @brief Computes the 5D magnetic field gradient at given positions and given electromagnet
   * currents
   *
   * A reference implementation is provided which makes repeated calls to computeFieldFromCurrents.
   * Derived classes may provide an optimized version by overriding this function.
   *
   * currents can either be supplied as a vector, where the same current is used at all positions or
   * as a matrix with N * cols
   *
   * @param positions: a 3xN matrix containing the position values as columns
   * @param currents: a vector of length Ne, where Ne is the number of electromagnets
   * of current values in Amps
   *
   * @return a 5xN matrix where the columns are the magnetic field gradient vectors in Tesla/meter
   * corresponding to the positions
   *
   */
  virtual Gradient5Vecs computeGradient5sFromCurrents(const PositionVecs& positions,
                                                      const CurrentsVec& currents) const;

  /**
   * @brief Computes the concatenation of the 3D magnetic field vector and 5D
   * magnetic field gradient at a given position and given electromagnet
   * currents.
   *
   * Returns a 8D vector where the first 3 rows are the magnetic field vector,
   * and the bottom 5 rows are the magnetic gradient vector.
   *
   * See computeFieldFromCurrents and computeGradient5FromCurrents for more
   * info on the magnetic field and gradients.
   *
   * @param position: 3D position vector with respect to the center of the
   * model's workspace in meters.
   * @param currents: A vector of length Nc, where Nc is the number of
   * electromagnets of current values in Amps.
   *
   * @return the concatenated magnetic field and magnetic field gradient
   * vector.
   */
  virtual FieldGradient5Vec computeFieldGradient5FromCurrents(
      const PositionVec& position, const CurrentsVec& currents) const = 0;

  /**
   * @brief Computes the concatenation of the 3D magnetic field vector and 5D magnetic field
   * gradient at several given positions and given electromagnet currents
   *
   * A reference implementation is provided which makes repeated calls to
   * computeFieldGradient5FromCurrents. Derived classes may provide an optimized version by
   * overriding this function.
   *
   * currents can either be supplied as a vector, where the same current is used at all positions or
   * as a matrix with N * cols
   *
   * @param positions: a 3xN matrix containing the position values as columns
   * @param currents: a vector of length Ne, where Ne is the number of electromagnets
   * of current values in Amps
   *
   * @return a 8xN matrix where the columns are the magnetic field/gradient vectors in Tesla/meter
   * corresponding to the positions
   *
   */
  virtual FieldGradient5Vecs computeFieldGradient5sFromCurrents(const PositionVecs& positions,
                                                                const CurrentsVec& currents) const;

  /**
   * @brief Set a cached position
   *
   * This is used if you want to call computeCurrentsFromFieldDipoleGradient3Cached
   *
   * throws NotImplementedException
   *
   * @param position: the position to cache
   */
  virtual void setCachedPosition(const PositionVec& position);

  /**
   * @brief Get the cached position
   *
   * Call setCachedPosition before calling this or it will raise an exception
   *
   * throws NotImplementedException
   *
   * @return the cached position
   */
  virtual PositionVec getCachedPosition() const;

  /**
   * @brief Cached version of computeFieldFromCurrents
   *
   * Call setCachedPosition before calling this or it will raise an exception
   *
   * throws NotImplementedException
   *
   * @param currents: the Nex1 vector of electromagnet currents
   *
   * @return the computed field
   */
  virtual FieldVec computeFieldFromCurrentsCached(const CurrentsVec& currents) const;

  /**
   * @brief Cached version of computeGradient5FromCurrentsCached
   *
   * Call setCachedPosition before calling this or it will raise an exception
   *
   * throws NotImplementedException
   *
   * @param currents: the Nex1 vector of electromagnet currents
   *
   * @return the computed gradient
   */
  virtual Gradient5Vec computeGradient5FromCurrentsCached(const CurrentsVec& currents) const;

  /**
   * @brief Cached version of computeFieldGradient5FromCurrents
   *
   * Call setCachedPosition before calling this or it will raise an exception
   *
   * throws NotImplementedException
   *
   * @param currents: the Nex1 vector of electromagnet currents
   *
   * @return the computed field and gradient
   */
  virtual FieldGradient5Vec computeFieldGradient5FromCurrentsCached(
      const CurrentsVec& currents) const;
};
}  // namespace mag_manip
