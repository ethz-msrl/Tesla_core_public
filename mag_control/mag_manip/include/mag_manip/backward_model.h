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
 * @brief Inverts magnetic field generation models.
 */
class BackwardModel {
 public:
  typedef std::unique_ptr<BackwardModel> UPtr;
  typedef std::shared_ptr<BackwardModel> Ptr;

  virtual ~BackwardModel() {}

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
   * @brief Computes currents that generate the desired field at a given
   * position.
   *
   * @param position: position at which the field should be applied in meters
   * @param field: desired magnetic field in Tesla
   *
   * @return currents vector that generates the desired field
   */
  virtual CurrentsVec computeCurrentsFromField(const PositionVec& position,
                                               const FieldVec& field) const = 0;
  /**
   * @brief Computes several current vectors that generate the desired field at given positions
   *
   * This reference implementation just repeatedly calls computeCurrenstFromField
   *
   * @param positions: a matrix of size 3xN with the N desired positions in meters associated with
   * the N desired fields
   * @param fields: a matrix of size 3xN with the N desired fields in Tesla
   *
   * @return a matrix of sized Ne x N with the current vectors as columns associated with each
   * desired field
   */
  virtual CurrentsVecs computeCurrentsFromFields(const PositionVecs& positions,
                                                 const FieldVecs& fields) const;

  /**
   * @brief Computes currents that generate the desired field and 5D gradient
   * at a given position.
   *
   * @param position: position at which the field and gradient should be
   * applied in meters
   * @param field: desired magnetic field in Tesla
   * @param gradient: desired 5D gradient vector in Tesla/meter
   *
   * @return currents vector that generates the desired field and gradient
   */
  virtual CurrentsVec computeCurrentsFromFieldGradient5(const PositionVec& position,
                                                        const FieldVec& field,
                                                        const Gradient5Vec& gradient) const = 0;

  /**
   * @brief Computes currents that generate the desired field and 5D gradient
   * at a list of given positions.
   *
   * Reference implementation just repeteadly calls computeCurrentsFromFieldGradient5
   *
   * @param positions: a matrix of size 3xN with the N desired positions in meters associated with
   * the N desired fields
   * @param fields: a matrix of size 3xN with the N desired fields in Tesla
   * @param gradients: a matrix of size 5xN with N desired gradients in Tesla/meters
   *
   * @return a matrix of sized Ne x N with the current vectors as columns associated with each
   * desired field
   */
  virtual CurrentsVecs computeCurrentsFromFieldGradient5s(const PositionVecs& positions,
                                                          const FieldVecs& fields,
                                                          const Gradient5Vecs& gradients) const;

  /**
   * @brief Computes the currents that generate the desired field and
   * the 3D gradient vector aligned with a direction vector
   *
   * The 3D gradient vector is calculated from the 3D gradient matrix as follows
   * \f[
   * \mathbf{g}_3 = \nabla \mathbf{b}~\mathbf{v}
   * \f]
   * where \f$\mathbf{g}_3\f$ is the 3D gradient vector, and \f$ \mathbf{v} \f$ is the dipole
   * direction vector.
   *
   * @param position: position at which the field and gradient should be
   * applied in meters
   * @param field: desired magnetic field in Tesla
   * @param dipole: a direction vector (should be a length 1 vector)
   * @param gradient: desired 3D gradient vector.
   *
   * @return currents vector that generates the desired field and gradient
   */
  virtual CurrentsVec computeCurrentsFromFieldDipoleGradient3(
      const PositionVec& position, const FieldVec& field, const DipoleVec& dipole,
      const Gradient3Vec& gradient) const = 0;

  /**
   * @brief Computes the currents that generate a list of desired fields and
   * the 3D gradient vector aligned with a direction vectors
   *
   * This reference implementation just repeteadly calls computeCurrentsFromFieldDipoleGradient3
   *
   * @param positions: a matrix of size 3xN with the N desired positions in meters associated with
   * the N desired fields
   * @param fields: a matrix of size 3xN with the N desired fields in Tesla
   * @param dipoles: a matrix of size 3xN with the N desired dipole directions
   * @param gradients: a matrix of size 3xN with N desired gradients in Tesla/meters
   *
   * @return currents vector that generates the desired field and gradient
   */
  virtual CurrentsVecs computeCurrentsFromFieldDipoleGradient3s(
      const PositionVecs& positions, const FieldVecs& fields, const DipoleVecs& dipoles,
      const Gradient3Vecs& gradients) const;

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
   * @brief Set a cached dipole and position
   *
   * This is used if you want to call computeCurrentsFromFieldDipoleGradient3Cached
   *
   * throws NotImplementedException
   *
   * @param position: the position to cache
   * @param dipole: the dipole direction to cache
   */
  virtual void setCachedPositionDipole(const PositionVec& position, const DipoleVec& dipole);

  /**
   * @brief get the cached dipole direction
   *
   * throws NotImplementedException
   *
   * @return cached dipole direction
   */
  virtual DipoleVec getCachedDipole() const;

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
   * @brief Cached version of computeCurrentsFromField
   *
   * throws NotImplementedException
   *
   * @param field: the desired field
   *
   * @return the computed currents
   */
  virtual CurrentsVec computeCurrentsFromFieldCached(const FieldVec& field) const;
  /**
   * @brief Cached version of computeCurrentsFromFieldGradient5
   *
   * @param field: the desired field
   * @param gradient: the desired gradient
   *
   * Call setCachedPosition before calling this or it will raise an exception
   *
   * throws NotImplementedException
   *
   * @return the computed Nex1 vector of electromagnet currents
   */
  virtual CurrentsVec computeCurrentsFromFieldGradient5Cached(const FieldVec& field,
                                                              const Gradient5Vec& gradient) const;

  /**
   * @brief Cached version of computeCurrentsFromFieldDipoleGradient3
   *
   * @param field: the desired field
   * @param gradient: the desired gradient
   *
   * Call setCachedPositionDipole before calling this or it will raise an exception
   *
   * throws NotImplementedException
   *
   * @return the computed Nex1 vector of electromagnet currents
   */
  virtual CurrentsVec computeCurrentsFromFieldDipoleGradient3Cached(
      const FieldVec& field, const Gradient3Vec& gradient) const;
};
}  // namespace mag_manip
