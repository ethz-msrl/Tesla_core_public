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

#include <Eigen/SVD>
#include "mag_manip/backward_model.h"

namespace mag_manip {

/**
 * @brief Inverts models that are linear to current using least-squares.
 *
 * Electromagnets that operate outside their saturation respond linearly to
 * current. Magnetic manipulation systems of several electromagnets thus obey
 * the principle of superposition.
 *
 * The relationship between magnetic control variables and current is linear
 * \f[
 *      \begin{bmatrix} \mathbf{b} \\ \mathbf{g} \end{bmatrix} =
 *      \mathbf{A}(\mathbf{p}) ~ \mathbf{i}
 * \f]
 *
 * \f$\mathbf{A}(\mathbf{p})\f$ is the 8xNc actuation matrix.
 */
class BackwardModelLinearL2 : public BackwardModel {
 public:
  typedef std::unique_ptr<BackwardModelLinearL2> UPtr;
  typedef std::shared_ptr<BackwardModelLinearL2> Ptr;

  /**
   * @brief returns the 8xNc actuation matrix at a given position.
   *
   * The first three rows of the actuation matrix are the linear gains between
   * currents (in Amps) and magnetic field (in Tesla).
   *
   * The bottom 5 rows are the gains between currents and magnetic field
   * gradients (in Tesla/meter)
   *
   * @param position at which the actuation matrix is calculated in meters.
   *
   * @return the actuation matrix.
   */
  virtual ActuationMat getActuationMatrix(const PositionVec& position) const = 0;

  /**
   * @brief Returns the 3xNc matrix relating currents to magnetic fields at a given position
   *
   * Nc is the number of coil. This is just the first 3 rows of the actuation matrix.
   *
   * @param position at which the actuation matrix is calculated in meters.
   *
   * @return the field actuation matrix
   */
  virtual ActuationMat getFieldActuationMatrix(const PositionVec& position) const;

  /**
   * @brief returns the pseudoinverse of the actuation matrix at a given position
   *
   * The Moore-Penrose pseudo-inverse is computed using SVD.
   * The size of the returned matrix is 8xNc where Nc is the number of coils.
   * This matrix can be used for computing currents using a matrix multiplication
   * instead of using computeCurrentsFromFieldGradient5.
   *
   * @param position at which the actuation matrix is calculated in meters.
   *
   * @return the pseudo-inverse of actuation matrix.
   */
  virtual ActuationMat getActuationMatrixInverse(const PositionVec& position) const;

  /**
   * @brief returns the pseudoinverse of the field actuation matrix at a given position
   *
   * The Moore-Penrose pseudo-inverse is computed using SVD.
   * The size of the returned matrix is 8xNc where Nc is the number of coils.
   * This matrix can be used for computing currents using a matrix multiplication
   * instead of using computeCurrentsFromField.
   *
   * @param position at which the actuation matrix is calculated in meters.
   *
   * @return the pseudo-inverse of field actuation matrix.
   */
  virtual ActuationMat getFieldActuationMatrixInverse(const PositionVec& position) const;

  /**
   * @brief Computes the least-squares currents that generate the desired
   * field at a given position.
   *
   * The relationship between magnetic control variables and current is linear
   * \f[
   *      \mathbf{b} =
   *      \mathbf{B}(\mathbf{p}) ~ \mathbf{i}
   * \f]
   * This solves for \f$\mathbf{i}\f$ while minimzing \f||$\mathbf{i}||\f$
   *
   *
   * @param position: position at which the field should be applied in meters
   * @param field: desired magnetic field in Tesla
   *
   * @return least-squares current vector
   */
  virtual CurrentsVec computeCurrentsFromField(const PositionVec& position,
                                               const FieldVec& field) const override;

  /**
   * @brief Computes the least-squares currents that generate the desired
   * field and 5D gradient at a given position.
   *
   * The relationship between magnetic control variables and current is linear
   * \f[
   *      \begin{bmatrix} \mathbf{b} \\ \mathbf{g} \end{bmatrix} =
   *      \mathbf{A}(\mathbf{p}) ~ \mathbf{i}
   * \f]
   * This solves for \f$\mathbf{i}\f$ while minimzing \f||$\mathbf{i}||\f$

   * @param position: position at which the field and gradient should be
   * applied in meters
   * @param field: desired magnetic field in Tesla
   * @param gradient: desired 5D gradient vector in Tesla/meter
   *
   * @return least-squares current vector
   */
  virtual CurrentsVec computeCurrentsFromFieldGradient5(
      const PositionVec& position, const FieldVec& field,
      const Gradient5Vec& gradient) const override;

  virtual CurrentsVec computeCurrentsFromFieldDipoleGradient3(
      const PositionVec& position, const FieldVec& field, const DipoleVec& dipole,
      const Gradient3Vec& gradient) const override;

  virtual void setCachedPosition(const PositionVec& position) override;

  virtual void setCachedPositionDipole(const PositionVec& position,
                                       const DipoleVec& dipole) override;

  virtual PositionVec getCachedPosition() const override;

  virtual DipoleVec getCachedDipole() const override;

  virtual CurrentsVec computeCurrentsFromFieldCached(const FieldVec& field) const override;

  virtual CurrentsVec computeCurrentsFromFieldGradient5Cached(
      const FieldVec& field, const Gradient5Vec& gradient) const override;

  virtual CurrentsVec computeCurrentsFromFieldDipoleGradient3Cached(
      const FieldVec& field, const Gradient3Vec& gradient) const override;

 private:
  PositionVec position_cached_;
  DipoleVec dipole_cached_;
  Eigen::JacobiSVD<ActuationMat> svd_field_cached_;
  Eigen::JacobiSVD<ActuationMat> svd_field_grad_cached_;
  Eigen::JacobiSVD<ActuationMat> svd_field_grad_mod_cached_;
};
}  // namespace mag_manip
