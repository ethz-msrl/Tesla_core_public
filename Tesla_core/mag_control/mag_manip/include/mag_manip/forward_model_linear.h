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

#include "mag_manip/forward_model.h"

namespace mag_manip {

/**
 * @brief Forward models that are linear to current.
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
class ForwardModelLinear : public ForwardModel {
 public:
  typedef std::unique_ptr<ForwardModelLinear> UPtr;
  typedef std::shared_ptr<ForwardModelLinear> Ptr;

  virtual ~ForwardModelLinear() {}

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
   * @brief returns the 3xNc actuation matrix at a given position
   *
   * The three rows are the linear gains between the currents (in Amps)
   * and the magnetic field (in Tesla)
   *
   * @param position at which the actuation matrix is calculated in meters.
   *
   * @return the actuation matrix.
   */
  virtual ActuationMat getFieldActuationMatrix(const PositionVec& position) const;

  /**
   * @brief Computes the 3D magnetic field at a given position and given
   * electromagnet currents.
   *
   * The magnetic field is returned as the 3D magnetic flux density vector in
   * Tesla.
   *
   * Non-optimized default version. This function uses the getFieldActuationMatrix function to
   * compute the predicted
   * field.
   *
   * @param position: 3D position vector with respect to the center of the
   * model's workspace in meters.
   * @param currents: A vector of length Nc, where Nc is the number of
   * electromagnets of current values in Amps.
   *
   * @return the magnetic field flux density in Tesla.
   */
  virtual FieldVec computeFieldFromCurrents(const PositionVec& position,
                                            const CurrentsVec& currents) const override;

  /**
   * @brief Default non optimized version
   *
   * The gradient is returned as the 5D magnetic flux density vector in
   * Tesla/meter. G = [dBx/dx, dBx/dy, dBz/dz, dBy/dy. dBy/dz]
   *
   * This function uses the getActuationMatrix function to compute the field
   *
   * @param position: 3D position vector with respect to the center of the
   * @param currents: A vector of length Nc, where Nc is the number of
   *
   * @return the magnetic gradient vector.
   */
  virtual Gradient5Vec computeGradient5FromCurrents(const PositionVec& position,
                                                    const CurrentsVec& currents) const override;

  /**
   * @brief Computes the concatenation of the 3D magnetic field vector and 5D
   * magnetic field gradient at a given position and given electromagnet
   * currents.
   *
   * Non optimized version. Uses getActuationMatrix to compute the field and gradient
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
      const PositionVec& position, const CurrentsVec& currents) const override;

  virtual void setCachedPosition(const PositionVec& position) override;

  virtual PositionVec getCachedPosition() const override;

  virtual FieldVec computeFieldFromCurrentsCached(const CurrentsVec& currents) const override;

  virtual Gradient5Vec computeGradient5FromCurrentsCached(
      const CurrentsVec& currents) const override;

  virtual FieldGradient5Vec computeFieldGradient5FromCurrentsCached(
      const CurrentsVec& currents) const override;

 private:
  ActuationMat act_mat_cached_;
  PositionVec position_cached_;
};

}  // namespace mag_manip
