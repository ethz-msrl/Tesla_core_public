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

#include "mag_manip/helpers.h"
#include "mag_manip/types.h"

namespace mag_manip {

/**
 * @brief Calculates the torque exerted on a magnetic dipole in an external field
 *
 * Implements the equation:
 * \f[
 * \mathbf{t} = \mathbf{m} \times \mathbf{b}
 * \f]
 *
 * @param moment: the magnetic moment in A.m^2
 * @param field in T
 *
 * @return The torque in N.m
 */
inline TorqueVec torqueOnDipole(const DipoleVec& moment, const FieldVec& field) {
  return moment.cross(field);
}

/**
 * @brief Computes the force exterted on a magnetic dipole in an external field gradient
 *
 * Implements the equation:
 * \f[
 * \mathbf{f} = \nabla ( \mathbf{m} . \mathbf{b} )
 * \f]
 *
 * @param moment: the magnetic moment in A.m^2
 * @param gradient5: 5D vector in T/m
 *
 * @return the force in N
 */
inline ForceVec forceOnDipole(const DipoleVec& moment, const Gradient5Vec& gradient5) {
  double moment_mag = moment.norm();
  DipoleVec moment_hat = moment.normalized();
  ForceVec force = moment_mag * directedGradient3Mat(moment_hat) * gradient5;
  return force;
}
}  // namespace mag_manip
