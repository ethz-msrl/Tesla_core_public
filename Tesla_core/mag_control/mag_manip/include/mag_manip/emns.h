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

#include <algorithm>

#pragma once

namespace mag_manip {
/**
 * @brief Calculates the maximum magnetic field vector that is parallel to the target field but with
 * electromagnet currents not larger than the vector of maximum electromagnet currents.
 *
 * This only works with linear models, since it assumes that all the components of the field scale
 * the same way
 * with a change in current.
 *
 * This assumes that the current maximums are strictly positive, and that for maximum current i_k of
 * electromagnet k,
 * the current can range between [-i_k, i_k]
 *
 * The field is always returned in the positive direction of target_field. Ie for [0, 0, 0.5], the
 * max field will be * [0,0,1] and not [0,0,-1]
 *
 * @see getMaxFieldMagnitudeAlignedWithTargetField()
 * @param bmodel: the backward model used to calculate the magnetic field
 * @param position: the position at which target_field is defined
 * @param target_field: the 3D field along which to compute the max field magnitude
 * @param max_currents: a vector containing the maximum allowed currents in each electromagnet in A
 *
 * @return scaled target_field such that one of the electromagnets is maxed out
 */
inline FieldVec getMaxFieldAlignedWithTargetField(const BackwardModel& bmodel,
                                                  const PositionVec& position,
                                                  const FieldVec& target_field,
                                                  const CurrentsVec& max_currents) {
  CurrentsVec currents_target = bmodel.computeCurrentsFromField(position, target_field);

  // this finds the max ratio of target max current to admissible max current (must be between -1
  // and 1)
  double max_scale = (currents_target.cwiseQuotient(max_currents)).maxCoeff();
  double min_scale = (currents_target.cwiseQuotient(max_currents)).minCoeff();

  double scale = (std::abs(min_scale) > std::abs(max_scale)) ? std::abs(min_scale) : max_scale;

  return target_field / scale;
}

/**
 * @brief Overload the function getMaxFieldAlignedWithTargetField() to respect a maximum power.
 *
 * @overload
 * @param coil_resistances: a vector containing the resistances of the coils in Ohms
 * @param max_power: a double maximum allowed power of the eMNS in W
 *
 * @return scaled target_field such that one of the electromagnets is maxed out
 */
inline FieldVec getMaxFieldAlignedWithTargetField(
    const BackwardModel& bmodel, const PositionVec& position, const FieldVec& target_field,
    const CurrentsVec& max_currents, const ResistancesVec& coil_resistances, double max_power) {
  CurrentsVec currents_target = bmodel.computeCurrentsFromField(position, target_field);

  // this finds the max ratio of target max current to admissible max current (must be between -1
  // and 1)
  double max_scale_current = (currents_target.cwiseQuotient(max_currents)).maxCoeff();
  double min_scale_current = (currents_target.cwiseQuotient(max_currents)).minCoeff();

  double scale_current = (std::abs(min_scale_current) > std::abs(max_scale_current))
                             ? std::abs(min_scale_current)
                             : max_scale_current;

  // this computes the power of the target currents
  double power = coil_resistances.dot(currents_target.cwiseProduct(currents_target));

  double scale_power = std::sqrt(power / max_power);

  // this scale down the field with the more stringent condition between max current and max power
  double scale = std::max(scale_current, scale_power);

  return target_field / scale;
}

/**
 * @brief Calculates the maximum magnetic field magnitude along the magnetic field direction
 * specified by a target field
 *
 * This only works with linear models, since it assumes that all the components of the field scale
 * the same way
 * with a change in current.
 *
 * @param bmodel: the backward model used to calculate the magnetic field
 * @param position: the position at which target_field is defined
 * @param target_field: the 3D field along which to compute the max field magnitude
 * @param max_currents: a vector containing the maximum allowed currents in each electromagnet in A
 *
 * @return maximum allowable field magnitude in T
 */
inline double getMaxFieldMagnitudeAlignedWithTargetField(const BackwardModel& bmodel,
                                                         const PositionVec& position,
                                                         const FieldVec& target_field,
                                                         const CurrentsVec& max_currents) {
  auto field_scaled =
      getMaxFieldAlignedWithTargetField(bmodel, position, target_field, max_currents);
  return field_scaled.norm();
}

/**
 * @brief Overload the function getMaxFieldMagnitudeAlignedWithTargetField() to respect a maximum
 * power
 *
 * @overload
 * @param coil_resistances: a vector containing the resistances of the coils in Ohms
 * @param max_power: a double maximum allowed power of the eMNS in W
 *
 * @return maximum allowable field magnitude in T
 */
inline double getMaxFieldMagnitudeAlignedWithTargetField(
    const BackwardModel& bmodel, const PositionVec& position, const FieldVec& target_field,
    const CurrentsVec& max_currents, const ResistancesVec& coil_resistances, double max_power) {
  auto field_scaled = getMaxFieldAlignedWithTargetField(bmodel, position, target_field,
                                                        max_currents, coil_resistances, max_power);
  return field_scaled.norm();
}

}  // namespace mag_manip
