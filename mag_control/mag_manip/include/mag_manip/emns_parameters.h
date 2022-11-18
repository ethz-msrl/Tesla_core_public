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
class EMNSParameters {
 public:
  typedef std::shared_ptr<EMNSParameters> Ptr;

  virtual ~EMNSParameters() {}

  /**
   * @brief Returns the name of the system
   *
   * @return the eMNS system name
   */
  virtual std::string getSystemName() const = 0;

  /**
   * @brief Returns the number of electromagnets in the system
   *
   * @return the number of electromagnets
   */
  virtual int getNumElectromagnets() const = 0;

  /**
   * @brief Returns the maximum electrical current that can be run on each electromagnet
   *
   * The maximum current is always positive. It is assumed that the electromagnets can supply
   * both -imax and imax
   *
   * @return a vector with the maximum current in A that each electromagnet can handle
   */
  virtual const CurrentsVec getMaxCurrents() const = 0;

  /**
   * @brief Returns the maximum power that can be run on the entire system
   *
   * The power is always positive.
   *
   * @return the maximum power in W
   */
  virtual double getMaxPower() const = 0;

  /**
   * @brief Returns the resistance of each electromagnet
   *
   * The resistance is always positive
   *
   * @return a vector with each electromagnet resistance in Ohms
   */
  virtual const ResistancesVec getCoilResistances() const = 0;
};
}  // namespace mag_manip
