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

#include "mag_manip/emns_parameters.h"

namespace mag_manip {

class EMNSParametersYAML : public EMNSParameters {
 public:
  typedef std::shared_ptr<EMNSParametersYAML> Ptr;

  /**
   * @brief Factory function to create a params object from a YAML filename
   *
   * @param filename: path to the YAML config file
   *
   * @return the constructed parameter object
   */
  static EMNSParametersYAML fromFile(const std::string& filename);

  /**
   * @brief Factory function to create a pointer to a  params object from a YAML filename
   *
   * @param filename: path to the YAML config file
   *
   * @return the constructed parameter object
   */
  static EMNSParametersYAML::Ptr ptrFromFile(const std::string& filename);

  /**
   * @brief Constructor. Calls EMNSParametersYAML::load internally.
   *
   * @param config_s: the YAML parameters stored as a string
   */
  explicit EMNSParametersYAML(const std::string& config_s);

  void operator=(const EMNSParametersYAML&) = delete;

  virtual std::string getSystemName() const;

  virtual int getNumElectromagnets() const;

  virtual const CurrentsVec getMaxCurrents() const;

  virtual double getMaxPower() const;

  virtual const ResistancesVec getCoilResistances() const;

 private:
  void load(const std::string& config_s);

  std::string system_name_;
  int ne_;
  CurrentsVec max_currents_;
  double max_power_;
  ResistancesVec coil_res_;
};
}  // namespace mag_manip
