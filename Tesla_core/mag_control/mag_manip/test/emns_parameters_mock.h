/* * Tesla - A ROS-based framework for performing magnetic manipulation
 *
 * Copyright 2018 Multi Scale Robotics Lab
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

#include <string>
#include "mag_manip/emns_parameters.h"

namespace mag_manip {

class EMNSParametersMock : public EMNSParameters {
 public:
  virtual std::string getSystemName() const { return "dummy"; }

  virtual int getNumElectromagnets() const { return 3; }

  virtual const CurrentsVec getMaxCurrents() const { return Eigen::Vector3d(8, 8, 8); }

  virtual double getMaxPower() const { return 15000.; }

  virtual const ResistancesVec getCoilResistances() const { return Eigen::Vector3d(8, 8, 8); }
};
}  // namespace mag_manip
