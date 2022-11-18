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

#include <benchmark/benchmark.h>
#include <ros/package.h>

#include "mag_manip/exceptions.h"
#include "mag_manip/forward_model_mpem.h"
#include "mag_manip/helpers.h"

using namespace std;
using namespace mag_manip;

static void bm_computeFieldFromCurrents(benchmark::State& state) {
  string cal_file = ros::package::getPath("mag_manip") + "/test/OctoMag_Calibration.yaml";
  ForwardModelMPEM model;
  model.setCalibrationFile(cal_file);
  PositionVec position(0, 0, 0);
  CurrentsVec currents = 10 * CurrentsVec::Random(8);

  while (state.KeepRunning()) {
    FieldVec field = model.computeFieldFromCurrents(position, currents);
  }
}

BENCHMARK(bm_computeFieldFromCurrents);

static void bm_computeGradient5FromCurrents(benchmark::State& state) {
  string cal_file = ros::package::getPath("mag_manip") + "/test/OctoMag_Calibration.yaml";
  ForwardModelMPEM model;
  model.setCalibrationFile(cal_file);
  PositionVec position(0, 0, 0);
  CurrentsVec currents = 10 * CurrentsVec::Random(8);

  while (state.KeepRunning()) {
    Gradient5Vec gradient = model.computeGradient5FromCurrents(position, currents);
  }
}

BENCHMARK(bm_computeGradient5FromCurrents);

// static void bm_computeCurrentsFromFieldGradient5(benchmark::State& state) {
// string cal_file = ros::package::getPath("mag_manip") + "/test/OctoMag_Calibration.yaml";
// BackwardModelMPEML2 model;
// model.setCalibrationFile(cal_file);
// PositionVec position(1e-12, 0, 0);
// FieldVec field(30e-3, 0, 0);
// Gradient3Vec gradient3(0.2, 0, 0);
// DipoleVec dipole(1, 0, 0);
// while (state.KeepRunning()) {
// CurrentsVec currents = model.computeCurrentsFromFieldDipoleGradient3(position,
// field, dipole, gradient3);
//}
//}

// BENCHMARK(bm_computeCurrentsFromFieldGradient5);

BENCHMARK_MAIN();
