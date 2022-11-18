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

#include "mag_manip/backward_model.h"
#include "mag_manip/exceptions.h"

using namespace std;
using namespace mag_manip;

CurrentsVecs BackwardModel::computeCurrentsFromFields(const PositionVecs& positions,
                                                      const FieldVecs& fields) const {
  const int N = positions.cols();
  if (fields.cols() != N) {
    throw InvalidInput(
        "The number of cols in fields does not match the number of cols in positions");
  }
  const int Ne = getNumCoils();
  CurrentsVecs currents(Ne, N);
  for (int i = 0; i < N; i++) {
    currents.col(i) = computeCurrentsFromField(positions.col(i), fields.col(i));
  }
  return currents;
}

CurrentsVecs BackwardModel::computeCurrentsFromFieldGradient5s(
    const PositionVecs& positions, const FieldVecs& fields, const Gradient5Vecs& gradients) const {
  const int N = positions.cols();
  if (fields.cols() != N) {
    throw InvalidInput(
        "The number of cols in fields does not match the number of cols in positions");
  }
  if (gradients.cols() != 3) {
    throw InvalidInput(
        "The number of cols in gradients does not match the number of cols in positions");
  }

  const int Ne = getNumCoils();
  CurrentsVecs currents(Ne, N);

  for (int i = 0; i < N; i++) {
    currents.col(i) =
        computeCurrentsFromFieldGradient5(positions.col(i), fields.col(i), gradients.col(i));
  }
  return currents;
}

CurrentsVecs BackwardModel::computeCurrentsFromFieldDipoleGradient3s(
    const PositionVecs& positions, const FieldVecs& fields, const DipoleVecs& dipoles,
    const Gradient3Vecs& gradients) const {
  const int N = positions.cols();
  if (fields.cols() != N) {
    throw InvalidInput(
        "The number of cols in fields does not match the number of cols in positions");
  }
  if (gradients.cols() != N) {
    throw InvalidInput(
        "The number of cols in gradients does not match the number of cols in positions");
  }

  if (dipoles.cols() != N) {
    throw InvalidInput(
        "The number of cols in gradients does not match the number of cols in positions");
  }

  const int Ne = getNumCoils();
  CurrentsVecs currents(Ne, N);
  for (int i = 0; i < N; i++) {
    currents.col(i) = computeCurrentsFromFieldDipoleGradient3(positions.col(i), fields.col(i),
                                                              dipoles.col(i), gradients.col(i));
  }

  return currents;
}

void BackwardModel::setCachedPosition(const PositionVec& position) {
  throw NotImplementedException();
}

PositionVec BackwardModel::getCachedPosition() const { throw NotImplementedException(); }

void BackwardModel::setCachedPositionDipole(const PositionVec& position, const DipoleVec& dipole) {
  throw NotImplementedException();
}

DipoleVec BackwardModel::getCachedDipole() const { throw NotImplementedException(); }

CurrentsVec BackwardModel::computeCurrentsFromFieldCached(const FieldVec& field) const {
  throw NotImplementedException();
}

CurrentsVec BackwardModel::computeCurrentsFromFieldGradient5Cached(
    const FieldVec& field, const Gradient5Vec& gradient) const {
  throw NotImplementedException();
}

CurrentsVec BackwardModel::computeCurrentsFromFieldDipoleGradient3Cached(
    const FieldVec& field, const Gradient3Vec& gradient) const {
  throw NotImplementedException();
}
