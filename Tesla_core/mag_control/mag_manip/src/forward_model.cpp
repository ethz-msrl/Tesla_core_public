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

#include "mag_manip/forward_model.h"
#include "mag_manip/exceptions.h"

using namespace std;
using namespace mag_manip;

FieldVecs ForwardModel::computeFieldsFromCurrents(const PositionVecs& positions,
                                                  const CurrentsVec& currents) const {
  const int N = positions.cols();
  FieldVecs fields(3, N);
  if (currents.IsVectorAtCompileTime) {
    for (int i = 0; i < N; i++) {
      fields.col(i) = computeFieldFromCurrents(positions.col(i), currents);
    }
  } else {
    if (currents.cols() != N) {
      throw InvalidInput(
          "Currents should either have a single column or the same number of columns as positions");
    }
    for (int i = 0; i < N; i++) {
      fields.col(i) = computeFieldFromCurrents(positions.col(i), currents.col(i));
    }
  }
  return fields;
}

Gradient5Vecs ForwardModel::computeGradient5sFromCurrents(const PositionVecs& positions,
                                                          const CurrentsVec& currents) const {
  const int N = positions.cols();
  Gradient5Vecs gradients(5, N);
  if (currents.IsVectorAtCompileTime) {
    for (int i = 0; i < N; i++) {
      gradients.col(i) = computeGradient5FromCurrents(positions.col(i), currents);
    }
  } else {
    if (currents.cols() != N) {
      throw InvalidInput(
          "Currents should either have a single column or the same number of columns as positions");
    }
    for (int i = 0; i < N; i++) {
      gradients.col(i) = computeGradient5FromCurrents(positions.col(i), currents.col(i));
    }
  }

  return gradients;
}

FieldGradient5Vecs ForwardModel::computeFieldGradient5sFromCurrents(
    const PositionVecs& positions, const CurrentsVec& currents) const {
  const int N = positions.cols();
  FieldGradient5Vecs field_gradients(8, N);

  if (currents.IsVectorAtCompileTime) {
    for (int i = 0; i < N; i++) {
      field_gradients.col(i) = computeFieldGradient5FromCurrents(positions.col(i), currents);
    }
  } else {
    if (currents.cols() != N) {
      throw InvalidInput(
          "Currents should either have a single column or the same number of columns as positions");
    }
    for (int i = 0; i < N; i++) {
      field_gradients.col(i) = computeFieldGradient5FromCurrents(positions.col(i), currents.col(i));
    }
  }

  return field_gradients;
}

void ForwardModel::setCachedPosition(const PositionVec& position) {
  throw NotImplementedException();
}

PositionVec ForwardModel::getCachedPosition() const { throw NotImplementedException(); }

FieldVec ForwardModel::computeFieldFromCurrentsCached(const CurrentsVec& currents) const {
  throw NotImplementedException();
}

Gradient5Vec ForwardModel::computeGradient5FromCurrentsCached(const CurrentsVec& currents) const {
  throw NotImplementedException();
}

FieldGradient5Vec ForwardModel::computeFieldGradient5FromCurrentsCached(
    const CurrentsVec& currents) const {
  throw NotImplementedException();
}
