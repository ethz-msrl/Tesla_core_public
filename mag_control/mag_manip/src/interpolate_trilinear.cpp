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

#include "mag_manip/interpolate_trilinear.h"

using namespace mag_manip;
using namespace Eigen;
using namespace std;

InterpolateTrilinear::InterpolateTrilinear(const DataMat& data, const VFieldGridProperties& props)
    : InterpolateRegular(data, props) {}

FieldVec InterpolateTrilinear::interpolateImpl(const PositionVec& p) const {
  PositionVec p_n = getNormalizedPosition(p);
  return trilinearN(p_n);
}

GradientMat InterpolateTrilinear::getGradientImpl(const PositionVec& p) const {
  PositionVec p_n = getNormalizedPosition(p);
  return gradientN(p);
}

FieldVec InterpolateTrilinear::trilinearN(const PositionVec& p) const {
  // see http://paulbourke.net/miscellaneous/interpolation/

  // normalized position
  PositionVec::Scalar x_ix = p(0);
  PositionVec::Scalar y_ix = p(1);
  PositionVec::Scalar z_ix = p(2);

  // ceiling
  int x_c = std::ceil(x_ix);
  int y_c = std::ceil(y_ix);
  int z_c = std::ceil(z_ix);

  // floor
  int x_f = std::floor(x_ix);
  int y_f = std::floor(y_ix);
  int z_f = std::floor(z_ix);

  // delta
  PositionVec::Scalar x_d = (x_ix - x_f);
  PositionVec::Scalar y_d = (y_ix - y_f);
  PositionVec::Scalar z_d = (z_ix - z_f);

  FieldVec p_000 = getAtIndex(x_f, y_f, z_f);
  FieldVec p_100 = getAtIndex(x_c, y_f, z_f);
  FieldVec p_010 = getAtIndex(x_f, y_c, z_f);
  FieldVec p_001 = getAtIndex(x_f, y_f, z_c);
  FieldVec p_101 = getAtIndex(x_c, y_f, z_c);
  FieldVec p_011 = getAtIndex(x_f, y_c, z_c);
  FieldVec p_110 = getAtIndex(x_c, y_c, z_f);
  FieldVec p_111 = getAtIndex(x_c, y_c, z_c);

  return p_000 * (1 - x_d) * (1 - y_d) * (1 - z_d) + p_100 * x_d * (1 - y_d) * (1 - z_d) +
         p_010 * (1 - x_d) * y_d * (1 - z_d) + p_001 * (1 - x_d) * (1 - y_d) * z_d +
         p_101 * x_d * (1 - y_d) * z_d + p_011 * (1 - x_d) * y_d * z_d +
         p_110 * x_d * y_d * (1 - z_d) + p_111 * x_d * y_d * z_d;
}

GradientMat InterpolateTrilinear::gradientN(const PositionVec& p) const {
  // normalized position
  PositionVec::Scalar x_ix = p(0);
  PositionVec::Scalar y_ix = p(1);
  PositionVec::Scalar z_ix = p(2);

  // ceiling
  int x_c = std::ceil(x_ix);
  int y_c = std::ceil(y_ix);
  int z_c = std::ceil(z_ix);

  // floor
  int x_f = std::floor(x_ix);
  int y_f = std::floor(y_ix);
  int z_f = std::floor(z_ix);

  FieldVec p_000 = getAtIndex(x_f, y_f, z_f);
  FieldVec p_100 = getAtIndex(x_c, y_f, z_f);
  FieldVec p_010 = getAtIndex(x_f, y_c, z_f);
  FieldVec p_001 = getAtIndex(x_f, y_f, z_c);
  FieldVec p_101 = getAtIndex(x_c, y_f, z_c);
  FieldVec p_011 = getAtIndex(x_f, y_c, z_c);
  FieldVec p_110 = getAtIndex(x_c, y_c, z_f);
  FieldVec p_111 = getAtIndex(x_c, y_c, z_c);

  GradientMat G;
  G.col(0) << (p_100 - p_000) / grid_props_.step_x;
  G.col(1) << (p_010 - p_000) / grid_props_.step_y;
  G.col(2) << (p_001 - p_000) / grid_props_.step_z;
  return G;
}
