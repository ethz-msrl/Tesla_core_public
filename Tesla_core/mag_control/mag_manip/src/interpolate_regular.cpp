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

#include "mag_manip/interpolate_regular.h"
using namespace std;

namespace mag_manip {

FieldVec InterpolateRegular::interpolate(const PositionVec& p) const {
  if (!isInBounds(p)) {
    throw OutsideBounds(p);
  }
  return interpolateImpl(p);
}

GradientMat InterpolateRegular::getGradient(const PositionVec& p) const {
  if (!isInBounds(p)) {
    throw OutsideBounds(p);
  }
  return getGradientImpl(p);
}

FieldVec InterpolateRegular::getDataMatD_dx(const int ix, const int jy, const int kz) const {
  double cx = 0;
  int il, ir;
  // if at the minimum, use forward difference
  if (ix == 0) {
    cx = 1;
    il = 1;
    ir = 0;
    // if at the maximum, use the backward difference
  } else if (ix == (grid_props_.dim_x - 1)) {
    cx = 1;
    il = grid_props_.dim_x - 1;
    ir = grid_props_.dim_x - 2;
    // or use the central difference
  } else if (ix > 0 && ix < (grid_props_.dim_x - 1)) {
    cx = 0.5;
    il = ix + 1;
    ir = ix - 1;
  } else {
    throw runtime_error("Invalid index ix: " + to_string(ix));
  }

  return cx * (getAtIndex(il, jy, kz) - getAtIndex(ir, jy, kz));
}

FieldVec InterpolateRegular::getDataMatD_dy(const int ix, const int jy, const int kz) const {
  double cy = 0;
  int jl, jr;
  // if at the minimum, use forward difference
  if (jy == 0) {
    cy = 1;
    jl = 1;
    jr = 0;
    // if at the maximum, use the backward difference
  } else if (jy == (grid_props_.dim_y - 1)) {
    cy = 1;
    jl = grid_props_.dim_y - 1;
    jr = grid_props_.dim_y - 2;
    // or use the central difference
  } else if (jy > 0 && jy < (grid_props_.dim_y - 1)) {
    cy = 0.5;
    jl = jy + 1;
    jr = jy - 1;
  } else {
    throw runtime_error("Invalid index jy: " + to_string(jy));
  }

  return cy * (getAtIndex(ix, jl, kz) - getAtIndex(ix, jr, kz));
}

FieldVec InterpolateRegular::getDataMatD_dz(const int ix, const int jy, const int kz) const {
  double cz = 0;
  int kl, kr;
  // if at the minimum, use forward difference
  if (kz == 0) {
    cz = 1;
    kl = 1;
    kr = 0;
    // if at the maximum, use the backward difference
  } else if (kz == (grid_props_.dim_z - 1)) {
    cz = 1;
    kl = grid_props_.dim_z - 1;
    kr = grid_props_.dim_z - 2;
    // or use the central difference
  } else if (kz > 0 && kz < (grid_props_.dim_z - 1)) {
    cz = 0.5;
    kl = kz + 1;
    kr = kz - 1;
  } else {
    throw runtime_error("Invalid index kz: " + to_string(kz));
  }

  return cz * (getAtIndex(ix, jy, kl) - getAtIndex(ix, jy, kr));
}

FieldVec InterpolateRegular::getDataMatD2_dxdy(const int ix, const int jy, const int kz) const {
  double cx;
  int il, ir;
  // if at the minimum, use forward difference
  if (ix == 0) {
    il = 1;
    ir = 0;
    cx = 1;
    // if at the maximum, use the backward difference
  } else if (ix == grid_props_.dim_x - 1) {
    il = grid_props_.dim_x - 1;
    ir = grid_props_.dim_x - 2;
    cx = 1;
    // or use the central difference
  } else if (ix > 0 && ix < (grid_props_.dim_x - 1)) {
    il = ix + 1;
    ir = ix - 1;
    cx = 0.5;
  } else {
    throw runtime_error("Invalid index ix: " + to_string(ix));
  }

  double cy;
  int jl, jr;
  if (jy == 0) {
    jl = 1;
    jr = 0;
    cy = 1;
    // if at the maximum, use the backward difference
  } else if (jy == grid_props_.dim_y - 1) {
    jl = grid_props_.dim_y - 1;
    jr = grid_props_.dim_y - 2;
    cy = 1;
    // or use the central difference
  } else if (jy > 0 && jy < (grid_props_.dim_y - 1)) {
    jl = jy + 1;
    jr = jy - 1;
    cy = 0.5;
  } else {
    throw runtime_error("Invalid index jy: " + to_string(jy));
  }

  return cx * cy *
         (getAtIndex(il, jl, kz) - getAtIndex(ir, jl, kz) - getAtIndex(il, jr, kz) +
          getAtIndex(ir, jr, kz));
}

FieldVec InterpolateRegular::getDataMatD2_dxdz(const int ix, const int jy, const int kz) const {
  double cx;
  int il, ir;
  // if at the minimum, use forward difference
  if (ix == 0) {
    il = 1;
    ir = 0;
    cx = 1;
    // if at the maximum, use the backward difference
  } else if (ix == grid_props_.dim_x - 1) {
    il = grid_props_.dim_x - 1;
    ir = grid_props_.dim_x - 2;
    cx = 1;
    // or use the central difference
  } else if (ix > 0 && ix < (grid_props_.dim_x - 1)) {
    il = ix + 1;
    ir = ix - 1;
    cx = 0.5;
  } else {
    throw runtime_error("Invalid index ix: " + to_string(ix));
  }

  double cz;
  int kl, kr;
  if (kz == 0) {
    kl = 1;
    kr = 0;
    cz = 1;
    // if at the minimum, use forward difference
  } else if (kz == grid_props_.dim_z - 1) {
    kl = grid_props_.dim_z - 1;
    kr = grid_props_.dim_z - 2;
    cz = 1;
    // if at the maximum, use the backward difference
  } else if (kz > 0 && kz < (grid_props_.dim_z - 1)) {
    kl = kz + 1;
    kr = kz - 1;
    cz = 0.5;
    // or use the central difference
  } else {
    throw runtime_error("Invalid index kz: " + to_string(kz));
  }

  return cx * cz *
         (getAtIndex(il, jy, kl) - getAtIndex(ir, jy, kl) - getAtIndex(il, jy, kr) +
          getAtIndex(ir, jy, kr));
}

FieldVec InterpolateRegular::getDataMatD2_dydz(const int ix, const int jy, const int kz) const {
  double cy;
  int jl, jr;
  // if at the minimum, use forward difference
  if (jy == 0) {
    jl = 1;
    jr = 0;
    cy = 1;
    // if at the maximum, use the backward difference
  } else if (jy == grid_props_.dim_y - 1) {
    jl = grid_props_.dim_y - 1;
    jr = grid_props_.dim_y - 2;
    cy = 1;
    // or use the central difference
  } else if (jy > 0 && jy < (grid_props_.dim_y - 1)) {
    jl = jy + 1;
    jr = jy - 1;
    cy = 0.5;
  } else {
    throw runtime_error("Invalid index jy: " + to_string(jy));
  }

  double cz;
  int kl, kr;
  // if at the minimum, use forward difference
  if (kz == 0) {
    kl = 1;
    kr = 0;
    cz = 1;
    // if at the maximum, use the backward difference
  } else if (kz == grid_props_.dim_z - 1) {
    kl = grid_props_.dim_z - 1;
    kr = grid_props_.dim_z - 2;
    cz = 1;
    // or use the central difference
  } else if (kz > 0 && kz < (grid_props_.dim_z - 1)) {
    kl = kz + 1;
    kr = kz - 1;
    cz = 0.5;
  } else {
    throw runtime_error("Invalid index kz: " + to_string(kz));
  }

  return cy * cz *
         (getAtIndex(ix, jl, kl) - getAtIndex(ix, jr, kl) - getAtIndex(ix, jl, kr) +
          getAtIndex(ix, jr, kr));
}

FieldVec InterpolateRegular::getDataMatD3_dxdydz(const int ix, const int jy, const int kz) const {
  double cx;
  int il, ir;
  // if at the minimum, use forward difference
  if (ix == 0) {
    il = 1;
    ir = 0;
    cx = 1;
    // if at the maximum, use the backward difference
  } else if (ix == grid_props_.dim_x - 1) {
    il = grid_props_.dim_x - 1;
    ir = grid_props_.dim_x - 2;
    cx = 1;
    // or use the central difference
  } else if (ix > 0 && ix < (grid_props_.dim_x - 1)) {
    il = ix + 1;
    ir = ix - 1;
    cx = 0.5;
  } else {
    throw runtime_error("Invalid index ix: " + to_string(ix));
  }

  double cy;
  int jl, jr;
  if (jy == 0) {
    jl = 1;
    jr = 0;
    cy = 1;
    // if at the maximum, use the backward difference
  } else if (jy == grid_props_.dim_y - 1) {
    jl = grid_props_.dim_y - 1;
    jr = grid_props_.dim_y - 2;
    cy = 1;
    // or use the central difference
  } else if (jy > 0 && jy < (grid_props_.dim_y - 1)) {
    jl = jy + 1;
    jr = jy - 1;
    cy = 0.5;
  } else {
    throw runtime_error("Invalid index jy: " + to_string(jy));
  }

  double cz;
  int kl, kr;
  // if at the minimum, use forward difference
  if (kz == 0) {
    kl = 1;
    kr = 0;
    cz = 1;
    // if at the maximum, use the backward difference
  } else if (kz == grid_props_.dim_z - 1) {
    kl = grid_props_.dim_z - 1;
    kr = grid_props_.dim_z - 2;
    cz = 1;
    // or use the central difference
  } else if (kz > 0 && kz < (grid_props_.dim_z - 1)) {
    kl = kz + 1;
    kr = kz - 1;
    cz = 0.5;
  } else {
    throw runtime_error("Invalid index kz: " + to_string(kz));
  }

  return cx * cy * cz *
         (getAtIndex(il, jl, kl) - getAtIndex(ir, jl, kl) - getAtIndex(il, jr, kl) +
          getAtIndex(ir, jr, kl) - getAtIndex(il, jl, kr) + getAtIndex(ir, jl, kr) +
          getAtIndex(il, jr, kr) - getAtIndex(ir, jr, kr));
}
}  // namespace mag_manip
