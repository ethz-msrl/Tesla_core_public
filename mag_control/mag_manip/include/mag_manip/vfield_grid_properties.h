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

namespace mag_manip {
/**
 * @brief Contains the properties of a regular grid for interpolation
 */
class VFieldGridProperties {
 public:
  VFieldGridProperties(const float min_x, const float max_x, const float min_y, const float max_y,
                       const float min_z, const float max_z, const int dim_x, const int dim_y,
                       const int dim_z)
      : min_x(min_x),
        max_x(max_x),
        min_y(min_y),
        max_y(max_y),
        min_z(min_z),
        max_z(max_z),
        dim_x(dim_x),
        dim_y(dim_y),
        dim_z(dim_z),
        step_x((max_x - min_x) / (dim_x - 1)),
        step_y((max_y - min_y) / (dim_y - 1)),
        step_z((max_z - min_z) / (dim_z - 1)) {}

  VFieldGridProperties& operator=(const VFieldGridProperties&) = delete;

  const float min_x;  /**< The minimum value of the x coordinate of the grid */
  const float min_y;  /**< The minimum value of the y coordinate of the grid */
  const float min_z;  /**< The minimum value of the z coordinate of the grid */
  const float max_x;  /**< The maximum value of the x coordinate of the grid */
  const float max_y;  /**< The maximum value of the y coordinate of the grid */
  const float max_z;  /**< The maximum value of the z coordinate of the grid */
  const int dim_x;    /**< The number of elements in the x direction of the grid */
  const int dim_y;    /**< The number of elements in the y direction of the grid */
  const int dim_z;    /**< The number of elements in the z direction of the grid */
  const float step_x; /**< The size between cells in the x direction */
  const float step_y; /**< The size between cells in the y direction */
  const float step_z; /**< The size between cells in the z direction */
};
}  // namespace mag_manip
