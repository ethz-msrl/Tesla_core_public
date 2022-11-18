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

#include <Eigen/Dense>
#include <memory>
#include <ostream>
#include <string>
#include <tuple>
#include <utility>
#include <vector>
#include "mag_manip/types.h"
#include "mag_manip/vfield_grid_properties.h"

namespace YAML {
class Node;
}

namespace mag_manip {
/**
 * @brief Gets the properties of the regular grid and the positions and vectors
 * of the vector field
 *
 * The returned magnetic field data is supplied as a 3 x N matrix where
 * N = Nx * Ny * Nz is the total number of positions in the 3D grid of size Nx, Ny, Nz
 *
 * The data is assumed to be ordered in XYZ where the Z axis is changing most rapidly
 * and the X axis is changing the most slowly
 *
 * @param filename: the filename of the Aeon vector field
 *
 * @return a pair containing the vector data and the properties of the regular grid
 */
std::pair<DataMat, VFieldGridProperties> parseVFieldFile(const std::string& filename);

/**
 * @brief Returns true if the a position is enclosed within the grid
 *
 * @param position: the position to check
 * @param v_props: the properties of the grid
 *
 * @return true if the position is contained within the grid defined by v_props
 */
bool pointInVFieldWorkspace(const PositionVec& position,
                            const std::vector<VFieldGridProperties>& v_props);

/**
 * @brief Gets only the properties of the regular grid from a YAML file definition
 *
 * @param node: the YAML description of the regular grid (using yaml-cpp)
 *
 * @return the parsed grid properties
 */
VFieldGridProperties parseVFieldGridPropertiesYAML(const YAML::Node& node);

/**
 * @brief Parses a YAML file definition of the vector field
 *
 * @param node: the YAML description of the vector field (using yaml-cpp)
 *
 * @return a pair containing the vector data and the properties of the regular grid
 */
std::pair<DataMat, VFieldGridProperties> parseVFieldYAML(const YAML::Node& node);

/**
 * @brief Returns the positions of the grid points as a matrix.
 *
 * The returned position data is supplied as a 3 x N matrix where
 * N = Nx * Ny * Nz is the total number of positions in the 3D grid of size Nx, Ny, Nz
 *
 * The data is assumed to be ordered in XYZ where the Z axis is changing most rapidly
 * and the X axis is changing the most slowly
 *
 * @param props: the parameters of the regular grid
 *
 * @return the positions in the grid defined by props
 */
PositionVecs getGridPositions(const VFieldGridProperties& props);

std::ostream& operator<<(std::ostream& os, VFieldGridProperties& props);

}  // namespace mag_manip
