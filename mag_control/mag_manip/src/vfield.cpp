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

#include <cmath>
#include <fstream>
#include <iostream>
#include <vector>

#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>

#include <tsc_utils/compare.h>
#include "mag_manip/exceptions.h"
#include "mag_manip/vfield.h"

using namespace std;
using namespace Eigen;
using namespace tesla;

namespace mag_manip {

pair<DataMat, VFieldGridProperties> parseVFieldFile(const string& filename) {
  ifstream f(filename, ios::in);

  if (!f.is_open()) {
    throw InvalidFile(filename, "");
  }

  int num_elements;
  f >> num_elements;

  // the first line should be the number of elements in the first, second and
  // third columns respectively
  int dim_x, dim_y, dim_z;
  f >> dim_x >> dim_y >> dim_z;

  if ((dim_x * dim_y * dim_z) != num_elements) {
    throw InvalidCalibration("Number of elements not equal to product of dimensions");
  }

  DataMat d_positions(3, num_elements);
  DataMat mag_fields(3, num_elements);

  float min_coord_x, min_coord_y, min_coord_z;
  float max_coord_x, max_coord_y, max_coord_z;
  f >> min_coord_x >> max_coord_x;
  f >> min_coord_y >> max_coord_y;
  f >> min_coord_z >> max_coord_z;

  // this is just used to check that the step sizes are equal amongst dimensions.
  // That's really a fault of the Aeon vfield file structure which doesn't allow different step
  // sizes per dimension
  float step_x = (max_coord_x - min_coord_x) / (dim_x - 1);
  float step_y = (max_coord_y - min_coord_y) / (dim_y - 1);
  float step_z = (max_coord_z - min_coord_z) / (dim_z - 1);

  if (!almost_equal(step_x, step_y, 6) || !almost_equal(step_y, step_z, 6)) {
    throw InvalidCalibration("Step sizes are not equal");
  }

  if (almost_equal(step_x, 0.f, 6)) {
    throw InvalidCalibration("Step size cannot be 0");
  }

  VFieldGridProperties props(min_coord_x, max_coord_x, min_coord_y, max_coord_y, min_coord_z,
                             max_coord_z, dim_x, dim_y, dim_z);

  for (int i = 0; i < dim_x; i++) {
    for (int j = 0; j < dim_y; j++) {
      for (int k = 0; k < dim_z; k++) {
        int idx = i * dim_x * dim_y + j * dim_y + k;
        PositionVec p, v;
        f >> p(0) >> p(1) >> p(2);
        d_positions.col(idx) = p;
        f >> v(0) >> v(1) >> v(2);
        mag_fields.col(idx) = v;
      }
    }
  }

  return {mag_fields, props};
}

bool pointInVFieldWorkspace(const PositionVec& position,
                            const vector<VFieldGridProperties>& v_props) {
  bool ret = true;
  for (auto& props : v_props) {
    ret &= position(0) >= props.min_x && position(0) <= props.max_x && position(1) >= props.min_y &&
           position(1) <= props.max_y && position(2) >= props.min_z && position(2) <= props.max_z;
  }

  return ret;
}

VFieldGridProperties parseVFieldGridPropertiesYAML(const YAML::Node& node) {
  const auto dim_x = node["dim_x"].as<int>();
  const auto dim_y = node["dim_y"].as<int>();
  const auto dim_z = node["dim_z"].as<int>();

  const auto min_x = node["min_x"].as<float>();
  const auto max_x = node["max_x"].as<float>();

  if (min_x >= max_x) {
    throw InvalidCalibration("min_x must be smaller than max_x");
  }

  const auto min_y = node["min_y"].as<float>();
  const auto max_y = node["max_y"].as<float>();
  if (min_y >= max_y) {
    throw InvalidCalibration("min_y must be smaller than max_y");
  }

  const auto min_z = node["min_z"].as<float>();
  const auto max_z = node["max_z"].as<float>();
  if (min_z >= max_z) {
    throw InvalidCalibration("min_z must be smaller than max_z");
  }

  return VFieldGridProperties(min_x, max_x, min_y, max_y, min_z, max_z, dim_x, dim_y, dim_z);
}

std::pair<DataMat, VFieldGridProperties> parseVFieldYAML(const YAML::Node& node) {
  VFieldGridProperties props = parseVFieldGridPropertiesYAML(node);

  const int num_elements = props.dim_x * props.dim_y * props.dim_z;
  DataMat data(3, num_elements);

  YAML::Node n_data = node["data"];
  if (!n_data.IsSequence() || n_data.size() != num_elements) {
    throw InvalidCalibration("Invalid data");
  }

  int col_idx = 0;
  for (YAML::const_iterator d_it = n_data.begin(); d_it != n_data.end(); d_it++) {
    if (!d_it->IsSequence() || d_it->size() != 3) {
      throw InvalidCalibration("Invalid data point");
    }
    data.col(col_idx) << (*d_it)[0].as<float>(), (*d_it)[1].as<float>(), (*d_it)[2].as<float>();
    col_idx += 1;
  }

  return {data, props};
}

PositionVecs getGridPositions(const VFieldGridProperties& props) {
  PositionVecs positions(3, props.dim_x * props.dim_y * props.dim_z);
  for (int i = 0; i < props.dim_x; i++) {
    for (int j = 0; j < props.dim_y; j++) {
      for (int k = 0; k < props.dim_z; k++) {
        int index = i * props.dim_y * props.dim_z + j * props.dim_z + k;
        positions.col(index) =
            Eigen::Vector3d(props.min_x + i * props.step_x, props.min_y + j * props.step_y,
                            props.min_z + k * props.step_z);
      }
    }
  }
  return positions;
}

std::ostream& operator<<(std::ostream& os, VFieldGridProperties& props) {
  os << "dim_x: " << props.dim_x << std::endl;
  os << "dim_y: " << props.dim_y << std::endl;
  os << "dim_z: " << props.dim_z << std::endl;
  os << "min_x: " << props.min_x << std::endl;
  os << "max_x: " << props.max_x << std::endl;
  os << "min_y: " << props.min_y << std::endl;
  os << "max_y: " << props.max_y << std::endl;
  os << "min_z: " << props.min_z << std::endl;
  os << "max_z: " << props.max_z << std::endl;
  return os;
}

}  // namespace mag_manip
