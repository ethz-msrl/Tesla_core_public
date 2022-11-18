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
#include <exception>
#include <memory>
#include "mag_manip/exceptions.h"
#include "mag_manip/types.h"
#include "mag_manip/vfield_grid_properties.h"

namespace mag_manip {

/**
 * @brief Interface for interpolation methods that work on regular grids
 */
class InterpolateRegular {
 public:
  typedef std::unique_ptr<InterpolateRegular> UPtr;
  typedef std::shared_ptr<InterpolateRegular> Ptr;

  enum class Type { TRILINEAR, TRICUBIC, TRICUBIC_SCALAR_FIELD };

  /**
   * @brief Constructor for the interpolator
   *
   * data is a row-wise matrix containing the vector field values at the grid postions.
   * The total number of data points is N = dim_x * dim_y * dim_z.
   * The data is ordered first by z, y then x ie:
   *
   * for i=1:dim_x
   *  for j=1:dim_y
   *      for k=1:dim_z
   *          data <- v(i,j,k)
   *
   * Throws runtime_error if the data matrix is not of the right size
   *
   * @param data: Matrix of size 3xN holding the vector field values.
   * @param props: The properties of the regular interpolation grid.
   */
  InterpolateRegular(const DataMat& data, const VFieldGridProperties& props)
      : data_(data), grid_props_(props) {
    if (data.rows() != 3 || data.cols() != (props.dim_x * props.dim_y * props.dim_z)) {
      throw std::runtime_error("Invalid data matrix");
    }
  }

  virtual ~InterpolateRegular() {}

  /**
   * @brief Returns the VFieldGridProperties for this field
   *
   * @return the grid properties
   */
  VFieldGridProperties getVFieldGridProperties() const { return grid_props_; }

  /**
   * @brief Interpolates the vector field
   *
   * Checks if the position is within the bounds of the grid and throws an exception
   * if it is outside
   *
   * @param p: 3D position at which to interpolate field
   *
   * @return the 3D vector interpolated at p.
   */
  FieldVec interpolate(const PositionVec& p) const;

  /**
   * @brief Implementation of the gradient interpolation function
   *
   * @param p: 3D position at which to calculate gradient
   *
   * Checks if the position is within the bounds of the grid and throws an exception
   * if it is outside
   *
   * @return the gradient matrix
   * \f[
   * \begin{bmatrix} \frac{\partial V}{\partial x} &
   * \frac{\partial V}{\partial y} & \frac{\partial V}{\partial z}
   * \end{bmatrix}
   * \f]
   */
  GradientMat getGradient(const PositionVec& p) const;

 protected:
  /**
   * @brief Implementation of the interpolation function
   *
   * @param p: 3D position at which to interpolate field
   *
   * @return the 3D vector interpolated at p.
   */
  virtual FieldVec interpolateImpl(const PositionVec& p) const = 0;

  /**
   * @brief Implementation of the gradient interpolation function
   *
   * @param p: 3D position at which to calculate gradient
   *
   * @return the gradient matrix
   * \f[
   * \begin{bmatrix} \frac{\partial V}{\partial x} &
   * \frac{\partial V}{\partial y} & \frac{\partial V}{\partial z}
   * \end{bmatrix}
   * \f]
   */
  virtual GradientMat getGradientImpl(const PositionVec& p) const = 0;

  /**
   * @brief Checks if the position is in the interpolation grid
   *
   * @param position: a 3D position
   *
   * @return true if position is in the interpolation grid
   */
  bool isInBounds(const PositionVec& position) const {
    return position(0) >= grid_props_.min_x && position(0) <= grid_props_.max_x &&
           position(1) >= grid_props_.min_y && position(1) <= grid_props_.max_y &&
           position(2) >= grid_props_.min_z && position(2) <= grid_props_.max_z;
  }

  /**
   * @brief Normalizes the position to the bounds of the interpolation grid
   *
   * This normalization ensures that grid cells are at integer values between 0 and dim_x, dim_y, or
   * dim_z
   *
   * @param p: a 3D position in the grid
   *
   * @return the position normalized to lie between integer grid cell positions
   */
  inline PositionVec getNormalizedPosition(const PositionVec& p) const {
    return PositionVec((p(0) - grid_props_.min_x) / grid_props_.step_x,
                       (p(1) - grid_props_.min_y) / grid_props_.step_y,
                       (p(2) - grid_props_.min_z) / grid_props_.step_z);
  }

  /**
   * @brief Undos the normalization of getNormalizedPosition
   *
   * @param p: a 3D position in the normalized grid
   *
   * @return the 3D position in the regular grid
   */
  inline PositionVec getDenormalizedPosition(const PositionVec& p) const {
    return PositionVec((p(0) * grid_props_.step_x + grid_props_.min_x),
                       (p(1) * grid_props_.step_y + grid_props_.min_y),
                       (p(2) * grid_props_.step_z + grid_props_.min_z));
  }

  /**
   * @brief Computes the numerical derivative of the data at the following index
   *
   * The position must first be indexed by normalizing the position and taking the int floor
   * This also assumes that the indices are between 0 and the maximum index in each dimension or a
   * std::runtime_error is thrown
   *
   * The central difference is used when possible, and the forward and backward differences are used
   * on the edges of the grid
   *
   *
   * @param ix: coordinate of the x position
   * @param jy: coordinate of the y position
   * @param kz: coordinate of the z position
   *
   * @return a 3D vector containing the derivatives of each entry of the data matrix at the
   * corresponding index
   */
  FieldVec getDataMatD_dx(const int ix, const int jy, const int kz) const;

  /**
   * @brief Computes the numerical derivative of the data at the following index
   *
   * The position must first be indexed by normalizing the position and taking the int floor
   * This also assumes that the indices are between 0 and the maximum index in each dimension or a
   * std::runtime_error is thrown
   *
   * The central difference is used when possible, and the forward and backward differences are used
   * on the edges of the grid
   *
   *
   * @param ix: coordinate of the x position
   * @param jy: coordinate of the y position
   * @param kz: coordinate of the z position
   *
   * @return a 3D vector containing the derivatives of each entry of the data matrix at the
   * corresponding index
   */
  FieldVec getDataMatD_dy(const int ix, const int jy, const int kz) const;

  /**
   * @brief Computes the numerical derivative of the data at the following index
   *
   * The position must first be indexed by normalizing the position and taking the int floor
   * This also assumes that the indices are between 0 and the maximum index in each dimension or a
   * std::runtime_error is thrown
   *
   * The central difference is used when possible, and the forward and backward differences are used
   * on the edges of the grid
   *
   *
   * @param ix: coordinate of the x position
   * @param jy: coordinate of the y position
   * @param kz: coordinate of the z position
   *
   * @return a 3D vector containing the derivatives of each entry of the data matrix at the
   * corresponding index
   */
  FieldVec getDataMatD_dz(const int ix, const int jy, const int kz) const;

  /**
   * @brief Computes the numerical 2nd derivative of the data at the following index
   *
   * @param ix: coordinate of the x position
   * @param jy: coordinate of the y position
   * @param kz: coordinate of the z position
   *
   * The position must first be indexed by normalizing the position and taking the int floor
   * This also assumes that the indices are between 0 and the maximum index in each dimension or a
   * std::runtime_error is thrown
   *
   * The central difference is used when possible, and the forward and backward differences are used
   * on the edges of the grid
   *
   * @return a 3D vector containing the 2nd derivatives of each entry of the data matrix at the
   * corresponding index
   */
  FieldVec getDataMatD2_dxdy(const int ix, const int jy, const int kz) const;

  /**
   * @brief Computes the numerical 2nd derivative of the data at the following index
   *
   * @param ix: coordinate of the x position
   * @param jy: coordinate of the y position
   * @param kz: coordinate of the z position
   *
   * @return a 3D vector containing the 2nd derivatives of each entry of the data matrix at the
   * corresponding index
   */
  FieldVec getDataMatD2_dxdz(const int ix, const int jy, const int kz) const;

  /**
   * @brief Computes the numerical 2nd derivative of the data at the following index
   *
   * @param ix: coordinate of the x position
   * @param jy: coordinate of the y position
   * @param kz: coordinate of the z position
   *
   * The position must first be indexed by normalizing the position and taking the int floor
   * This also assumes that the indices are between 0 and the maximum index in each dimension or a
   * std::runtime_error is thrown
   *
   * The central difference is used when possible, and the forward and backward differences are used
   * on the edges of the grid
   *
   * @return a 3D vector containing the 2nd derivatives of each entry of the data matrix at the
   * corresponding index
   */
  FieldVec getDataMatD2_dydz(const int ix, const int jy, const int kz) const;

  /**
   * @brief Computes the numerical 3rd derivative of the data at the following index
   *
   * @param ix: coordinate of the x position
   * @param jy: coordinate of the y position
   * @param kz: coordinate of the z position
   *
   * The position must first be indexed by normalizing the position and taking the int floor
   * This also assumes that the indices are between 0 and the maximum index in each dimension or a
   * std::runtime_error is thrown
   *
   * The central difference is used when possible, and the forward and backward differences are used
   * on the edges of the grid
   *
   * @return a 3D vector containing the 2nd derivatives of each entry of the data matrix at the
   * corresponding index
   */
  FieldVec getDataMatD3_dxdydz(const int ix, const int jy, const int kz) const;

  /**
   * @brief Gets the data at an integer grid index.
   *
   * @param index: the grid cell index
   *
   * @return the data point at that index
   */
  inline FieldVec getAtIndex(int ix, int jy, int kz) const {
    const int col_idx = ix * grid_props_.dim_x * grid_props_.dim_y + jy * grid_props_.dim_y + kz;
    return data_.col(col_idx);
  }

  VFieldGridProperties grid_props_; /**< The regular grid properties */

  DataMat data_; /**< The vector field data */
};               // namespace mag_manip
}  // namespace mag_manip
