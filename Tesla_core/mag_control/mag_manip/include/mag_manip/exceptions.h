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

#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "mag_manip/types.h"

namespace mag_manip {

class NotImplementedException : public std::logic_error {
 public:
  NotImplementedException() : std::logic_error("Function not yet implemented.") {}
};

class InvalidFile : public std::runtime_error {
 public:
  InvalidFile(std::string filename, std::string error)
      : std::runtime_error(make_what(filename, error)) {}

 private:
  static const std::string make_what(const std::string filename, const std::string error_msg) {
    std::stringstream ss;
    ss << "Invalid file " << filename << std::endl;
    ss << "Failed because: " << error_msg;
    return ss.str();
  }
};

class InvalidCalibration : public std::runtime_error {
 public:
  explicit InvalidCalibration(const std::string& what) : std::runtime_error(what) {}
};

class CalibrationNotLoaded : public std::runtime_error {
 public:
  CalibrationNotLoaded() : std::runtime_error("Calibration not loaded") {}
};

class OutsideBounds : public std::runtime_error {
 public:
#ifndef SWIG
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
#endif
  explicit OutsideBounds(const PositionVec& p) : std::runtime_error(make_what(p)) {}

 private:
  static const std::string make_what(const PositionVec& p) {
    std::stringstream ss;
    ss << "Position " << p.transpose() << " is out of bounds.";
    return ss.str();
  }
};

class InvalidCurrentsLength : public std::runtime_error {
 public:
  InvalidCurrentsLength()
      : std::runtime_error(
            "The length of the currents vector does not match the number of "
            "coils") {}
};

class InvalidInput : public std::runtime_error {
 public:
  explicit InvalidInput(const std::string& what) : std::runtime_error(what) {}
};

class VFieldNearestPositionNotFound : public std::runtime_error {
 public:
#ifndef SWIG
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
#endif
  explicit VFieldNearestPositionNotFound(const PositionVec& p) : std::runtime_error(make_what(p)) {}

 private:
  static const std::string make_what(const PositionVec& p) {
    std::stringstream ss;
    ss << "Could not find nearest position to " << p.transpose() << " in vector field";
    return ss.str();
  }
};

/**
 * @brief This exception is raised when a tensorflow model generates an output
 * with unexpected dimensions
 */
class InvalidOutputTensorDimensions : public std::runtime_error {
 public:
  InvalidOutputTensorDimensions(const std::vector<int>& expected_dims,
                                const std::vector<int>& actual_dims)
      : std::runtime_error(composeMessage(expected_dims, actual_dims)) {}

 private:
  static std::string composeMessage(const std::vector<int>& expected_dims,
                                    const std::vector<int>& actual_dims) {
    std::stringstream ss;
    ss << "Invalid output tensor dimensions." << std::endl;
    ss << "Expected dims: ";
    for (int d : expected_dims) {
      ss << d << ", ";
    }
    ss << std::endl;
    ss << "Actual dims: ";
    for (int d : actual_dims) {
      ss << d << ", ";
    }
    ss << std::endl;
    return ss.str();
  }
};

/**
 * @brief This exception is raised when trying to invert a saturation where
 * it is near or above the asymptotic max
 */
class OverSaturationException : public std::runtime_error {
 public:
  OverSaturationException()
      : std::runtime_error("SaturationFunction above the maximum and cannot be inverted") {}
};

/**
 * @brief This exception is raised when trying to use the cached version of a computation function
 * but the model was not cached first
 */
class NotCachedException : public std::runtime_error {
 public:
  NotCachedException() : std::runtime_error("Cached function was called but data was not cached") {}
};

class FailedComputeCurrentsException : public std::runtime_error {
 public:
  explicit FailedComputeCurrentsException(const std::string& why)
      : std::runtime_error(composeMessage(why)) {}

 private:
  static std::string composeMessage(const std::string& why) {
    std::stringstream ss;
    ss << "Failed computing currents." << std::endl;
    if (!why.empty()) {
      ss << "Failed because: " << why;
    }
    return ss.str();
  }
};
}  // namespace mag_manip
