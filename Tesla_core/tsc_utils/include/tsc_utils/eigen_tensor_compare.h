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

#include <gtest/gtest.h>
#include <algorithm>
#include <cmath>
#include <unsupported/Eigen/CXX11/Tensor>
#include "tsc_utils/eigen_matrix_compare.h"

#define EIGEN_TENSOR2_EQUAL(TensorA, TensorB) tesla::assertTensors2Near(TensorA, TensorB, 0.0)

#define EIGEN_TENSOR3_EQUAL(TensorA, TensorB) tesla::assertTensors3Near(TensorA, TensorB, 0.0)

#define EIGEN_TENSOR2_NEAR_ABS(TensorA, TensorB, Threshold) \
  tesla::assertTensors2Near(TensorA, TensorB, Threshold, tesla::ABSOLUTE)

#define EIGEN_TENSOR3_NEAR_ABS(TensorA, TensorB, Threshold) \
  tesla::assertTensors3Near(TensorA, TensorB, Threshold, tesla::ABSOLUTE)

#define EIGEN_TENSOR2_NEAR_REL(TensorA, TensorB, Threshold) \
  tesla::assertTensors2Near(TensorA, TensorB, Threshold, tesla::RELATIVE)

#define EIGEN_TENSOR3_NEAR_REL(TensorA, TensorB, Threshold) \
  tesla::assertTensors3Near(TensorA, TensorB, Threshold, tesla::RELATIVE)

namespace tesla {

template <typename ScalarA, typename ScalarB>
::testing::AssertionResult assertTensors3Near(const Eigen::Tensor<ScalarA, 3>& t_A,
                                              const Eigen::Tensor<ScalarB, 3>& t_B,
                                              double threshold = 1e-12,
                                              MatrixAssertionType type = ABSOLUTE) {
  if (t_A.dimension(0) != t_B.dimension(0) || t_A.dimension(1) != t_B.dimension(1) ||
      t_A.dimension(2) != t_B.dimension(2)) {
    std::cout << "Tensors do not match" << std::endl;
    return ::testing::AssertionFailure()
           << "Tensors do not have the same size. "
           << "lhs: (" << t_A.dimension(0) << ", " << t_A.dimension(1) << ", " << t_A.dimension(2)
           << "), rhs(" << t_B.dimension(0) << ", " << t_B.dimension(1) << ", " << t_B.dimension(2)
           << ")";
  }

  for (int i = 0; i < t_A.dimension(0); i++) {
    for (int j = 0; j < t_A.dimension(1); j++) {
      for (int k = 0; k < t_A.dimension(2); k++) {
        double diff = std::abs(t_A(i, j, k) - t_B(i, j, k));
        if (std::isnan(diff)) {
          return ::testing::AssertionFailure()
                 << "difference is nan: at (" << i << ", " << j << ", " << k << ")";
        }
        if (type == ABSOLUTE) {
          if (diff > threshold) {
            return ::testing::AssertionFailure()
                   << "Tensors do not absolutely match at (" << i << ", " << j << ", " << k
                   << "), lhs: " << t_A(i, j, k) << " rhs: " << t_B(i, j, k);
          } else if (type == RELATIVE) {
            double ref = std::max(std::abs(t_A(i, j, k)), std::abs(t_B(i, j, k)));

            if ((diff / ref) > threshold) {
              return ::testing::AssertionFailure()
                     << "Tensors do not relatively match at (" << i << ", " << j << ", " << k
                     << "), lhs: " << t_A(i, j, k) << " rhs: " << t_B(i, j, k);
            }
          }
        }
      }
    }
  }
  return ::testing::AssertionSuccess() << "Tensors match";
}

template <typename ScalarA, typename ScalarB>
::testing::AssertionResult assertTensors2Near(const Eigen::Tensor<ScalarA, 2>& t_A,
                                              const Eigen::Tensor<ScalarB, 2>& t_B,
                                              double threshold = 1e-12,
                                              MatrixAssertionType type = ABSOLUTE) {
  if (t_A.dimension(0) != t_B.dimension(0) || t_A.dimension(1) != t_B.dimension(1)) {
    std::cout << "Tensors do not match" << std::endl;
    return ::testing::AssertionFailure()
           << "Tensors do not have the same size. "
           << "lhs: (" << t_A.dimension(0) << ", " << t_A.dimension(1) << "), rhs("
           << t_B.dimension(0) << ", " << t_B.dimension(1) << ")";
  }

  for (int i = 0; i < t_A.dimension(0); i++) {
    for (int j = 0; j < t_A.dimension(1); j++) {
      double diff = std::abs(t_A(i, j) - t_B(i, j));
      if (std::isnan(diff)) {
        return ::testing::AssertionFailure() << "difference is nan: at (" << i << ", " << j << ")";
      }
      if (type == ABSOLUTE) {
        if (diff > threshold) {
          return ::testing::AssertionFailure()
                 << "Tensors do not absolutely match at (" << i << ", " << j
                 << "), lhs: " << t_A(i, j) << " rhs: " << t_B(i, j);
        } else if (type == RELATIVE) {
          double ref = std::max(std::abs(t_A(i, j)), std::abs(t_B(i, j)));

          if ((diff / ref) > threshold) {
            return ::testing::AssertionFailure()
                   << "Tensors do not relatively match at (" << i << ", " << j
                   << "), lhs: " << t_A(i, j) << " rhs: " << t_B(i, j);
          }
        }
      }
    }
  }
  return ::testing::AssertionSuccess() << "Tensors match";
}
}  // namespace tesla
