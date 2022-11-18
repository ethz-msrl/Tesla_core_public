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
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>

#define EIGEN_MATRIX_EQUAL(MatrixA, MatrixB) tesla::assertMatricesNear(MatrixA, MatrixB, 0.0)

#define EIGEN_MATRIX_NEAR_ABS(MatrixA, MatrixB, Threshold) \
  tesla::assertMatricesNear(MatrixA, MatrixB, Threshold, tesla::ABSOLUTE)

#define EIGEN_MATRIX_NEAR_REL(MatrixA, MatrixB, Threshold) \
  tesla::assertMatricesNear(MatrixA, MatrixB, Threshold, tesla::RELATIVE)

namespace tesla {

enum MatrixAssertionType { ABSOLUTE, RELATIVE };

template <typename TypeA, typename TypeB>
::testing::AssertionResult assertMatricesNear(const TypeA& m_A, const TypeB& m_B,
                                              double threshold = 1e-12,
                                              MatrixAssertionType type = ABSOLUTE) {
  if (m_A.rows() != m_B.rows() || m_A.cols() != m_B.cols()) {
    std::cout << "Matrices do not match" << std::endl;
    return ::testing::AssertionFailure() << "Matrices do not have the same size. "
                                         << "lhs: (" << m_A.rows() << ", " << m_A.cols()
                                         << "), rhs: (" << m_B.rows() << ", " << m_B.cols() << ")";
  }

  for (int i = 0; i < m_A.rows(); i++) {
    for (int j = 0; j < m_A.cols(); j++) {
      double diff = std::abs(m_A(i, j) - m_B(i, j));

      if (std::isnan(diff)) {
        return ::testing::AssertionFailure() << "difference is nan: at (" << i << ", " << j << ")";
      }

      if (type == ABSOLUTE) {
        if (diff > threshold) {
          return ::testing::AssertionFailure()
                 << "Matrices do not absolutely match at (" << i << ", " << j
                 << "), lhs: " << m_A(i, j) << " rhs: " << m_B(i, j);
        }
      } else if (type == RELATIVE) {
        double ref = std::max(std::abs(m_A(i, j)), std::abs(m_B(i, j)));

        if ((diff / ref) > threshold) {
          return ::testing::AssertionFailure()
                 << "Matrices do not relatively match at (" << i << ", " << j
                 << "), lhs: " << m_A(i, j) << " rhs: " << m_B(i, j);
        }
      }
    }
  }

  return ::testing::AssertionSuccess() << "Matrices match";
}
}  // namespace tesla
