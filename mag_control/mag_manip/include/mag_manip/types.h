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

namespace mag_manip {
typedef Eigen::Vector3d FieldVec;
typedef Eigen::Matrix<double, 3, Eigen::Dynamic> FieldVecs;
typedef Eigen::Vector3d DipoleVec;
typedef Eigen::Matrix<double, 3, Eigen::Dynamic> DipoleVecs;
typedef Eigen::Matrix3d GradientMat;
typedef Eigen::Matrix<double, 5, 1> Gradient5Vec;
typedef Eigen::Matrix<double, 5, Eigen::Dynamic> Gradient5Vecs;
typedef Eigen::Vector3d Gradient3Vec;
typedef Eigen::Matrix<double, 3, Eigen::Dynamic> Gradient3Vecs;
typedef Eigen::Matrix<double, 8, 1> FieldGradient5Vec;
typedef Eigen::Matrix<double, 8, Eigen::Dynamic> FieldGradient5Vecs;
typedef Eigen::Matrix<double, 6, 1> FieldGradient3Vec;
typedef Eigen::Vector3d PositionVec;
typedef Eigen::Matrix<double, 3, Eigen::Dynamic> PositionVecs;
typedef Eigen::VectorXd CurrentsVec;
typedef Eigen::MatrixXd CurrentsVecs;
typedef Eigen::MatrixXd CurrentsJacobian;
typedef Eigen::MatrixXd ActuationMat;
typedef Eigen::Matrix<double, 3, 5> Grad35Mat;
typedef Eigen::Vector3d TorqueVec;
typedef Eigen::Vector3d ForceVec;
typedef Eigen::MatrixXd DataMat;
typedef Eigen::VectorXd ResistancesVec;
}  // namespace mag_manip
