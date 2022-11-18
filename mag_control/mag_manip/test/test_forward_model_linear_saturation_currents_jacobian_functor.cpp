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

#include <gtest/gtest.h>
#include <ros/package.h>
#include <tsc_utils/eigen_matrix_compare.h>
#include "mag_manip/forward_model_linear_saturation_currents_jacobian_functor.h"
#include "mag_manip/forward_model_mpem.h"

using namespace mag_manip;
using namespace std;

TEST(compute, compate_with_dyn_diff) {
  ForwardModelLinearSaturation::Ptr p_fmls(new ForwardModelLinearSaturation);
  string cal_file = ros::package::getPath("mag_manip") + "/models/cmag_fmls_v1/params.yaml";
  ForwardModelLinear::Ptr p_fml = make_shared<ForwardModelMPEM>();
  p_fml->setCalibrationFile(ros::package::getPath("mag_manip") +
                            "/models/cmag_fmls_v1/CardioMag_CalibrationCube_03-04-19.yaml");
  p_fmls->setLinearModel(p_fml);
  p_fmls->setSaturationFunctionsFile(ros::package::getPath("mag_manip") +
                                     "/models/cmag_fmls_v1/cmag_sat_erf_v1.yaml");

  const int Ne = p_fml->getNumCoils();

  auto p_jac = make_shared<ForwardModelLinearSaturationCurrentsJacobianFunctor>();
  p_jac->setForwardModelLinearSaturation(p_fmls);

  const auto position = PositionVec::Zero();
  const CurrentsVec currents = CurrentsVec::Ones(Ne) * 8;

  const auto jac = (*p_jac)(position, currents);

  const FieldGradient5Vec field_grad =
      p_fmls->computeFieldGradient5FromCurrents(position, currents);

  const double eps = 1e-6;

  const Eigen::MatrixXd del = eps * Eigen::MatrixXd::Identity(Ne, Ne);

  CurrentsJacobian jac_fd(8, Ne);

  for (int k = 0; k < Ne; k++) {
    CurrentsVec currents_ = currents + del.col(k);
    const FieldGradient5Vec field_grad_ =
        p_fmls->computeFieldGradient5FromCurrents(position, currents_);
    jac_fd.col(k) = (field_grad_ - field_grad) / eps;
  }

  EXPECT_TRUE(EIGEN_MATRIX_NEAR_ABS(jac, jac_fd, 1e-4));
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
