//
// Tesla - A ROS-based framework for performing magnetic manipulation
//
// Software License Agreement (BSD License)
//
// ©2022 ETH Zurich, Andrew Petruska, Janis Edelmann, D-​MAVT; Multi-Scale Robotics Lab (MSRL) ;
// Prof Bradley J. Nelson All rights reserved.
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

#include "mpem/PseudoInverse.h"
#include <cmath>
#include <iostream>

Eigen::VectorXd Math::pseudoInverse(const Eigen::MatrixXd& A, const Eigen::VectorXd& b,
                                    double singularMinToMaxRatio, double dampingFactor) {
  // uses a singular value decomposisition (SVD) to solve for the least squares solution

  Eigen::JacobiSVD<Eigen::MatrixXd> svd(
      A, Eigen::ComputeThinU | Eigen::ComputeThinV);  // computes the SVD

  //    cout << "SVD COMPUTED" << endl;
  Eigen::MatrixXd S_inv = Eigen::MatrixXd::Zero(svd.matrixV().cols(), svd.matrixU().cols());

  // std::cout << svd.singularValues() << std::endl;
  for (int i = 0; i < std::min(A.rows(), A.cols()); i++) {
    double val = 0;
    if (svd.singularValues()[i] >
        svd.singularValues()[0] * singularMinToMaxRatio)  // threashold singular values anything
                                                          // less than 1/1000 of the max is set to 0
      // val = 1.0 / svd.singularValues()[i];
      val = svd.singularValues()[i] /
            (svd.singularValues()[i] * svd.singularValues()[i] + dampingFactor);

    S_inv(i, i) = val;
  }
  //    cout << "Singular Values Flipped" << endl;

  Eigen::VectorXd answer;

  //        cout << "Size of U:  " << svd.matrixU().rows() << " x " << svd.matrixU().cols() << endl;
  //        cout << "Size of V:  " << svd.matrixV().rows() << " x " << svd.matrixV().cols() << endl;
  //        cout << "Size of S:  " << S.rows() << " x " << S.cols() << endl;
  //        cout << "Size of b:  " << b.rows() << " x " << b.cols() << endl;

  answer = svd.matrixV() * (S_inv * (svd.matrixU().transpose() * b));

  //   cout << "Answer Computed " << endl;
  return answer;
}

Eigen::MatrixXd Math::pseudoInverseMatrix(const Eigen::MatrixXd& A, double singularMinToMaxRatio,
                                          double dampingFactor) {
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(
      A, Eigen::ComputeThinU | Eigen::ComputeThinV);  // computes the SVD

  //    cout << "SVD COMPUTED" << endl;
  Eigen::MatrixXd S_inv = Eigen::MatrixXd::Zero(svd.matrixV().cols(), svd.matrixU().cols());
  /*
  std:: cout << std::endl;
  std:: cout << A << std::endl << std::endl;
  std::cout << svd.matrixV() << std::endl << std::endl;
  std::cout << svd.matrixU() << std::endl << std::endl;
  std::cout << svd.singularValues() << std::endl;
  */
  for (int i = 0; i < std::min(A.rows(), A.cols()); i++) {
    double val = 0;
    if (svd.singularValues()[i] >
        svd.singularValues()[0] * singularMinToMaxRatio)  // threashold singular values anything
                                                          // less than 1/1000 of the max is set to 0
      // val = 1.0 / svd.singularValues()[i];
      val = svd.singularValues()[i] /
            (svd.singularValues()[i] * svd.singularValues()[i] + dampingFactor);

    S_inv(i, i) = val;
  }
  //    cout << "Singular Values Flipped" << endl;

  Eigen::MatrixXd answer;

  //    cout << "Size of A:  " << A.rows() << " x " << A.cols() << endl;
  //    cout << "Size of U:  " << svd.matrixU().rows() << " x " << svd.matrixU().cols() << endl;
  //    cout << "Size of V:  " << svd.matrixV().rows() << " x " << svd.matrixV().cols() << endl;
  //    cout << "Size of Sinv:  " << S_inv.rows() << " x " << S_inv.cols() << endl;
  //    cout << endl << S_inv << endl << endl;

  answer = svd.matrixV() * (S_inv * (svd.matrixU().transpose()));

  // std::cout << "Answer Computed " << std::endl << answer << "\n__________________\n"<<  A*answer
  // << "\n__________________\n" << answer*A << std::endl<<std::endl;
  return answer;
}
