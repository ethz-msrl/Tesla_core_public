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

#ifndef ELECTROMAGNET_CALIBRATION_H
#define ELECTROMAGNET_CALIBRATION_H

/**
 ElectromagnetCalibration is a calss that returns the mangetic field and gradient (as packed into a
 vector)
    based on a spherical harmonic calibration. The constructor takes a YAML formatted calibration
 file. An example file is provided below.

    Functions:
    fieldAtPoint( current, position ) :  Returns the field at the specified point given the currents
 packed into a vector

    gradientAtPoint( current position) : Returns the 3x3 gradient matrix given the currents packed
 into a vetctor

    fieldAndGradientAtPoint(current, position) : Returns a 8x1 vector of field stacked on a 5
 element vector packing
                of the unique elements in the gradient matrix ([dBx/dx, dBx/dy, dBx/dz, dBy/dy,
 dBy/dz]).

    fieldCurrentJacobian( position ) : Returns a 3xN matrix of the local current to field
 relationship at a point

    gradientCurrentJacobian( position) : Returns a 5xN matrix of the local current to gradient
 relationship at a point

    fieldAndGradientCurrentJacobian( position ) : Returns an 8xN matrix of the local current to
 field stacked ontop of gradient

    loadCalibration( fileName ) : loads a  new calibration file in YAML format.  Returns true if
 sucessful.

    getNumberOfCoils() : Returns the number of coils (currents) for the system

    getName() : Returns the Name of the system from the calibration file

    pointInWorkspace( position ) : Returns true if the point is in the calibrated workspace


    Libraries:
    This class requires the Eigen library version 3 or higher and the libyaml-cpp verson 0.5 (add
 LIBS += -lyaml-cpp to project file or equivalent)

    Files:
        This class requires the following files to compile
            electromagnet_calibration.h
            electromagnet_calibration.cpp
            EigenToYAML.h

    Example calibration file layout:
        System_Name: 'Test System'
        Workspace_Dimensions:
        - [-0.025, 0.025]
        - [-0.025, 0.025]
        - [-0.025, 0.025]
        Coil_List: [Coil_1, Coil_2, Offset]
        Coil_1:
          Source_List: [Src_1, Src_2]
          Src_1:
            A_Coeff: []
            B_Coeff:
            - [1]
            - [-0.5]
            Source_Position:
            - [0.22416258792214033]
            - [-0.07556653205541619]
            - [-0.22109796762044504]
            Source_Direction:
            - [-0.7025088251776764]
            - [0.14854822954743013]
            - [0.6959991249740192]
          Src_2:
            A_Coeff: []
            B_Coeff:
            - [0.25]
            - [-0.25]
            Source_Position:
            - [0.3168094119654824]
            - [0.21831564272785464]
            - [-0.0776279052373666]
            Source_Direction:
            - [-0.96373336339233]
            - [-0.25663768967502687]
            - [0.07317870247555225]
        Coil_2:
          Source_List: [Src_1]
          Src_1:
            A_Coeff: []
            B_Coeff:
            - [1]
            Source_Position:
            - [0.15865180505201498]
            - [-0.022607524623308707]
            - [-0.34998355269929454]
            Source_Direction:
            - [-0.6483538763510639]
            - [0.07075237800794039]
            - [0.7580444261800644]
        Offset:
          Source_List: [Src_1]
          Src_1:
            A_Coeff: []
            B_Coeff:
            - [.01]
            Source_Position:
            - [0.15865180505201498]
            - [-0.022607524623308707]
            - [-0.34998355269929454]
            Source_Direction:
            - [-0.6483538763510639]
            - [0.07075237800794039]
            - [0.7580444261800644]

 **/

#include <assert.h>
#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>
#include <exception>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "mpem/EigenToYAML.h"
#include "mpem/scalorPotential.h"

typedef Eigen::Matrix<double, 8, 1> Vector8d;
typedef Eigen::Matrix<double, 5, 1> Vector5d;

struct MagneticCalibrationException : public std::exception {
  const char* what() const throw() { return "Magnetic calibration exception"; }
};

/**
 * @brief The MagneticMeasurementData struct provides the format calibration data must be supplied
 * for the calibration
 */
struct MagneticMeasurement {
  Eigen::Vector3d Field;                /**< The measured Field in Tesla **/
  Eigen::Vector3d Position;             /**< The position of the measurement in meters **/
  Eigen::VectorXd AppliedCurrentVector; /**< The applied current vector in Amps **/
  MagneticMeasurement();

  ///
  /// \brief calibrationDataPoint
  /// \param Field the field value in Tesla
  /// \param Pos the position of the measurement in meters
  /// \param CurrentVec the applied current vector in Amps
  ///
  MagneticMeasurement(const Eigen::Vector3d& Field, const Eigen::Vector3d& Pos,
                      const Eigen::VectorXd& CurrentVec);

  // Doesn't work with SWIG
  // EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct MagneticState {
  Eigen::Vector3d Field;
  Eigen::Matrix3d Gradient;
  Eigen::Matrix<double, 5, 3> GradientPositionJacobian;

  Eigen::MatrixXd FieldGradientActuationMatrix;

  MagneticState() {
    Field.setZero(3);
    Gradient.setZero(3, 3);
    GradientPositionJacobian.setZero(5, 3);
    FieldGradientActuationMatrix.setZero(0, 0);
  }

  // doesn't work with SWIG
  // EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct MagneticWorkSpace {
  double xMin;
  double xMax;
  double yMin;
  double yMax;
  double zMin;
  double zMax;
  MagneticWorkSpace();
  explicit MagneticWorkSpace(double size);
  MagneticWorkSpace(double xMin, double xMax, double yMin, double yMax, double zMin, double zMax);
};

/// @brief a class describing the calibration of an arbitrary magnetic system
///
/// This class assumes the system can be described by spherical harmonic scalor
/// potentials at multiple locations with weightings proportional to the currents
/// appied.  See. "PUBLICATION HERE"
class ElectromagnetCalibration {
 public:
  /**
   * @brief the constructor requires a yaml encoded calibration file
   *
   * @param calibrationFileName is the file location for the calibration
   */
  explicit ElectromagnetCalibration(std::string calibrationFileName);

  /**
   * @brief the constructor from a coil list.
   *
   * @param systemName
   * @param workSpace_
   * @param coilList coilList The list of coils and their respective sources.
   * @param dc_field_offset dc_field_offset The dc offset field, if any.
   */
  ElectromagnetCalibration(std::string systemName, const MagneticWorkSpace& workSpace_,
                           const std::vector<ScalorPotential>& coilList,
                           const ScalorPotential& dc_field_offset = ScalorPotential());

  /* The following functions return the field and/or gradient given a current vector and a position.
     If no position is provided, it is assumed to be at the workspace origin. For the combined
     vector,
     the gradient matrix has been repacked into a 5 element vetor form (because it is symetric and
     has zero trace).
     The order of the gradient terms is: [dBx/dx, dBx/dy, dBx/dz, dBy/dy, dBy/dz].
  */

  /**
   * @brief returns the field at a point
   * This function does not check to see if the point is actually in the calibrated workspace.
   *
   * @param currentVector is an orderd list of currents in each coil
   * @param position is the position in the workspace the field is desired
   */
  Eigen::Vector3d fieldAtPoint(const Eigen::VectorXd& currentVector,
                               const Eigen::Vector3d& position = Eigen::Vector3d::Zero()) const;

  /**
   * @brief returns the 3x3 symetric gradient matrix at a desired location
   *
   * This function does not check to see if the point is actually in the calibrated workspace.
   *
   * @param currentVector is an orderd list of currents in each coil
   * @param position is the position in the workspace the field is desired
   */
  Eigen::Matrix3d gradientAtPoint(const Eigen::VectorXd& currentVector,
                                  const Eigen::Vector3d& position = Eigen::Vector3d::Zero()) const;

  /**
   * @brief returns the 8x1 stacked field over gradient vector
   *
   * The gradient matrix has been repacked, since it is symetric an has zero trace, into a five
   * element vector.
   *   The element order is: [dBx/dx, dBx/dy, dBx/dz, dBy/dy, dBy/dz]^T
   * This function does not check to see if the point is actually in the calibrated workspace.
   *
   * @param currentVector is an orderd list of currents in each coil
   * @param position is the position in the workspace the field is desired
   */
  Vector8d fieldAndGradientAtPoint(const Eigen::VectorXd& currentVector,
                                   const Eigen::Vector3d& position = Eigen::Vector3d::Zero()) const;

  /**
   * @brief returns the 8x1 stacked field over gradient vector due to the zero-current field offset
   *
   * The gradient matrix has been repacked, since it is symetric an has zero trace, into a five
   * element vector.
   *   The element order is: [dBx/dx, dBx/dy, dBx/dz, dBy/dy, dBy/dz]^T
   * This function does not check to see if the point is actually in the calibrated workspace.
   *
   * @param position is the position in the workspace the field is desired
   */
  Vector8d offsetFieldAndGradientAtPoint(
      const Eigen::Vector3d& position = Eigen::Vector3d::Zero()) const;

  /* The following functions return the Current Jacobian of the field and/or gradient.  This matrix
   *  can be inverted to determine the
   *    currents necessary to achieve a desired field and gradient.  The gradient portion of the
   *    matrix has been vectorized into 5 elements.
   *    They are [dBx/dx, dBx/dy, dBx/dz, dBy/dy, dBy/dz].
   */

  /**
   * @brief returns the 3xN matrix mapping field at a point to the current in each of the N sources
   *
   * This function does not check to see if the point is actually in the calibrated workspace.
   *
   * @param position is the position in the workspace the field is desired
   */
  Eigen::MatrixXd fieldCurrentJacobian(
      const Eigen::Vector3d& position = Eigen::Vector3d::Zero()) const;

  /**
   * @brief  returns the 5xN matrix mapping the field radient at a point to the current in each of
   * the N sources
   *
   * The gradient matrix has been repacked, since it is symetric an has zero trace, into a five
   * element vector.
   *   The element order is: [dBx/dx, dBx/dy, dBx/dz, dBy/dy, dBy/dz]^T
   * This function does not check to see if the point is actually in the calibrated workspace.
   *
   * @param position is the position in the workspace the field is desired
   */
  Eigen::MatrixXd gradientCurrentJacobian(
      const Eigen::Vector3d& position = Eigen::Vector3d::Zero()) const;

  /**
   * @brief returns the 8xN matrix mapping of the stacked field over field gradient at a point to
   * the current in each of the N sources
   *
   * The gradient matrix has been repacked, since it is symetric an has zero trace, into a five
   * element vector.
   *   The element order is: [dBx/dx, dBx/dy, dBx/dz, dBy/dy, dBy/dz]^T
   * This function does not check to see if the point is actually in the calibrated workspace.
   *
   * @param position is the position in the workspace the field is desired
   */
  Eigen::MatrixXd fieldAndGradientCurrentJacobian(
      const Eigen::Vector3d& position = Eigen::Vector3d::Zero()) const;

  /**
   * @brief returns the 5x3 field gradient jacobian at a point given the currents in each of the N
   * sources
   *
   * This function returns the 5x3 Jacobian describing how gradient changes with position.  The
   * gradient change is a 5x3 packing of a 3x3x3 tensor.
   *  The first column is how the gradient vector packing [dBx/dx, dBx/dy, dBx/dz, dBy/dy, dBy/dz]
   *  changes with x, the second is how it changes with y, and third is
   *  how it changes with z.
   *
   * The 3x3x3 tensor has been repacked, since the 3x3 field gradient is symetric an has zero
   * trace, into a 5x3 element vector.
   *   The element order is:
   *    [dBx/dxdx, dBx/dxdy, dBx/dxdz
   *     dBx/dydx, dBx/dydy, dBx/dydz
   *     dBx/dzdx, dBx/dzdy, dBx/dzdz
   *     dBy/dydx, dBy/dydy, dBy/dydz
   *     dBy/dzdx, dBy/dzdy, dBy/dzdz]
   * This function does not check to see if the point is actually in the calibrated workspace.
   *
   * @param currentVector is an orderd list of currents in each coil
   * @param position is the position in the workspace the field is desired
   */
  Eigen::Matrix<double, 5, 3> gradientPositionJacobian(
      const Eigen::VectorXd& currentVector,
      const Eigen::Vector3d& position = Eigen::Vector3d::Zero()) const;

  /**
   * @brief calculates the field, field and field gradient jacobians, and the current jacobians at a
   *
   * The 3x3 gradient and 3x3x3 field gradient jacobian tensor has been repacked, since the 3x3
   * field gradient is symetric an has zero trace, into a 8x3 element vector.
   *   The element order is:
   *    [dBx/dx,   dBx/dy,   dBx/dz
   *     dBx/dy,   dBy/dy,   dBy/dz
   *     dBx/dz,   dBy/dz, -(dBx/dx+dBy/dy)
   *     dBx/dxdx, dBx/dxdy, dBx/dxdz
   *     dBx/dydx, dBx/dydy, dBx/dydz
   *     dBx/dzdx, dBx/dzdy, dBx/dzdz
   *     dBy/dydx, dBy/dydy, dBy/dydz
   *     dBy/dzdx, dBy/dzdy, dBy/dzdz]
   * This function does not check to see if the point is actually in the calibrated workspace.
   *
   * @param fieldAtPoint a pass by reference Vector to return the calculated field
   * @param fieldGradientPositionJacobian a pass by reference matrix to return the field jacobian
   * stacked over the field gradient jacobian
   * @param fieldGradientCurrentJacobian a pass by reference matrix to return the field and
   * gradient current jacobian
   * @param currentVector is an orderd list of currents in each coil
   * @param position is the position in the workspace the field is desired
   */
  void fullMagneticState(Eigen::Vector3d& fieldAtPoint,
                         Eigen::Matrix<double, 8, 3>& fieldGradientPositionJacobian,
                         Eigen::MatrixXd& fieldGradientCurrentJacobian,
                         const Eigen::VectorXd& current,
                         const Eigen::Vector3d& position = Eigen::Vector3d::Zero()) const;

  /**
   * @brief calculates the field, field and field gradient jacobians, and the current jacobians at a
   * point given the currents in each of the N sources
   * this faunction is more efficient than calling each of the applicable functions seporately.
   *
   * @param currentVector is an orderd list of currents in each coil
   * @param position is the position in the workspace the field is desired
   *
   * @return \see MagneticState
   */
  MagneticState fullMagneticState(const Eigen::VectorXd& currentVector,
                                  const Eigen::Vector3d& position = Eigen::Vector3d::Zero()) const;

  /**
   * @brief  converts a 5x1 gradient vector into a symetric zero trace 3x3 matrix
   *
   * @param gradVector is the desired vector to be remaped
   */
  static Eigen::Matrix3d remapGradientVector(const Vector5d& gradVector);

  /**
   * @brief converts a 3x3 gradient marix into the 5x1 gradient vector
   *
   * @param gradMatrix is the desired matrix to be remaped
   *
   * This function does not check to verify the matrix is indeed symetric and zero trace
   */
  static Vector5d remapGradientMatrix(const Eigen::Matrix3d& gradMatrix);

  /**
   * @brief converts a 3x1 moment vector into a 3x5 force matrix
   *
   * converts a 3x1 moment vector into a 3x5 force matrix to allow easy calculation of the magnetic
   * force by multiplication with the gradient vector
   *
   * @param moment is the magnetic moment that is packed into the force matrix
   */
  static Eigen::MatrixXd packForceMatrix(const Eigen::Vector3d& moment);

  /**
   * @brief loads a new calibration file
   *
   *  @param a string pointing to the location of the yaml formated calbiration file
   * @raises std::runtime_error if there is a problem loading fileName
   * @returns true if loading succeeded. WARNING: never returns false since an exception is thrown
   * if the loading failed
   *
   * @return
   */
  bool loadCalibration(std::string fileName);

  /**
   * @brief writes a new calibration file
   *
   * @param a string pointing to the location for the yaml formated calbiration file
   *
   * @return
   */
  bool writeCalibration(std::string fileName) const;

  /**
   * @brief returns the number of coils in the calibration
   *
   * @return
   */
  int getNumberOfCoils() const;

  /**
   * @brief returns the number of sources for the given coil
   */
  int getNumberOfSources(unsigned int coilNum) const;

  /**
   * @brief returns the number of coefficients for the given source
   */
  int getNumberOfCoeffients(unsigned int coilNum, unsigned int srcNum) const;

  /**
   * @brief returns if the calibration has a DC offset
   */
  bool hasOffset() const;

  /**
   * @brief returns the calibration name
   */
  std::string getName() const;

  /**
   * @brief checks to see if a point lies within the calibrated workspace
   *
   * @param position position the desired position to check
   */
  bool pointInWorkspace(const Eigen::Vector3d& position) const;

  /**
   * @brief returns the rectanglar extent of the calibrated workspace
   */
  MagneticWorkSpace getWorkSpace() const;

  /**
   * @brief  sets the magnetic workspace to the desired specification
   *
   * @param ws
   */
  void setWorkSpace(const MagneticWorkSpace& ws);

  void useOffset(bool offsetOn = true);
  bool queryUseOffset() const;

  /**
   * @brief The calibration_constraints enum
   * UNIT_HEADING_ONLY constrains the azimuth of the potentials to be unit length
   * HEADING_AND_POSITION constrains the azimuth of potentials to be unit length and enforces that
   * the positions lie in a spherical anulous outside of the measured data and prevents them from
   * going to infinity
   * HEADING_THEN_POSITION first solves with the heading constraints, then it resolves pushing any
   * sources that lie inside the measurement data region out of a bounding circle of the data.
   */
  enum calibration_constraints { UNIT_HEADING_ONLY, HEADING_AND_POSITION, HEADING_THEN_POSITION };
  void calibrate(std::string calibrationName, const std::vector<MagneticMeasurement>& dataList,
                 bool printProgress = true, bool printStats = true,
                 calibration_constraints constraint = HEADING_THEN_POSITION,
                 double minimumSourceToCenterDistance = -1,
                 double maximumSourceToCenterDistance = -1, double converganceTolerance = 1e-12,
                 int maxIterations = 10000);

  void printStats(const std::vector<MagneticMeasurement>& dataList) const;

 protected:
  ElectromagnetCalibration(); /**< Default Constructor only available to inheriting classes **/

  MagneticWorkSpace workSpace;

  std::vector<ScalorPotential> coilList;
  ScalorPotential offset;

  bool use_offset;

  std::string name;

  bool checkSourcePositions(bool printWarning = false) const;

 private:
  Eigen::VectorXd addOffset(const Eigen::VectorXd& current);

  void applyPHI(const Eigen::VectorXd& PHI);
  void obtainPHI(Eigen::VectorXd& PHI);

  void packError(Eigen::VectorXd& error, const std::vector<MagneticMeasurement>& dataList);
  void packErrorJacobian(Eigen::MatrixXd& jacobian,
                         const std::vector<MagneticMeasurement>& dataList);

  void linearLeastSquareCoeffFit(const std::vector<MagneticMeasurement>& dataList);

  int numberOfParameters;
  int numberOfConstraints;
  int numberOfMeasurements;
  int numberOfSources;

  int nConst;

  bool minRadIsActive;
  double rMinSq, rMaxSq, posWeight;
  Eigen::Vector3d pCenter;
};

namespace mpem {
// This function is required because versions of yaml-cpp below 0.6 don't
// parse ~ as a null value
bool is_null(const YAML::Node& node);
}  // namespace mpem

#endif  // ElectromagnetCalibration_H
