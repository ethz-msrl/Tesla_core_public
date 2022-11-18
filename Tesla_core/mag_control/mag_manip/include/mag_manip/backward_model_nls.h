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

#include <ceres/solver.h>

#include <memory>
#include <string>
#include <utility>

#include "mag_manip/backward_model.h"
#include "mag_manip/currents_jacobian_functor.h"
#include "mag_manip/forward_model.h"
#include "mag_manip/types.h"

namespace mag_manip {

/**
 * @brief backward model that uses nonlinear least squares for inversion
 *
 * this method should be able to invert any type of nonlinear or linear forward model.
 * it works by solving the following nonlinear least squares problem where \f$\mathbf{i}_c\f$ is the
 * computed current
 * that produces \f$\mathbf{b}_d\f$.
 *
 * \f[
 * \mathbf{i}_c = \min_{\mathbf{i}} || \mathbf{b}(\mathbf{p}, \mathbf{i}) - \mathbf{b}_d||^2
 * \f]
 *
 * for underconstrained problems such as computing a 3d field from n > 3 electromagnet currents, the
 * solution returned
 * is not necessarily the solution that minimizes the power in the system ie the l2 norm of the
 * currents vector. if the
 * system is linear, BackwardModelLinearL2 does this naturally by solving the least square solution.
 * You can try to
 * minimize the currents by adding L2 regularization to the NLS problem.
 *
 * \f[
 * \mathbf{i}_c = \min_{\mathbf{i}} || \mathbf{b}(\mathbf{p}, \mathbf{i}) - \mathbf{b}_d||^2 +
 * \lambda ||i||^2
 * \f]
 *
 * Where \f$\lambda$\f is a scalar term that penalizes solutions with too large currents. It should
 * be a small number,
 * with 1e-8 being the default.
 *
 * You can do this by calling setDoMinimizeCurrents(true). To set lambda, call
 * setWeightCurrents(lambda).
 *
 * The NLS problem is solved using the Ceres solver.
 *
 */
class BackwardModelNLS : public BackwardModel {
 public:
  typedef std::unique_ptr<BackwardModelNLS> UPtr;
  typedef std::shared_ptr<BackwardModelNLS> Ptr;

  BackwardModelNLS();

  /**
   * @brief Load the calibration of the model
   *
   * The calibration should be contained in a folder with a metafile containing the params fo the
   * calibration. The folder should also contain a calibration file for the forward model and for
   * the solver params that are used by the nls model
   *
   * @param filename: path to the metafile of the calibration
   */
  virtual void setCalibrationFile(const std::string& filename);

  virtual bool isValid() const;

  virtual int getNumCoils() const;

  /**
   * @brief Sets the underlying forward model.
   *
   * This should be done before calling any compute functions
   *
   * @param p_forward_model: pointer to the forward model
   */
  virtual void setForwardModel(ForwardModel::Ptr p_forward_model);

  /**
   * @brief Sets the calibration model for the forward model only
   *
   * @param filename: path to the forward model calibration file
   */
  virtual void setForwardModelCalibrationFile(const std::string& filename);

  /**
   * @brief Gets a const reference to the underyling forward model
   *
   * @return reference to the forward model
   */
  const ForwardModel& getForwardModel() const;

  /**
   * @brief Set optional initial currents for the model inversion
   *
   * If not the initial currents default to 0
   *
   * @param currents
   */
  virtual void setInitialCurrents(const CurrentsVec& currents);

  /**
   * @brief To set custom options for the Ceres solver
   *
   * If not set, Ceres defaults are used
   *
   * @param options: the solver options
   */
  virtual void setSolverOptions(const ceres::Solver::Options& options);

  /**
   * @brief If true, will log all iterations to console using
   * ceres::Solver::options::trust_region_minimizer_iterations_to_dump
   *
   * WARNING: This will log all iterations unless you change
   * ceres::Solver::Options::max_num_iterations
   *
   * @param enable: if true, turn on logging
   */
  void setDoLogIterationsToConsole(const bool enable);

  /**
   * @brief returns true if logging is enabled to console
   *
   * @return
   */
  bool getDoLogIterationsToConsole() const;

  /**
   * @brief If true, will print a full summary after a compute
   *
   * @param enable
   */
  void setDoPrintSummary(const bool enable);

  /**
   * @brief Returns true if a summary is printed after a compute
   *
   * @return
   */
  bool getDoPrintSummary() const;

  /**
   * @brief Turns on L2 regularization on currents
   *
   * @param enable: if true, will add regularization
   */
  void setDoMinimizeCurrents(const bool enable);

  /**
   *
   * @return true if current regularization is on
   */
  bool getDoMinimizeCurrents() const;

  /**
   * @brief Sets the weight of the current L2 norm term.
   *
   * This should be something quite small in the 1e-8 range.
   *
   * @param lambda: weight of the L2 norm term
   */
  void setWeightCurrents(const double lambda);

  /**
   * @brief Returns the weight of the currents regularization
   *
   * @return lambda the weight of the currents L2 norm term
   */
  double getWeightCurrents() const;

  /**
   * @brief The functor computes the Jacobian relating changes in the current to changes in the
   * field
   *
   * You need to set this and turn on analytical gradients using setUseAnalyticalGradients. Make
   * sure p_jac is using the same model as this BackwardModelNLS! p_jac must already be fully
   * initialized before passing it here.
   *
   * @param p_jac: pointer to the Jacobian calculator
   */
  void setCurrentsJacobianFunctor(CurrentsJacobianFunctor::Ptr p_jac);

  /**
   * @brief Turn off numerical differentiation and use analytical gradients
   *
   * WARNING: you first need to set the functor which calculates the Jacobian using
   * setCurrentsJacobianFunctor
   *
   * Disable by default
   *
   * @param enable: if true, will use analytical gradients, if false will use numerical gradients.
   */
  void setUseAnalyticalGradients(const bool enable);

  /**
   * @brief Returns true if computing analytical gradients
   *
   * @return true if computing analytical gradients
   */
  bool getUseAnalyticalGradients() const;

  /**
   * @brief Turn on caching of positions to speed up computation
   *
   * @param enable: if true, caches positions
   */
  void setDoCachePosition(const bool enable);

  /**
   * @brief Returns true if position caching is set to turned on
   *
   * By default caching is set to off
   * If your forward model supports position caching,
   * you can turn caching on for considerable speedups
   *
   * @return true if caching positions
   */
  bool getDoCachePosition() const;

  /**
   * @brief Sets lower bounds on the currents
   *
   * @param max_currents: A vector of size Ne setting the lower bounds on currents
   */
  void setMinCurrents(const CurrentsVec& min_currents);

  /**
   * @brief Gets the lower bounds on the currents
   *
   * @return: A vector of size Ne
   */
  CurrentsVec getMinCurrents() const;

  /**
   * @brief Sets upper bounds on the currents
   *
   * @param max_currents: A vector of size Ne setting the upper bounds on currents
   */
  void setMaxCurrents(const CurrentsVec& max_currents);

  /**
   * @brief Gets the upper bounds on the currents
   *
   * @return: A vector of size Ne
   */
  CurrentsVec getMaxCurrents() const;

  /**
   * @brief Sets the threshold on the distance between the computed field
   * and the desired field.
   *
   * If the distance exceeds this threshold, the computeCurrents*Ret functions will
   * return false
   *
   * @param dist
   */
  void setMaximumFieldDistance(const double dist);

  /**
   * @brief Gets the threshold on the distance between the computed field
   * and the desired field
   *
   * See setMaximumFieldDistance
   *
   * @return the distance parameter
   */
  double getMaximumFieldDistance() const;

  /**
   * @brief Sets the threshold on the distance between the computed gradient
   * and the desired field.
   *
   * If the distance exceeds this threshold, the computeCurrents*Ret functions will
   * return false
   *
   * @param dist
   */
  void setMaximumGradientDistance(const double dist);

  /**
   * @brief Gets the threshold on the distance between the computed gradient
   * and the desired field
   *
   * See setMaximumFieldDistance
   *
   * @return the distance parameter
   */
  double getMaximumGradientDistance() const;

  /**
   * @brief Sets whether the model will throw an exception
   * in case the return status of the solver is false due a failed computeCurrents
   * call
   *
   * @param enable: true if exceptions should be thrown
   */
  void setDoThroughExceptions(const bool enable);

  /**
   * @brief Returns true if the model will throw an exception
   * in case the return status of the solver is false due a failed computeCurrents
   * call
   *
   * @return true if exceptions should be thrown
   */
  bool getDoThroughExceptions() const;

  /**
   * @brief Gets the Ceres solver option
   *
   * @return the solver options
   */
  ceres::Solver::Options getSolverOptions() const;

  virtual CurrentsVec computeCurrentsFromField(const PositionVec& position,
                                               const FieldVec& field) const;

  virtual CurrentsVec computeCurrentsFromFieldGradient5(const PositionVec& position,
                                                        const FieldVec& field,
                                                        const Gradient5Vec& gradient) const;

  virtual CurrentsVec computeCurrentsFromFieldDipoleGradient3(const PositionVec& position,
                                                              const FieldVec& field,
                                                              const DipoleVec& dipole,
                                                              const Gradient3Vec& gradient) const;

  /**
   * @brief Computes currents but also checks if the solver succeeded
   *
   * You can call this instead of computeCurrentsFromField to see if
   * solver succeeded. If the return is false, you should not
   * trust the output currents vector
   *
   * @param position: the position of field
   * @param field: the magnetic field vector in T
   * @param currents: the computed currents vector in A
   *
   * @return std::pair with the first value being true if the solver succeeded,
   * and the second value being the computed currents
   */
  std::pair<bool, CurrentsVec> computeCurrentsFromFieldRet(const PositionVec& position,
                                                           const FieldVec& field) const;

  /**
   * @brief Computes currents but also checks if the solver succeeded
   *
   * You can call this instead of computeCurrentsFromFieldGradient5 to see if
   * solver succeeded. If the return is false, you should not
   * trust the output currents vector
   *
   * @param position: the position of field
   * @param field: the magnetic field vector in T
   * @param gradient: the magnetic field gradient vector in T/m
   * @param currents: the computed currents vector in A
   *
   * @return std::pair with the first value being true if the solver succeeded,
   * and the second value being the computed currents
   */
  std::pair<bool, CurrentsVec> computeCurrentsFromFieldGradient5Ret(
      const PositionVec& position, const FieldVec& field, const Gradient5Vec& gradient) const;

  /**
   * @brief Returns true if the solver succeeded
   *
   * You can call this instead of computeCurrentsFromFieldDipoleGradient3 to see if
   * solver succeeded. If the return is false, you should not
   * trust the output currents vector
   *
   * @param position: the position of field
   * @param field: the magnetic field vector in T
   * @param dipole: the dipole vector (unitless)
   * @param gradient: the magnetic field 3D gradient vector in T/m
   * @param currents: the computed currents vector in A
   *
   * @return std::pair with the first value being true if the solver succeeded,
   * and the second value being the computed currents
   */
  std::pair<bool, CurrentsVec> computeCurrentsFromFieldDipoleGradient3Ret(
      const PositionVec& position, const FieldVec& field, const DipoleVec& dipole,
      const Gradient3Vec& gradient) const;

 protected:
  std::string cal_name_;
  ForwardModel::Ptr p_forward_model_;
  CurrentsVec initial_currents_;
  int num_coils_;
  ceres::Solver::Options ceres_options_;

  /**
   * @brief A vector of size Ne setting the lower bounds on currents
   */
  CurrentsVec min_currents_;

  /**
   * @brief A vector of size Ne setting the upper bounds on currents
   */
  CurrentsVec max_currents_;

  /**
   * @brief if the distance between the computed field
   * and the desired field exceeds this, the solution is considered invalid
   */
  float max_field_dist_;

  /**
   * @brief if the distance between the computed gradient vector
   * and the desired gradient exceeds this, the solution is considered invalid
   */
  float max_gradient_dist_;

  /**
   * @brief If true, this will print the ceres solver summary at the end of a currents computation
   */
  bool do_print_summary_;

  /**
   * @brief If this is false, the currents L2 regularization term is ignored
   */
  bool do_minimize_currents_;

  /**
   * @brief The lambda parameter that weights field error residuals to the L2 norm of currents
   */
  double weight_currents_;

  /**
   * @brief If true, will throw an exception if the solver return status is false
   */
  bool do_throw_exceptions_;

  bool use_analytical_gradients_;
  CurrentsJacobianFunctor::Ptr p_currents_jac_func_;

  /**
   * @brief If true, this keeps the position fixed during computeCurrents computation which
   * allows for optimizations
   */
  bool do_cache_position_;

  /**
   * @brief If true, will log all iterations to console using
   * ceres::Solver::options::trust_region_minimizer_iterations_to_dump
   */
  bool do_log_iterations_to_console_;
};

}  // namespace mag_manip
