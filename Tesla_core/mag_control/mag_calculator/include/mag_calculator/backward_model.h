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

#include <ros/ros.h>
#include <ros/time.h>

#include <Eigen/Dense>
#include <memory>
#include <string>

#include "mag_manip/backward_model.h"
#include "tf/transform_listener.h"

// msgs
#include "ecb_msgs/CurrentsBlockStamped.h"
#include "mag_msgs/DipoleGradientStamped.h"
#include "mag_msgs/FieldGradient5Stamped.h"
#include "mag_msgs/FieldStamped.h"

// srv
#include "mag_calculator/load_calibration_file.h"

namespace mag_calculator {

/** @brief A simple node for setting magnetic fields and gradients on a magnetic
 *manipulation system.
 *
 * BackwardModel should allow users to set quasi-static fields and
 *gradients by way of ROS messages.
 *
 * It is compatible with both ECB type systems like the MFG or OctoMag and with
 *other systems by both broadcasting ecb_msgs/CurrentsBlock and
 *mag_msgs/CurrentsStamped messages.
 *
 * It communicates with magnetic controllers of the ECB type by broadcasting ECB
 *current block messages over the standard (for now) ECB message topic
 * /ECB/des_currents_block
 *
 **/
class BackwardModel {
 public:
  /**
   * @brief Constructor
   *
   * @param nh: public nodehandle
   */
  BackwardModel(ros::NodeHandle& nh, ros::NodeHandle& nh_private);

 private:
  /**
   * @brief Takes a vector of currents and sends both a
   * mag_msgs/CurrentStamped message and a ecb_msgs/CurrentsBlockStamped
   * message
   *
   * Publishes on both the pub_currents_ and the pub_currents_block_
   * publishers. The default topic are /desired_currents and
   * /ECB/des_currents_block messages respectively.
   * @param currents vector. The length should be 8.
   */
  void sendCurrents(const mag_manip::CurrentsVec& currents);

  ////////////////////
  ///   Callbacks  ///
  ////////////////////

  /**
   * @brief Takes a magnetic field message and sends currents messages.
   *
   * Sends currents using BackwardModel::sendCurrents
   *
   * Transforms the position from the frame in msg's header to the frame
   * called mns. The field is calculated at this position. It also transforms
   * the field vector from the frame in the msg's header to the frame called
   * mns.
   *
   * The field is inverted if the is_field_inverted_ parameter is set.
   *
   * The field is rescaled uniformly to the max_field_intensity_T_ value if
   * the field maximum is exceeded.
   *
   * @param msg: a mag_msgs/FieldStamped message with the desired magnetic
   * field.
   */
  void fieldSub(const mag_msgs::FieldStamped::ConstPtr& msg);

  /**
   * @brief Takes a dipole gradient message and sends currents.
   *
   * Sends currents using BackwardModel::sendCurrents
   *
   * Transforms the position, the gradient vector, and the dipole vector from
   * the frame in msg's header to the frame called mns. The field is
   * calculated at the position in the mns frame.
   *
   * @param msg: A dipole gradient to be converted to currents.
   *
   */
  void dipoleGradientSub(const mag_msgs::DipoleGradientStamped::ConstPtr& msg);

  /**
   * @brief Takes a mag_msgs/FieldGradient5Stamped message and sends current
   * messages.
   *
   * Sends currents using BackwardModel::sendCurrents
   *
   * DEPRECATED. Does not convert the position, field, or gradient vectors to
   * the mns frame.
   *
   * The field is rescaled uniformly to the max_field_intensity_T_ value if
   * the field maximum is exceeded.
   *
   * The Gradient is rescaled uniformly to the max_gradient_intensity_T_m_
   * value if the field maximum is exceeded.
   *
   *
   * @param msg containing the field and 5D gradient to be converted to
   * currents.
   */
  void fieldGradient5Sub(const mag_msgs::FieldGradient5Stamped::ConstPtr& msg);

  /**
   * @brief Resets the calibration file.
   *
   * @param req: request
   * @param resp: response
   *
   * @return true if file loading succeeded.
   */
  bool loadCalibrationSrv(mag_calculator::load_calibration_file::Request& req,
                          mag_calculator::load_calibration_file::Response& resp);

  ros::NodeHandle nh_;         /**< public nodehandle */
  ros::NodeHandle nh_private_; /**< private nodehandle */

  mag_manip::BackwardModel::Ptr
      p_backward_model_;   /**< The model used to compute currents from magnetic fields */
  unsigned int num_coils_; /**< The number of electromagnets in the magnetic system */

  std::string cal_path_; /**< The path to the model calibration file */

  std::string cal_type_; /**< The type of mag_manip calibration associated with cal_path_ */

  bool send_ecb_block_msg_; /** If true, also sends a block currents message to the ECB */
  bool ecb_direct_;         /**< If true, uses the ECB in direct mode */

  double max_field_intensity_T_; /**< Maximum allowed field intensity */

  double max_gradient_intensity_T_m_; /**< maximum allowed gradient intensity */

  bool is_field_inverted_; /**< Inverted magnetic field
                             Use when magnet has inverted magnetization
                            */

  ros::ServiceServer srv_load_cal_file_; /**< Service for loading a magnetic calibration file */

  ros::Subscriber sub_mag_field_;       /**< Subscriber for incoming magnetic field */
  ros::Subscriber sub_dipole_gradient_; /**< Subscriber for incoming dipole gradient */
  ros::Subscriber sub_field_gradient5_; /**< Subscriber for incoming field, gradient5 */

  ros::Publisher pub_currents_block_; /**< Publishes ECB current block messages */

  ros::Publisher pub_currents_; /**< Publishes CurrentStamped messages */

  tf::TransformListener listener_; /**< Listens for transforms between mns and the frame indictated
                                      by incoming magnetic messages */
};
}  // namespace mag_calculator
