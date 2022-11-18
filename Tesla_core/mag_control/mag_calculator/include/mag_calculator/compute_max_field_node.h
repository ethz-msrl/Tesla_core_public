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

#include <mag_msgs/FieldStamped.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <memory>
#include "mag_manip/backward_model.h"
#include "mag_manip/emns_parameters.h"

namespace mag_calculator {

/**
 * @brief A node for computing the maximum magnetic field that can be reached in a given direction
 *
 * Note that this node is only implemented as a demonstration on how to use the
 * mag_manip::EMNSParameters interface and the mag_manip::getMaxFieldMagnitudeAlignedWithTargetField
 * function
 *
 * This function mirrors mag_calculator:computeMaxField.py in functionality
 *
 * Subscribes to /backward_model/field as a mag_msgs::FieldStamped messages
 * Publishes on /max_field as a std_msgs::Float32 message
 */
class ComputeMaxFieldNode {
 public:
  /**
   * @brief Constructor
   *
   * @param nh: Global ros::NodeHandle
   * @param p_emns_params: a pointer to the emns_params. This should be loaded from a YAML file
   * @param p_model: a pointer to the backward model used to compute the maximum. This should be
   * created using
   * mag_manip::BackwardModelFactory
   */
  ComputeMaxFieldNode(ros::NodeHandle& nh, mag_manip::EMNSParameters::Ptr p_emns_params,
                      mag_manip::BackwardModel::Ptr p_model);

 private:
  void fieldSub(const mag_msgs::FieldStampedConstPtr msg);

  ros::NodeHandle nh_;
  ros::Subscriber sub_mag_field_;
  ros::Publisher pub_max_field_mag_;
  tf::TransformListener listener_; /**< Listens for transforms between mns and the frame indictated
                                      by incoming magnetic messages */

  mag_manip::EMNSParameters::Ptr p_emns_params_;
  mag_manip::BackwardModel::Ptr p_model_;
};
}  // namespace mag_calculator
