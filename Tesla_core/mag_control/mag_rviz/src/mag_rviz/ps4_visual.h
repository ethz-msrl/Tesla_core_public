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

#include <rviz/panel.h>

#include <ros/ros.h>
#include <ros/subscriber.h>

#include <mag_msgs/DipoleGradientStamped.h>
#include <std_msgs/Float64MultiArray.h>

#include <QGridLayout>
#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>

namespace mag_control {

class Ps4Visual : public rviz::Panel {
  Q_OBJECT

 public:
  explicit Ps4Visual(QWidget* parent = 0);

 protected slots:

  void dg_cb(const mag_msgs::DipoleGradientStamped::ConstPtr& msg);
  void dg_armed_cb(const mag_msgs::DipoleGradientStamped::ConstPtr& msg);
  void ps4_cb(const std_msgs::Float64MultiArray::ConstPtr& msg);

 protected:
  int mode_;

  double grad_magn_;
  double grad_magn_armed_;

  double mf_magn_;
  double mf_magn_armed_;

  ros::Subscriber dg_sub_;
  ros::Subscriber dg_armed_sub_;
  ros::Subscriber ps4_sub_;

  ros::NodeHandle nh_;

  QVBoxLayout* layout_;

  QLabel* grad_magn_lab_;
  QLabel* grad_magn_armed_lab_;
  QLabel* mf_magn_lab_;
  QLabel* mf_magn_armed_lab_;

  QPushButton* mode_field_;
  QPushButton* mode_gradient_;
};

}  // namespace mag_control
