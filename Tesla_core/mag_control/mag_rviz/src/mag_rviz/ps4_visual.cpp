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

#include "mag_rviz/ps4_visual.h"

namespace mag_control {

Ps4Visual::Ps4Visual(QWidget* parent) : rviz::Panel(parent) {
  mode_ = 0;  // gradient mode

  grad_magn_ = 0.0;
  grad_magn_armed_ = 0.0;

  mf_magn_ = 0.0;
  mf_magn_armed_ = 0.0;

  dg_sub_ = nh_.subscribe<mag_msgs::DipoleGradientStamped>("/ps4_controller/target_dg", 1,
                                                           &Ps4Visual::dg_cb, this);
  dg_armed_sub_ = nh_.subscribe<mag_msgs::DipoleGradientStamped>("/ps4_controller/target_dg_armed",
                                                                 1, &Ps4Visual::dg_armed_cb, this);
  ps4_sub_ = nh_.subscribe<std_msgs::Float64MultiArray>("/ps4_controller/instructions", 1,
                                                        &Ps4Visual::ps4_cb, this);

  layout_ = new QVBoxLayout;
  QGridLayout* grid_layout_1_ = new QGridLayout;

  grad_magn_lab_ = new QLabel("");
  grad_magn_lab_->setNum(0.000);
  grad_magn_armed_lab_ = new QLabel("");
  grad_magn_armed_lab_->setNum(0.000);

  mf_magn_lab_ = new QLabel("");
  mf_magn_lab_->setNum(0.000);
  mf_magn_armed_lab_ = new QLabel("");
  mf_magn_armed_lab_->setNum(0.000);

  mode_field_ = new QPushButton("Field");
  mode_field_->setEnabled(false);
  mode_gradient_ = new QPushButton("Gradient");
  mode_gradient_->setEnabled(true);

  grid_layout_1_->addWidget(new QLabel("Unarmed"), 0, 1);
  grid_layout_1_->addWidget(new QLabel("Armed"), 0, 2);

  grid_layout_1_->addWidget(new QLabel("Grad Magn: "), 1, 0);
  grid_layout_1_->addWidget(grad_magn_lab_, 1, 1);
  grid_layout_1_->addWidget(grad_magn_armed_lab_, 1, 2);

  grid_layout_1_->addWidget(new QLabel("Field Magn: "), 2, 0);
  grid_layout_1_->addWidget(mf_magn_lab_, 2, 1);
  grid_layout_1_->addWidget(mf_magn_armed_lab_, 2, 2);

  layout_->addLayout(grid_layout_1_);

  QGridLayout* grid_layout_2_ = new QGridLayout;

  grid_layout_2_->addWidget(new QLabel("Mode:"), 0, 0);
  grid_layout_2_->addWidget(mode_field_, 0, 1);
  grid_layout_2_->addWidget(mode_gradient_, 0, 2);

  layout_->addLayout(grid_layout_2_);

  layout_->setAlignment(Qt::AlignTop);
  setLayout(layout_);
}

void Ps4Visual::dg_cb(const mag_msgs::DipoleGradientStamped::ConstPtr& msg) {
  // ROS_INFO("dg received");
  grad_magn_ = std::sqrt(msg->gradient.x * msg->gradient.x + msg->gradient.y * msg->gradient.y +
                         msg->gradient.z * msg->gradient.z);
  grad_magn_lab_->setNum(grad_magn_);

  mf_magn_ = std::sqrt(msg->field.x * msg->field.x + msg->field.y * msg->field.y +
                       msg->field.z * msg->field.z);
  mf_magn_lab_->setNum(mf_magn_);
}

void Ps4Visual::dg_armed_cb(const mag_msgs::DipoleGradientStamped::ConstPtr& msg) {
  // ROS_INFO("dg armed received");
  grad_magn_armed_ =
      std::sqrt(msg->gradient.x * msg->gradient.x + msg->gradient.y * msg->gradient.y +
                msg->gradient.z * msg->gradient.z);
  grad_magn_armed_lab_->setNum(grad_magn_armed_);

  mf_magn_armed_ = std::sqrt(msg->field.x * msg->field.x + msg->field.y * msg->field.y +
                             msg->field.z * msg->field.z);
  mf_magn_armed_lab_->setNum(mf_magn_armed_);
}

void Ps4Visual::ps4_cb(const std_msgs::Float64MultiArray::ConstPtr& msg) {
  if (msg->data[6] == 0.0) {
    mode_field_->setEnabled(false);
    mode_gradient_->setEnabled(true);
  } else if (msg->data[6] == 1.0) {
    mode_field_->setEnabled(true);
    mode_gradient_->setEnabled(false);
  }
}

}  // namespace mag_control

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mag_control::Ps4Visual, rviz::Panel)
