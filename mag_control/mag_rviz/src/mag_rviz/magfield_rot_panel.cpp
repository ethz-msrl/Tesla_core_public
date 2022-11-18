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

#include <math.h>
#include <string>

#include "mag_rviz/magfield_rot_panel.h"

#include <mag_msgs/FieldStamped.h>
#include <ros/subscriber.h>
#include <rosgraph_msgs/Log.h>
#include <QCheckBox>
#include <QFont>
#include <QFormLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QSlider>
#include <QSpinBox>
#include <QTimer>
#include <QVBoxLayout>
#include <QVector>

#include <tf/transform_datatypes.h>
// #include <tf/Vector3.h>

namespace mag_control {

MagFieldRotPanel::MagFieldRotPanel(QWidget* parent)
    : rviz::Panel(parent), magfield_topic_("magnetic_field"), rot_angle_(0.), rot_direction_(1.) {
  layout_ = new QVBoxLayout;
  gb_initial_axis_ = new QGroupBox(tr("Initial magnetic field"));
  gb_rotation_axis_ = new QGroupBox(tr("Rotation axis"));
  QHBoxLayout* topic_layout = new QHBoxLayout;
  topic_layout->addWidget(new QLabel("Magnetic Field Topic: "));

  topic_edit_ = new QLineEdit(magfield_topic_);
  topic_layout->addWidget(topic_edit_);
  layout_->addLayout(topic_layout);

  magnitude_slider_ = new QSpinBox();
  magnitude_slider_->setRange(0, 140);
  magnitude_slider_->setSingleStep(10);
  inclination_angle_sb_ = new QSpinBox();
  inclination_angle_sb_->setRange(-180, 180);
  inclination_angle_sb_->setSingleStep(10);
  inclination_angle_sb_->setWrapping(true);
  azimuth_angle_sb_ = new QSpinBox();
  azimuth_angle_sb_->setRange(-180, 180);
  azimuth_angle_sb_->setSingleStep(10);
  azimuth_angle_sb_->setWrapping(true);

  rot_checkbox_ = new QCheckBox("Start rotation", this);
  inclination_angle_rot_sb_ = new QSpinBox();
  inclination_angle_rot_sb_->setRange(-180, 180);
  inclination_angle_rot_sb_->setSingleStep(10);
  inclination_angle_rot_sb_->setWrapping(true);
  azimuth_angle_rot_sb_ = new QSpinBox();
  azimuth_angle_rot_sb_->setRange(-180, 180);
  azimuth_angle_rot_sb_->setSingleStep(10);
  azimuth_angle_rot_sb_->setWrapping(true);

  rot_speed_ = new QSpinBox();
  rot_speed_->setRange(0, 720);
  rot_speed_->setSingleStep(10);
  rot_speed_->setWrapping(true);
  min_angle_rot_ = new QSpinBox();
  min_angle_rot_->setRange(-360, 0);
  min_angle_rot_->setSingleStep(10);
  min_angle_rot_->setWrapping(true);
  max_angle_rot_ = new QSpinBox();
  max_angle_rot_->setRange(0, 360);
  max_angle_rot_->setSingleStep(10);
  max_angle_rot_->setWrapping(true);

  QFormLayout* formLayout_send_rotation = new QFormLayout;
  formLayout_send_rotation->addRow(tr("Rotation Speed (deg/s):"), rot_speed_);
  formLayout_send_rotation->addRow(tr("Start/Stop:"), rot_checkbox_);
  layout_->addLayout(formLayout_send_rotation);

  QFormLayout* formLayout_init_axis = new QFormLayout;
  formLayout_init_axis->addRow(tr("&Magnitude (mT):"), magnitude_slider_);
  formLayout_init_axis->addRow(tr("&Inclination Angle (deg):"), inclination_angle_sb_);
  formLayout_init_axis->addRow(tr("&Azimuth Angle (deg):"), azimuth_angle_sb_);

  gb_initial_axis_->setLayout(formLayout_init_axis);
  layout_->addWidget(gb_initial_axis_);

  QFormLayout* formLayout_rotation_axis = new QFormLayout;
  formLayout_rotation_axis->addRow(tr("Minimal Angle (deg):"), min_angle_rot_);
  formLayout_rotation_axis->addRow(tr("Maximal Angle (deg):"), max_angle_rot_);
  formLayout_rotation_axis->addRow(tr("&Inclination Angle (deg):"), inclination_angle_rot_sb_);
  formLayout_rotation_axis->addRow(tr("&Azimuth Angle (deg):"), azimuth_angle_rot_sb_);
  gb_rotation_axis_->setLayout(formLayout_rotation_axis);
  layout_->addWidget(gb_rotation_axis_);

  QFormLayout* formLayout_log = new QFormLayout;
  formLayout_log->addRow(tr("&Log msgs:"), new QLabel(QString(" ")));
  log_msgs_ = new QLabel(QString(""));
  formLayout_log->addRow(log_msgs_);
  layout_->addLayout(formLayout_log);

  rot_timer_ = new QTimer();
  rot_timer_->setInterval(50);

  // Magnetic field signal
  connect(topic_edit_, SIGNAL(editingFinished()), this, SLOT(updateTopic()));
  connect(magnitude_slider_, SIGNAL(valueChanged(int)), this, SLOT(updateMagVector(int)));
  connect(azimuth_angle_sb_, SIGNAL(valueChanged(int)), this, SLOT(updateMagVector(int)));
  connect(inclination_angle_sb_, SIGNAL(valueChanged(int)), this, SLOT(updateMagVector(int)));
  connect(rot_timer_, SIGNAL(timeout()), this, SLOT(processRotTimer()));
  connect(rot_checkbox_, SIGNAL(stateChanged(int)), this, SLOT(processCheckBox(int)));

  setLayout(layout_);
  mag_field_pub_ = nh_.advertise<mag_msgs::FieldStamped>(magfield_topic_.toStdString(), 1000);
  log_subscriber_ =
      nh_.subscribe<rosgraph_msgs::Log>("/rosout_agg", 1, &MagFieldRotPanel::LogCb, this);
}

void MagFieldRotPanel::load(const rviz::Config& config) {
  rviz::Panel::load(config);
  QString magfield_topic;
  if (config.mapGetString("magfield_topic", &magfield_topic)) {
    topic_edit_->setText(magfield_topic);
    updateTopic();
  }
}

void MagFieldRotPanel::save(rviz::Config config) const {
  rviz::Panel::save(config);
  config.mapSetValue("magfield_topic", magfield_topic_);
}

void MagFieldRotPanel::setTopic(const QString& topic_name) {
  if (topic_name == "") {
    ROS_DEBUG("Empty topic");
    return;
  }

  if (topic_name == magfield_topic_) {
    ROS_DEBUG("Already subscribed to topic");
  }

  magfield_topic_ = topic_name;
  mag_field_pub_.shutdown();
  mag_field_pub_ = nh_.advertise<mag_msgs::FieldStamped>(magfield_topic_.toStdString(), 1000);

  Q_EMIT configChanged();
}

void MagFieldRotPanel::updateTopic() { setTopic(topic_edit_->text()); }

void MagFieldRotPanel::updateMagVector(int value) {
  log_msgs_->setText("-");

  mag_msgs::FieldStamped mf_msg;
  mf_msg.header.stamp = ros::Time::now();
  mf_msg.header.frame_id = "mns";

  double pi = std::acos(-1);
  double r = static_cast<double>(magnitude_slider_->value());
  double ia = static_cast<double>(inclination_angle_sb_->value()) / 180 * pi;
  double aa = static_cast<double>(azimuth_angle_sb_->value()) / 180 * pi;
  mf_msg.field.vector.x = r * sin(ia) * cos(aa) * 0.001;
  mf_msg.field.vector.y = r * sin(ia) * sin(aa) * 0.001;
  mf_msg.field.vector.z = r * cos(ia) * 0.001;

  mf_msg.field.position.x = 0;
  mf_msg.field.position.y = 0;
  mf_msg.field.position.z = 0;

  mag_field_pub_.publish(mf_msg);
}

void MagFieldRotPanel::processRotTimer() {
  log_msgs_->setText("-");

  // Rotation axis
  double pi = std::acos(-1);
  double rot_ia = static_cast<double>(inclination_angle_rot_sb_->value()) / 180 * pi;
  double rot_aa = static_cast<double>(azimuth_angle_rot_sb_->value()) / 180 * pi;
  tf::Vector3 axis(sin(rot_ia) * cos(rot_aa), sin(rot_ia) * sin(rot_aa), cos(rot_ia));

  // Magnetic field
  double r = static_cast<double>(magnitude_slider_->value());
  double ia = static_cast<double>(inclination_angle_sb_->value()) / 180 * pi;
  double aa = static_cast<double>(azimuth_angle_sb_->value()) / 180 * pi;
  tf::Vector3 mf(sin(ia) * cos(aa), sin(ia) * sin(aa), cos(ia));

  // Rotation Speed
  double angleIncr = rot_speed_->value() / 20. / 180. * pi;

  if (min_angle_rot_->value() == 0 & max_angle_rot_->value() == 0) {
    // rot_angle_ = rot_angle_ + angleIncr;
    rot_direction_ = 1.;
  } else {
    if (rot_direction_ > 0.) {
      if ((rot_angle_ + angleIncr) > (max_angle_rot_->value() / 180. * pi)) {
        rot_direction_ = -1.;
      }
    } else {
      if ((rot_angle_ - angleIncr) < (min_angle_rot_->value() / 180. * pi)) {
        rot_direction_ = 1.;
      }
    }
  }

  rot_angle_ += (angleIncr * rot_direction_);

  mf = mf.rotate(axis, rot_angle_);

  mag_msgs::FieldStamped mf_msg;
  mf_msg.header.stamp = ros::Time::now();
  mf_msg.header.frame_id = "mns";

  mf_msg.field.vector.x = r * mf.x() * 0.001;
  mf_msg.field.vector.y = r * mf.y() * 0.001;
  mf_msg.field.vector.z = r * mf.z() * 0.001;

  mf_msg.field.position.x = 0;
  mf_msg.field.position.y = 0;
  mf_msg.field.position.z = 0;

  mag_field_pub_.publish(mf_msg);

  // updateMagVector(1);
}

void MagFieldRotPanel::processCheckBox(int value) {
  if (value)
    rot_timer_->start();
  else
    rot_timer_->stop();
}

void MagFieldRotPanel::LogCb(const rosgraph_msgs::Log::ConstPtr& log) {
  std::stringstream logging_model_msg;
  std::string function = log->function;
  int level = log->level;

  if (function == "BackwardModel::fieldSub") {
    switch (level) {
      case (rosgraph_msgs::Log::DEBUG): {
        logging_model_msg << std::setw(6) << "[DEBUG]: " << log->msg;
        break;
      }
      case (rosgraph_msgs::Log::INFO): {
        logging_model_msg << std::setw(6) << "[INFO]: " << log->msg;
        break;
      }
      case (rosgraph_msgs::Log::WARN): {
        logging_model_msg << std::setw(6) << "[WARN]: " << log->msg;
        break;
      }
      case (rosgraph_msgs::Log::ERROR): {
        logging_model_msg << std::setw(6) << "[ERROR]:" << log->msg;
        break;
      }
      case (rosgraph_msgs::Log::FATAL): {
        logging_model_msg << std::setw(6) << "[FATAL]:" << log->msg;
        break;
      }
    }
    log_msgs_->setText(logging_model_msg.str().c_str());
    log_msgs_->setWordWrap(true);
  }
}

}  // namespace mag_control

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mag_control::MagFieldRotPanel, rviz::Panel)
