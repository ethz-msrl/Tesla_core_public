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

#include "mag_rviz/magfield_panel.h"

#include <mag_msgs/FieldStamped.h>
#include <ros/subscriber.h>
#include <rosgraph_msgs/Log.h>

#include <QFont>
#include <QFormLayout>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QSlider>
#include <QSpinBox>
#include <QVBoxLayout>
#include <QVector>

namespace mag_control {

MagFieldPanel::MagFieldPanel(QWidget *parent)
    : rviz::Panel(parent), magfield_topic_("/desired_field") {
  layout_ = new QVBoxLayout;

  topic_edit_ = new QLineEdit(magfield_topic_);
  log_msgs_ = new QLabel(QString(""));
  frame_edit_ = new QLineEdit("mns");
  magnitude_slider_ = new QSpinBox();
  magnitude_slider_->setRange(0, 140);
  magnitude_slider_->setSingleStep(10);
  inclination_angle_sb_ = new QSpinBox();
  inclination_angle_sb_->setRange(-180, 180);
  inclination_angle_sb_->setSingleStep(5);
  inclination_angle_sb_->setWrapping(true);
  azimuth_angle_sb_ = new QSpinBox();
  azimuth_angle_sb_->setRange(-180, 180);
  azimuth_angle_sb_->setSingleStep(5);
  azimuth_angle_sb_->setWrapping(true);

  QFormLayout *formLayout = new QFormLayout;
  formLayout->addRow(tr("&Magnetic Field Topic:"), topic_edit_);
  formLayout->addRow(tr("&Frame:"), frame_edit_);
  formLayout->addRow(tr("&Magnitude (mT):"), magnitude_slider_);
  formLayout->addRow(tr("&Inclination Angle (deg):"), inclination_angle_sb_);
  formLayout->addRow(tr("&Azimuth Angle (deg):"), azimuth_angle_sb_);
  formLayout->addRow(tr("&Log msgs:"), new QLabel(QString(" ")));
  formLayout->addRow(log_msgs_);
  layout_->addLayout(formLayout);

  connect(topic_edit_, SIGNAL(editingFinished()), this, SLOT(updateTopic()));
  connect(magnitude_slider_, SIGNAL(valueChanged(int)), this, SLOT(updateMagVector(int)));
  connect(azimuth_angle_sb_, SIGNAL(valueChanged(int)), this, SLOT(updateMagVector(int)));
  connect(inclination_angle_sb_, SIGNAL(valueChanged(int)), this, SLOT(updateMagVector(int)));

  setLayout(layout_);
  mag_field_pub_ = nh_.advertise<mag_msgs::FieldStamped>(magfield_topic_.toStdString(), 1000);
  log_subscriber_ =
      nh_.subscribe<rosgraph_msgs::Log>("/rosout_agg", 1, &MagFieldPanel::LogCb, this);
}

void MagFieldPanel::load(const rviz::Config &config) {
  rviz::Panel::load(config);
  QString magfield_topic;
  if (config.mapGetString("magfield_topic", &magfield_topic)) {
    topic_edit_->setText(magfield_topic);
    updateTopic();
  }

  QString frame_id;
  if (config.mapGetString("frame_id", &frame_id)) {
    frame_edit_->setText(frame_id);
  }
}

void MagFieldPanel::save(rviz::Config config) const {
  rviz::Panel::save(config);
  config.mapSetValue("magfield_topic", magfield_topic_);
  config.mapSetValue("frame_id", frame_edit_->text());
}

void MagFieldPanel::setTopic(const QString &topic_name) {
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

void MagFieldPanel::updateTopic() { setTopic(topic_edit_->text()); }

void MagFieldPanel::updateMagVector(int value) {
  mag_msgs::FieldStamped mf_msg;
  mf_msg.header.stamp = ros::Time::now();
  mf_msg.header.frame_id = frame_edit_->text().toStdString();

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
  log_msgs_->setText("-");
}

void MagFieldPanel::LogCb(const rosgraph_msgs::Log::ConstPtr &log) {
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
PLUGINLIB_EXPORT_CLASS(mag_control::MagFieldPanel, rviz::Panel)
