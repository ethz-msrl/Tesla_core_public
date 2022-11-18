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

#include "mag_rviz/dipolegradient_panel.h"

#include <mag_msgs/DipoleGradientStamped.h>
#include <ros/subscriber.h>
#include <rosgraph_msgs/Log.h>

#include <QCheckBox>
#include <QDoubleSpinBox>
#include <QFont>
#include <QFormLayout>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QSlider>
#include <QSpinBox>
#include <QTimer>
#include <QVBoxLayout>
#include <QVector>

#include <tf/transform_datatypes.h>

namespace mag_control {

DipoleGradientPanel::DipoleGradientPanel(QWidget* parent)
    : rviz::Panel(parent),
      magfield_topic_("dipole_gradient"),
      position_topic_("intersection_selector/best_candidate") {
  layout_ = new QVBoxLayout;

  topic_edit_ = new QLineEdit(magfield_topic_);
  pos_topic_edit_ = new QLineEdit(position_topic_);
  frame_edit_ = new QLineEdit("mns");

  QGridLayout* grid_layout_1_ = new QGridLayout;

  position_feedback_checkbox_ = new QCheckBox(" ", this);
  position_feedback_checkbox_->setChecked(true);
  grid_layout_1_->addWidget(new QLabel("Mag Topic: "), 0, 0);
  grid_layout_1_->addWidget(topic_edit_, 0, 1);
  grid_layout_1_->addWidget(position_feedback_checkbox_, 1, 2);
  grid_layout_1_->addWidget(new QLabel("Pos Topic: "), 1, 0);
  grid_layout_1_->addWidget(pos_topic_edit_, 1, 1);
  grid_layout_1_->addWidget(new QLabel("Frame: "), 2, 0);
  grid_layout_1_->addWidget(frame_edit_, 2, 1);
  layout_->addLayout(grid_layout_1_);

  QGridLayout* grid_layout_2_ = new QGridLayout;

  grid_layout_2_->addWidget(new QLabel("Position (mm): "), 1, 0);
  position_x_sb_ = new QSpinBox();
  position_x_sb_->setRange(-300, 300);
  position_y_sb_ = new QSpinBox();
  position_y_sb_->setRange(-300, 300);
  position_z_sb_ = new QSpinBox();
  position_z_sb_->setRange(-300, 300);
  grid_layout_2_->addWidget(position_x_sb_, 1, 1);
  grid_layout_2_->addWidget(position_y_sb_, 1, 2);
  grid_layout_2_->addWidget(position_z_sb_, 1, 3);
  grid_layout_2_->addWidget(new QLabel("G (mT/mm): "), 2, 0);
  gradient_x_sb_ = new QDoubleSpinBox();
  gradient_x_sb_->setRange(0., 1.);
  gradient_x_sb_->setSingleStep(0.01);
  gradient_x_sb_->setDecimals(2);
  gradient_x_sb_->setRange(-100, 100);
  gradient_y_sb_ = new QDoubleSpinBox();
  gradient_y_sb_->setRange(0., 1.);
  gradient_y_sb_->setSingleStep(0.01);
  gradient_y_sb_->setDecimals(2);
  gradient_y_sb_->setRange(-100, 100);
  gradient_z_sb_ = new QDoubleSpinBox();
  gradient_z_sb_->setRange(0., 1.);
  gradient_z_sb_->setSingleStep(0.01);
  gradient_z_sb_->setDecimals(2);
  gradient_z_sb_->setRange(-100, 100);
  grid_layout_2_->addWidget(gradient_x_sb_, 2, 1);
  grid_layout_2_->addWidget(gradient_y_sb_, 2, 2);
  grid_layout_2_->addWidget(gradient_z_sb_, 2, 3);

  field_magnitude_sb_ = new QSpinBox();
  field_magnitude_sb_->setRange(0, 140);
  field_magnitude_sb_->setSingleStep(10);
  field_inclination_sb_ = new QSpinBox();
  field_inclination_sb_->setRange(-180, 180);
  field_inclination_sb_->setSingleStep(10);
  field_inclination_sb_->setWrapping(true);
  field_azimuth_sb_ = new QSpinBox();
  field_azimuth_sb_->setRange(-180, 180);
  field_azimuth_sb_->setSingleStep(10);
  field_azimuth_sb_->setWrapping(true);

  gradient_magnitude_sb_ = new QDoubleSpinBox();
  gradient_magnitude_sb_->setRange(0., 1.);
  gradient_magnitude_sb_->setSingleStep(0.01);
  gradient_magnitude_sb_->setDecimals(2);
  gradient_inclination_sb_ = new QSpinBox();
  gradient_inclination_sb_->setRange(-180, 180);
  gradient_inclination_sb_->setSingleStep(10);
  gradient_inclination_sb_->setWrapping(true);
  gradient_azimuth_sb_ = new QSpinBox();
  gradient_azimuth_sb_->setRange(-180, 180);
  gradient_azimuth_sb_->setSingleStep(10);
  gradient_azimuth_sb_->setWrapping(true);

  aligned_dipole_checkbox_ = new QCheckBox("Aligned", this);
  dipole_inclination_sb_ = new QSpinBox();
  dipole_inclination_sb_->setRange(-180, 180);
  dipole_inclination_sb_->setSingleStep(10);
  dipole_inclination_sb_->setWrapping(true);
  dipole_azimuth_sb_ = new QSpinBox();
  dipole_azimuth_sb_->setRange(-180, 180);
  dipole_azimuth_sb_->setSingleStep(10);
  dipole_azimuth_sb_->setWrapping(true);
  aligned_dipole_checkbox_->setChecked(true);
  dipole_azimuth_sb_->setEnabled(false);
  dipole_inclination_sb_->setEnabled(false);

  grid_layout_2_->addWidget(new QLabel("MF (mT)"), 3, 1);
  grid_layout_2_->addWidget(new QLabel("G (mT/mm)"), 3, 2);
  grid_layout_2_->addWidget(new QLabel("Dipole"), 3, 3);
  grid_layout_2_->addWidget(new QLabel("Magnitude"), 4, 0);
  grid_layout_2_->addWidget(field_magnitude_sb_, 4, 1);
  grid_layout_2_->addWidget(gradient_magnitude_sb_, 4, 2);
  grid_layout_2_->addWidget(aligned_dipole_checkbox_, 4, 3);
  grid_layout_2_->addWidget(new QLabel("IA (deg)"), 5, 0);
  grid_layout_2_->addWidget(field_inclination_sb_, 5, 1);
  grid_layout_2_->addWidget(gradient_inclination_sb_, 5, 2);
  grid_layout_2_->addWidget(dipole_inclination_sb_, 5, 3);
  grid_layout_2_->addWidget(new QLabel("AA (deg)"), 6, 0);
  grid_layout_2_->addWidget(field_azimuth_sb_, 6, 1);
  grid_layout_2_->addWidget(gradient_azimuth_sb_, 6, 2);
  grid_layout_2_->addWidget(dipole_azimuth_sb_, 6, 3);

  layout_->addLayout(grid_layout_2_);

  // Magnetic field signal
  connect(topic_edit_, SIGNAL(editingFinished()), this, SLOT(updateTopic()));
  connect(position_x_sb_, SIGNAL(valueChanged(int)), this, SLOT(updateDipoleGradient(int)));
  connect(position_y_sb_, SIGNAL(valueChanged(int)), this, SLOT(updateDipoleGradient(int)));
  connect(position_z_sb_, SIGNAL(valueChanged(int)), this, SLOT(updateDipoleGradient(int)));
  connect(gradient_x_sb_, SIGNAL(valueChanged(double)), this, SLOT(updateDipoleGradient(double)));
  connect(gradient_y_sb_, SIGNAL(valueChanged(double)), this, SLOT(updateDipoleGradient(double)));
  connect(gradient_z_sb_, SIGNAL(valueChanged(double)), this, SLOT(updateDipoleGradient(double)));
  connect(field_magnitude_sb_, SIGNAL(valueChanged(int)), this, SLOT(updateDipoleGradient(int)));
  connect(field_azimuth_sb_, SIGNAL(valueChanged(int)), this, SLOT(updateDipoleGradient(int)));
  connect(field_inclination_sb_, SIGNAL(valueChanged(int)), this, SLOT(updateDipoleGradient(int)));
  connect(gradient_magnitude_sb_, SIGNAL(valueChanged(double)), this,
          SLOT(updateDipoleGradient(double)));
  connect(gradient_inclination_sb_, SIGNAL(valueChanged(int)), this,
          SLOT(updateDipoleGradient(int)));
  connect(gradient_azimuth_sb_, SIGNAL(valueChanged(int)), this, SLOT(updateDipoleGradient(int)));
  connect(dipole_inclination_sb_, SIGNAL(valueChanged(int)), this, SLOT(updateDipoleGradient(int)));
  connect(dipole_azimuth_sb_, SIGNAL(valueChanged(int)), this, SLOT(updateDipoleGradient(int)));
  connect(field_inclination_sb_, SIGNAL(valueChanged(int)), this, SLOT(updateDipoleGradient(int)));
  connect(aligned_dipole_checkbox_, SIGNAL(stateChanged(int)), this, SLOT(processCheckBox(int)));

  connect(pos_topic_edit_, SIGNAL(editingFinished()), this, SLOT(updatePosTopic()));
  connect(position_feedback_checkbox_, SIGNAL(stateChanged(int)), this,
          SLOT(processPosCheckBox(int)));

  // sending currents / automatic or manual
  QGridLayout* grid_layout_3_ = new QGridLayout;

  auto_update_checkbox_ = new QCheckBox("Auto send currents", this);
  auto_update_checkbox_->setChecked(true);
  grid_layout_3_->addWidget(auto_update_checkbox_, 0, 0);

  send_button_ = new QPushButton("Send Currents");
  send_button_->setEnabled(false);
  grid_layout_3_->addWidget(send_button_, 0, 1);

  connect(auto_update_checkbox_, SIGNAL(stateChanged(int)), this, SLOT(processUpdateCheckBox(int)));
  connect(send_button_, SIGNAL(released()), this, SLOT(updateDipoleGradientManual()));

  layout_->addLayout(grid_layout_3_);

  // log
  QGridLayout* grid_layout_4_ = new QGridLayout;

  grid_layout_4_->addWidget(new QLabel("Log msgs: "), 0, 0);

  log_msgs_ = new QLabel(QString("-"));
  grid_layout_4_->addWidget(log_msgs_, 1, 0);

  layout_->addLayout(grid_layout_4_);

  layout_->setAlignment(Qt::AlignTop);
  setLayout(layout_);
  mag_field_pub_ =
      nh_.advertise<mag_msgs::DipoleGradientStamped>(magfield_topic_.toStdString(), 1000);
  mag_field_pub_unarmed_ = nh_.advertise<mag_msgs::DipoleGradientStamped>(
      magfield_topic_.toStdString() + "_unarmed", 1000);
  pos_sub_ = nh_.subscribe<geometry_msgs::PointStamped>(position_topic_.toStdString(), 1,
                                                        &DipoleGradientPanel::updatePosition, this);
  log_subscriber_ =
      nh_.subscribe<rosgraph_msgs::Log>("/rosout_agg", 1, &DipoleGradientPanel::LogCb, this);
  processPosCheckBox(true);
  processCheckBox(true);
}

void DipoleGradientPanel::load(const rviz::Config& config) {
  rviz::Panel::load(config);
  QString magfield_topic;
  if (config.mapGetString("dipole_gradient_topic", &magfield_topic)) {
    topic_edit_->setText(magfield_topic);
    updateTopic();
  }
  QString position_topic;
  if (config.mapGetString("position_topic", &position_topic)) {
    pos_topic_edit_->setText(position_topic);
    updatePosTopic();
  }
}

void DipoleGradientPanel::save(rviz::Config config) const {
  rviz::Panel::save(config);
  config.mapSetValue("dipole_gradient_topic", magfield_topic_);
  config.mapSetValue("position_topic", position_topic_);
}

void DipoleGradientPanel::setTopic(const QString& topic_name) {
  if (topic_name == "") {
    ROS_DEBUG("Empty topic");
    return;
  }

  if (topic_name == magfield_topic_) {
    ROS_DEBUG("Already subscribed to topic");
  }

  magfield_topic_ = topic_name;
  mag_field_pub_.shutdown();
  mag_field_pub_ =
      nh_.advertise<mag_msgs::DipoleGradientStamped>(magfield_topic_.toStdString(), 1000);

  Q_EMIT configChanged();
}

void DipoleGradientPanel::setPosTopic(const QString& topic_name) {
  if (topic_name == "") {
    ROS_DEBUG("Empty topic");
    return;
  }

  if (topic_name == position_topic_) {
    ROS_DEBUG("Already subscribed to topic");
  }

  position_topic_ = topic_name;
  pos_sub_.shutdown();
  pos_sub_ = nh_.subscribe<geometry_msgs::PointStamped>(position_topic_.toStdString(), 1,
                                                        &DipoleGradientPanel::updatePosition, this);

  Q_EMIT configChanged();
}

void DipoleGradientPanel::updateTopic() { setTopic(topic_edit_->text()); }

void DipoleGradientPanel::updatePosTopic() { setPosTopic(pos_topic_edit_->text()); }

void DipoleGradientPanel::updateDipoleGradient(int value) { updateDipoleGradient(); }

void DipoleGradientPanel::updateDipoleGradient(double value) { updateDipoleGradient(); }

void DipoleGradientPanel::updateDipoleGradient() {
  log_msgs_->setText("-");

  mag_msgs::DipoleGradientStamped mf_msg;
  mf_msg.header.stamp = ros::Time::now();
  mf_msg.header.frame_id = frame_edit_->text().toStdString();

  mf_msg.position.x = static_cast<double>(position_x_sb_->value()) * 0.001;
  mf_msg.position.y = static_cast<double>(position_y_sb_->value()) * 0.001;
  mf_msg.position.z = static_cast<double>(position_z_sb_->value()) * 0.001;

  double pi = std::acos(-1);
  double r = static_cast<double>(field_magnitude_sb_->value());
  double ia = static_cast<double>(field_inclination_sb_->value()) / 180 * pi;
  double aa = static_cast<double>(field_azimuth_sb_->value()) / 180 * pi;
  mf_msg.field.x = r * sin(ia) * cos(aa) * 0.001;
  mf_msg.field.y = r * sin(ia) * sin(aa) * 0.001;
  mf_msg.field.z = r * cos(ia) * 0.001;

  double gr = static_cast<double>(gradient_magnitude_sb_->value());
  double gia = static_cast<double>(gradient_inclination_sb_->value()) / 180 * pi;
  double gaa = static_cast<double>(gradient_azimuth_sb_->value()) / 180 * pi;
  double gx = static_cast<double>(gradient_x_sb_->value());
  double gy = static_cast<double>(gradient_y_sb_->value());
  double gz = static_cast<double>(gradient_z_sb_->value());

  mf_msg.gradient.x = gr * sin(gia) * cos(gaa) + gx;
  mf_msg.gradient.y = gr * sin(gia) * sin(gaa) + gy;
  mf_msg.gradient.z = gr * cos(gia) + gz;

  if (aligned_dipole_checkbox_->isChecked()) {
    dipole_inclination_sb_->setValue(field_inclination_sb_->value());
    dipole_azimuth_sb_->setValue(field_azimuth_sb_->value());
    mf_msg.dipole.x = sin(ia) * cos(aa);
    mf_msg.dipole.y = sin(ia) * sin(aa);
    mf_msg.dipole.z = cos(ia);
  } else {
    double dia = static_cast<double>(dipole_inclination_sb_->value()) / 180 * pi;
    double daa = static_cast<double>(dipole_azimuth_sb_->value()) / 180 * pi;
    mf_msg.dipole.x = sin(dia) * cos(daa);
    mf_msg.dipole.y = sin(dia) * sin(daa);
    mf_msg.dipole.z = cos(dia);
  }

  if (auto_update_checkbox_->isChecked()) {
    mag_field_pub_.publish(mf_msg);
    ROS_INFO("dipole gradient updated and published");
  } else {
    mag_field_pub_unarmed_.publish(mf_msg);
    ROS_INFO("dipole gradient updated (unarmed) and published");
  }
}

void DipoleGradientPanel::updateDipoleGradientManual() {
  log_msgs_->setText("-");

  mag_msgs::DipoleGradientStamped mf_msg;
  mf_msg.header.stamp = ros::Time::now();
  mf_msg.header.frame_id = frame_edit_->text().toStdString();

  mf_msg.position.x = static_cast<double>(position_x_sb_->value()) * 0.001;
  mf_msg.position.y = static_cast<double>(position_y_sb_->value()) * 0.001;
  mf_msg.position.z = static_cast<double>(position_z_sb_->value()) * 0.001;

  double pi = std::acos(-1);
  double r = static_cast<double>(field_magnitude_sb_->value());
  double ia = static_cast<double>(field_inclination_sb_->value()) / 180 * pi;
  double aa = static_cast<double>(field_azimuth_sb_->value()) / 180 * pi;
  mf_msg.field.x = r * sin(ia) * cos(aa) * 0.001;
  mf_msg.field.y = r * sin(ia) * sin(aa) * 0.001;
  mf_msg.field.z = r * cos(ia) * 0.001;

  double gr = static_cast<double>(gradient_magnitude_sb_->value());
  double gia = static_cast<double>(gradient_inclination_sb_->value()) / 180 * pi;
  double gaa = static_cast<double>(gradient_azimuth_sb_->value()) / 180 * pi;
  double gx = static_cast<double>(gradient_x_sb_->value());
  double gy = static_cast<double>(gradient_y_sb_->value());
  double gz = static_cast<double>(gradient_z_sb_->value());

  mf_msg.gradient.x = gr * sin(gia) * cos(gaa) + gx;
  mf_msg.gradient.y = gr * sin(gia) * sin(gaa) + gy;
  mf_msg.gradient.z = gr * cos(gia) + gz;

  if (aligned_dipole_checkbox_->isChecked()) {
    dipole_inclination_sb_->setValue(field_inclination_sb_->value());
    dipole_azimuth_sb_->setValue(field_azimuth_sb_->value());
    mf_msg.dipole.x = sin(ia) * cos(aa);
    mf_msg.dipole.y = sin(ia) * sin(aa);
    mf_msg.dipole.z = cos(ia);
  } else {
    double dia = static_cast<double>(dipole_inclination_sb_->value()) / 180 * pi;
    double daa = static_cast<double>(dipole_azimuth_sb_->value()) / 180 * pi;
    mf_msg.dipole.x = sin(dia) * cos(daa);
    mf_msg.dipole.y = sin(dia) * sin(daa);
    mf_msg.dipole.z = cos(dia);
  }

  mag_field_pub_.publish(mf_msg);

  ROS_INFO("dipole gradient updated manually");
}

void DipoleGradientPanel::processPosCheckBox(int value) {
  if (value) {
    position_x_sb_->setEnabled(false);
    position_y_sb_->setEnabled(false);
    position_z_sb_->setEnabled(false);
    frame_edit_->setEnabled(false);
  } else {
    position_x_sb_->setEnabled(true);
    position_y_sb_->setEnabled(true);
    position_z_sb_->setEnabled(true);
    frame_edit_->setEnabled(true);
  }
  updateDipoleGradient();
}

void DipoleGradientPanel::processCheckBox(int value) {
  if (value) {
    dipole_azimuth_sb_->setEnabled(false);
    dipole_inclination_sb_->setEnabled(false);
  } else {
    dipole_azimuth_sb_->setEnabled(true);
    dipole_inclination_sb_->setEnabled(true);
  }
  updateDipoleGradient();
}

void DipoleGradientPanel::updatePosition(const geometry_msgs::PointStamped::ConstPtr& msg) {
  log_msgs_->setText("-");

  if (position_feedback_checkbox_->checkState() != 0) {
    position_x_sb_->setValue(static_cast<int>(msg->point.x * 1000.0));
    position_y_sb_->setValue(static_cast<int>(msg->point.y * 1000.0));
    position_z_sb_->setValue(static_cast<int>(msg->point.z * 1000.0));
    frame_edit_->setText(QString::fromStdString(msg->header.frame_id));
  }

  ROS_DEBUG_STREAM("x = " << position_x_sb_->value());

  updateDipoleGradient();
}

void DipoleGradientPanel::processUpdateCheckBox(int value) {
  if (value) {
    send_button_->setEnabled(false);
  } else {
    send_button_->setEnabled(true);
  }
}

void DipoleGradientPanel::LogCb(const rosgraph_msgs::Log::ConstPtr& log) {
  std::stringstream logging_model_msg;
  std::string function = log->function;
  int level = log->level;

  if (function == "BackwardModel::dipoleGradientSub") {
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
PLUGINLIB_EXPORT_CLASS(mag_control::DipoleGradientPanel, rviz::Panel)
