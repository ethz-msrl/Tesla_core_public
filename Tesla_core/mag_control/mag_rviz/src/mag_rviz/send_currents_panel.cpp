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

#include "mag_rviz/send_currents_panel.h"

#include <QDoubleValidator>
#include <QFont>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QSpinBox>
#include <QVBoxLayout>

#include "mag_msgs/CurrentsStamped.h"

namespace mag_control {

SendCurrentsPanel::SendCurrentsPanel(QWidget* parent)
    : rviz::Panel(parent), currents_topic_("/desired_currents"), num_currents_(8) {
  layout_ = new QVBoxLayout;
  QHBoxLayout* topic_layout = new QHBoxLayout;
  topic_layout->addWidget(new QLabel("Currents Topic: "));

  topic_edit_ = new QLineEdit(currents_topic_);

  topic_layout->addWidget(topic_edit_);

  topic_layout->addWidget(new QLabel("N: "));

  num_currents_box_ = new QSpinBox;
  // cannot have less than 1 coil
  // Setting max to an arbitrarily high valu
  num_currents_box_->setRange(1, 16);
  num_currents_box_->setValue(num_currents_);
  topic_layout->addWidget(num_currents_box_);
  connect(num_currents_box_, SIGNAL(valueChanged(int)), this, SLOT(updateNumCurrents(int)));

  layout_->addLayout(topic_layout);

  connect(topic_edit_, SIGNAL(editingFinished()), this, SLOT(updateTopic()));

  pub_currents_ = nh_.advertise<mag_msgs::CurrentsStamped>(currents_topic_.toStdString(), 1);

  currents_layout_ = new QGridLayout;
  send_button_ = new QPushButton("Send");
  resetCurrentsLayout();

  layout_->setAlignment(Qt::AlignTop);

  setLayout(layout_);
}

void SendCurrentsPanel::load(const rviz::Config& config) {
  rviz::Panel::load(config);
  QString currents_topic;
  QString num_currents;
  if (config.mapGetString("currents_topic", &currents_topic)) {
    topic_edit_->setText(currents_topic);
    updateTopic();
  }
  if (config.mapGetString("num_currents", &num_currents)) {
    num_currents_ = num_currents.toInt();
    num_currents_box_->setValue(num_currents_);
    resetCurrentsLayout();
  }
}

void SendCurrentsPanel::save(rviz::Config config) const {
  rviz::Panel::save(config);
  config.mapSetValue("currents_topic", currents_topic_);
  config.mapSetValue("num_currents", num_currents_);
}

void SendCurrentsPanel::setTopic(const QString& topic_name) {
  if (topic_name == "") {
    ROS_DEBUG("Empty topic");
    return;
  }

  if (topic_name == currents_topic_) {
    ROS_DEBUG("Already subscribed to topic");
  }

  currents_topic_ = topic_name;
  pub_currents_.shutdown();
  pub_currents_ = nh_.advertise<mag_msgs::CurrentsStamped>(currents_topic_.toStdString(), 1);

  Q_EMIT configChanged();
}

void SendCurrentsPanel::updateNumCurrents(int num) {
  num_currents_ = num;
  resetCurrentsLayout();
}

void SendCurrentsPanel::updateTopic() { setTopic(topic_edit_->text()); }

void SendCurrentsPanel::sendCurrents() {
  mag_msgs::CurrentsStamped msg;
  msg.header.frame_id = "NA";
  msg.header.stamp = ros::Time::now();

  for (int i = 0; i < num_currents_; i++) {
    msg.currents.push_back(current_values_[i]->text().toDouble());
  }

  pub_currents_.publish(msg);
}

void SendCurrentsPanel::resetCurrentsLayout() {
  assert(coil_labels_.size() == current_values_.size());

  delete send_button_;

  for (int i = 0; i < coil_labels_.size(); i++) {
    delete coil_labels_[i];
    delete current_values_[i];
  }

  coil_labels_.clear();
  current_values_.clear();

  QLayoutItem* item;
  QLayout* sublayout;
  QWidget* widget;
  while ((item = currents_layout_->takeAt(0))) {
    if ((sublayout = item->layout()) != 0) { /* do the same for sublayout*/
    } else if ((widget = item->widget()) != 0) {
      widget->hide();
      delete widget;
    } else {
      delete item;
    }
  }

  delete currents_layout_;

  currents_layout_ = new QGridLayout;

  for (int i = 0; i < num_currents_; i++) {
    QLabel* current_label = new QLabel(QString("<b>") + QString::number(i) + QString("</b>"));
    current_label->setAlignment(Qt::AlignHCenter);
    currents_layout_->addWidget(current_label, 0, i);
    coil_labels_.push_back(current_label);
  }

  for (int i = 0; i < num_currents_; i++) {
    std::stringstream ss;
    ss << std::setw(6) << std::left << std::fixed << std::setprecision(2) << "";
    QLineEdit* current_val = new QLineEdit("0");
    current_val->setValidator(new QDoubleValidator);
    currents_layout_->addWidget(current_val, 1, i);
    current_values_.push_back(current_val);
  }

  send_button_ = new QPushButton("Send");
  connect(send_button_, SIGNAL(released()), this, SLOT(sendCurrents()));
  currents_layout_->addWidget(send_button_, 1, num_currents_);

  layout_->addLayout(currents_layout_);
}

}  // namespace mag_control

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mag_control::SendCurrentsPanel, rviz::Panel)
