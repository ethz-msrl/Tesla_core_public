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

#include "mag_rviz/currents_panel.h"

#include <ros/subscriber.h>

#include <QFont>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QVector>

namespace mag_control {

CurrentsPanel::CurrentsPanel(QWidget* parent)
    : rviz::Panel(parent), currents_topic_("/actual_currents"), num_currents_(8) {
  layout_ = new QVBoxLayout;
  QHBoxLayout* topic_layout = new QHBoxLayout;
  topic_layout->addWidget(new QLabel("Currents Topic: "));

  topic_edit_ = new QLineEdit(currents_topic_);
  topic_layout->addWidget(topic_edit_);

  layout_->addLayout(topic_layout);

  connect(topic_edit_, SIGNAL(editingFinished()), this, SLOT(updateTopic()));

  currents_layout_ = new QGridLayout;

  resetCurrentsLayout();

  setLayout(layout_);
}

void CurrentsPanel::load(const rviz::Config& config) {
  rviz::Panel::load(config);
  QString currents_topic;
  QString num_currents;
  if (config.mapGetString("currents_topic", &currents_topic)) {
    topic_edit_->setText(currents_topic);
    updateTopic();
  }
  if (config.mapGetString("num_currents", &num_currents)) {
    num_currents_ = num_currents.toInt();
    resetCurrentsLayout();
  }
}

void CurrentsPanel::save(rviz::Config config) const {
  rviz::Panel::save(config);
  config.mapSetValue("currents_topic", currents_topic_);
  config.mapSetValue("num_currents", num_currents_);
}

void CurrentsPanel::setTopic(const QString& topic_name) {
  if (topic_name == "") {
    ROS_DEBUG("Empty topic");
    return;
  }

  if (topic_name == currents_topic_) {
    ROS_DEBUG("Already subscribed to topic");
  }

  currents_topic_ = topic_name;
  currents_sub_.shutdown();
  currents_sub_ =
      nh_.subscribe(currents_topic_.toStdString(), 1, &CurrentsPanel::updateCurrents, this);

  Q_EMIT configChanged();
}

void CurrentsPanel::updateTopic() { setTopic(topic_edit_->text()); }

void CurrentsPanel::updateCurrents(const mag_msgs::CurrentsStampedConstPtr& msg) {
  std::stringstream ss;

  if (num_currents_ != msg->currents.size()) {
    ROS_DEBUG("changing the number of coils to %i", msg->currents.size());

    /* Delete the current layout and recreate with the right number
     * of coils */
    num_currents_ = msg->currents.size();

    resetCurrentsLayout();
  }

  for (int i = 0; i < num_currents_; i++) {
    std::stringstream ss;
    ss << std::setw(6) << std::left << std::fixed << std::setprecision(2) << msg->currents[i];
    current_values_[i]->setText(ss.str().c_str());
  }

  //    for(int i=0; i < num_currents_; i++) {
  //        ss << std::setw(6) << std::left << std::fixed << std::setprecision(2) <<
  //        msg->currents_A[i];
  //    }

  //    ROS_DEBUG("received following currents message:\n%s", ss.str().c_str());
}

void CurrentsPanel::resetCurrentsLayout() {
  assert(coil_labels_.size() == current_values_.size());

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
    QLabel* current_val = new QLabel(ss.str().c_str());
    current_val->setAlignment(Qt::AlignHCenter);
    currents_layout_->addWidget(current_val, 1, i);
    current_values_.push_back(current_val);
  }

  layout_->setAlignment(Qt::AlignTop);
  layout_->addLayout(currents_layout_);
}

}  // namespace mag_control

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mag_control::CurrentsPanel, rviz::Panel)
