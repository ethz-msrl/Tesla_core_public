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

#include "mag_rviz/tracking_update.h"

#include <QDoubleValidator>
#include <QFont>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QVBoxLayout>

#include "std_srvs/Trigger.h"

namespace mag_control {

TrackingUpdate::TrackingUpdate(QWidget* parent)
    : rviz::Panel(parent), service_topic_("/intersection_selector/update_position") {
  layout_ = new QVBoxLayout;
  grid_layout_ = new QGridLayout;

  topic_edit_ = new QLineEdit(service_topic_);
  connect(topic_edit_, SIGNAL(editingFinished()), this, SLOT(updateTopic()));
  send_button_ = new QPushButton("Update");
  connect(send_button_, SIGNAL(released()), this, SLOT(sendServiceRequest()));

  grid_layout_->addWidget(topic_edit_, 0, 0);
  grid_layout_->addWidget(send_button_, 1, 0);

  layout_->addLayout(grid_layout_);

  layout_->setAlignment(Qt::AlignTop);
  setLayout(layout_);

  service_ = nh_.serviceClient<std_srvs::Trigger>(service_topic_.toStdString());
}

void TrackingUpdate::load(const rviz::Config& config) {
  rviz::Panel::load(config);
  QString service_topic;

  if (config.mapGetString("service_topic", &service_topic)) {
    topic_edit_->setText(service_topic);
    updateTopic();
  }
}

void TrackingUpdate::save(rviz::Config config) const {
  rviz::Panel::save(config);
  config.mapSetValue("service_topic", service_topic_);
}

void TrackingUpdate::setTopic(const QString& topic_name) {
  if (topic_name == "") {
    ROS_DEBUG("Empty topic");
    return;
  }

  if (topic_name == service_topic_) {
    ROS_DEBUG("Already subscribed to topic");
  }

  service_topic_ = topic_name;
  service_.shutdown();
  service_ = nh_.serviceClient<std_srvs::Trigger>(service_topic_.toStdString());

  std::cout << service_topic_.toStdString() << std::endl;

  Q_EMIT configChanged();
}

void TrackingUpdate::updateTopic() { setTopic(topic_edit_->text()); }

void TrackingUpdate::sendServiceRequest() {
  std_srvs::Trigger srv;

  if (service_.exists()) {
    if (service_.call(srv)) {
      ROS_INFO("Service called");
    } else {
      ROS_ERROR("Failed to call service");
    }
  } else {
    ROS_ERROR("Service does not exist");
  }
}

}  // namespace mag_control

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mag_control::TrackingUpdate, rviz::Panel)
