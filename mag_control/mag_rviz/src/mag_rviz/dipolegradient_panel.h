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

#ifndef Q_MOC_RUN
#include <geometry_msgs/PointStamped.h>
#include <mag_msgs/DipoleGradientStamped.h>
#include <ros/ros.h>
#include <rosgraph_msgs/Log.h>
#include <rviz/panel.h>
#endif

class QLineEdit;
class QLabel;
class QVBoxLayout;
class QGridLayout;
class QSlider;
class QSpinBox;
class QCheckBox;
class QTimer;
class QDoubleSpinBox;
class QPushButton;

namespace mag_control {

class DipoleGradientPanel : public rviz::Panel {
  Q_OBJECT

 public:
  explicit DipoleGradientPanel(QWidget* parent = 0);

  virtual void load(const rviz::Config& config);
  virtual void save(rviz::Config config) const;

  void updateDipoleGradient();

 public slots:

  void setTopic(const QString& topic_name);

  // related to position
  void setPosTopic(const QString& topic_name);

 protected slots:

  void updateTopic();
  void updateDipoleGradient(int value);
  void updateDipoleGradient(double value);
  void updateDipoleGradientManual();
  void processCheckBox(int value);

  // related to position
  void updatePosTopic();
  void updatePosition(const geometry_msgs::PointStamped::ConstPtr& msg);
  void processPosCheckBox(int value);

  void processUpdateCheckBox(int value);

  void LogCb(const rosgraph_msgs::Log::ConstPtr& msg);

 protected:
  QLineEdit* topic_edit_;
  QLineEdit* pos_topic_edit_;
  QLineEdit* frame_edit_;
  QString magfield_topic_;
  QString position_topic_;
  QSpinBox* position_x_sb_;
  QSpinBox* position_y_sb_;
  QSpinBox* position_z_sb_;
  QDoubleSpinBox* gradient_x_sb_;
  QDoubleSpinBox* gradient_y_sb_;
  QDoubleSpinBox* gradient_z_sb_;

  QSpinBox* field_magnitude_sb_;
  QSpinBox* field_inclination_sb_;
  QSpinBox* field_azimuth_sb_;

  QDoubleSpinBox* gradient_magnitude_sb_;
  QSpinBox* gradient_inclination_sb_;
  QSpinBox* gradient_azimuth_sb_;

  // QSpinBox* dipole_magnitude_sb_;
  QSpinBox* dipole_inclination_sb_;
  QSpinBox* dipole_azimuth_sb_;
  QCheckBox* position_feedback_checkbox_;

  QCheckBox* aligned_dipole_checkbox_;
  QCheckBox* auto_update_checkbox_;

  QPushButton* send_button_;

  QLabel* log_msgs_;

  ros::Publisher mag_field_pub_;
  ros::Publisher mag_field_pub_unarmed_;
  // position subscriber
  ros::Subscriber pos_sub_;
  ros::Subscriber log_subscriber_;

  QVBoxLayout* layout_;

  ros::NodeHandle nh_;
};
}  // namespace mag_control
