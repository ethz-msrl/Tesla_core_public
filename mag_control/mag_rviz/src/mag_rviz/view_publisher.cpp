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

#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>

#include <rviz/frame_manager.h>
#include <rviz/geometry.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/vector_property.h>
#include <rviz/visualization_manager.h>
#include "rviz/default_plugin/view_controllers/orbit_view_controller.h"
#include "rviz/ogre_helpers/shape.h"

#include "mag_rviz/view_publisher.h"

#include <ros/ros.h>

// test
#include <OGRE/OgreCamera.h>
#include <rviz/tool.h>
#include <rviz/view_manager.h>
#include <rviz/viewport_mouse_event.h>
#include <exception>
// test over

static const float PITCH_START = Ogre::Math::HALF_PI / 2.0;
static const float YAW_START = Ogre::Math::HALF_PI * 0.5;
static const float DISTANCE_START = 10;

namespace mag_control {

void ViewPublisher::handleMouseEvent(rviz::ViewportMouseEvent& event) {
  try {
    rviz::OrbitViewController::handleMouseEvent(event);
    Ogre::Vector3 pos = camera_->getPosition();
    Ogre::Vector3 dir = camera_->getDirection();
    Ogre::Quaternion q = camera_->getOrientation();

    tf::Transform transform;
    transform.setOrigin(tf::Vector3(pos.x, pos.y, pos.z));
    transform.setRotation(tf::Quaternion(q.x, q.y, q.z, q.w));

    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "rviz_view"));
  } catch (std::exception& e) {
    ROS_INFO("failed to handle mouse event");
  }
}

void ViewPublisher::reset() {
  rviz::OrbitViewController::reset();
  ROS_INFO("reset");
}

void ViewPublisher::update(float dt, float ros_dt) {
  rviz::OrbitViewController::update(dt, ros_dt);
}

void ViewPublisher::onInitialize() {
  ROS_INFO("on initialize");
  rviz::OrbitViewController::onInitialize();

  view_ctrl_sub_ = nh_.subscribe<std_msgs::Float64MultiArray>("/ps4_controller/instructions", 1,
                                                              &ViewPublisher::viewCtrlCb, this);

  ROS_INFO("on initialize completed");
}

void ViewPublisher::viewCtrlCb(const std_msgs::Float64MultiArray::ConstPtr& msg) {
  rviz::OrbitViewController::pitch(msg->data[1]);
  rviz::OrbitViewController::yaw(msg->data[0]);
  rviz::OrbitViewController::zoom(msg->data[2]);

  Ogre::Vector3 new_orbit = Ogre::Vector3(msg->data[3], msg->data[4], msg->data[5]);
  rviz::OrbitViewController::focal_point_property_->setVector(new_orbit);
}

}  // namespace mag_control

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mag_control::ViewPublisher, rviz::ViewController)
