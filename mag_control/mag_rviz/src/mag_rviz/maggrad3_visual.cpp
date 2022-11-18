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
#include <OGRE/OgreVector3.h>

#include <rviz/ogre_helpers/arrow.h>

#include "mag_rviz/maggrad3_visual.h"

#include <ros/console.h>

namespace mag_control {

MagGrad3Visual::MagGrad3Visual(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node) {
  _scene_manager = scene_manager;
  _frame_node = parent_node->createChildSceneNode();

  _gradient3_arrow.reset(new rviz::Arrow(scene_manager, _frame_node));

  _gradient3_arrow->setPosition(Ogre::Vector3::ZERO);
  _gradient3_arrow->setColor(Ogre::ColourValue::Green);
  _gradient3_arrow->setScale(Ogre::Vector3::ZERO);

  _scale = 1.f;
  _display_at_gradient_position = false;
}

MagGrad3Visual::~MagGrad3Visual() { _scene_manager->destroySceneNode(_frame_node); }

void MagGrad3Visual::setMessage(const mag_msgs::Gradient3StampedConstPtr& msg) {
  Ogre::Vector3 gradient(msg->gradient.x, msg->gradient.y, msg->gradient.z);

  float gradient_mag = gradient.length();

  Ogre::Vector3 gradient_position(msg->position.x, msg->position.y, msg->position.z);

  _gradient3_arrow->setScale(Ogre::Vector3(_scale, _scale, _scale));
  _gradient3_arrow->setDirection(gradient);

  if (_display_at_gradient_position) {
    _gradient3_arrow->setPosition(gradient_position);
  } else {
    _gradient3_arrow->setPosition(_position);
  }
}

void MagGrad3Visual::setFramePosition(const Ogre::Vector3& position) {
  _frame_node->setPosition(position);
}

void MagGrad3Visual::setFrameOrientation(const Ogre::Quaternion& orientation) {
  _frame_node->setOrientation(orientation);
}

void MagGrad3Visual::setColor(float r, float g, float b, float a) {
  _gradient3_arrow->setColor(r, g, b, a);
}

void MagGrad3Visual::setPosition(const Ogre::Vector3& position) { _position = position; }

void MagGrad3Visual::setScale(float s) { _scale = s; }

void MagGrad3Visual::setDisplayAtGradientPosition(bool display) {
  _display_at_gradient_position = display;
}
}  // namespace mag_control
