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
#include <rviz/ogre_helpers/shape.h>

#include "mag_rviz/dipolegradient_visual.h"

#include <ros/console.h>

namespace mag_control {

DipoleGradientVisual::DipoleGradientVisual(Ogre::SceneManager* scene_manager,
                                           Ogre::SceneNode* parent_node) {
  _scene_manager = scene_manager;
  _frame_node = parent_node->createChildSceneNode();

  _magfield_arrow.reset(new rviz::Arrow(scene_manager, _frame_node, 0.8f, 0.1f, 0.2f, 0.2f));
  _magfield_arrow->setPosition(Ogre::Vector3::ZERO);
  _magfield_arrow->setColor(Ogre::ColourValue::White);
  _magfield_arrow->setScale(Ogre::Vector3::ZERO);

  _gradient_arrow.reset(new rviz::Arrow(scene_manager, _frame_node, 0.8f, 0.1f, 0.2f, 0.2f));
  _gradient_arrow->setPosition(Ogre::Vector3::ZERO);
  _gradient_arrow->setColor(Ogre::ColourValue::White);
  _gradient_arrow->setScale(Ogre::Vector3::ZERO);

  _scale = 1.f;
  _gradient_scale = 1.f;
}

DipoleGradientVisual::~DipoleGradientVisual() { _scene_manager->destroySceneNode(_frame_node); }

void DipoleGradientVisual::setMessage(const mag_msgs::DipoleGradientStampedConstPtr& msg) {
  _field_msg = msg;

  Ogre::Vector3 field(msg->field.x, msg->field.y, msg->field.z);
  Ogre::Vector3 gradient(msg->gradient.x, msg->gradient.y, msg->gradient.z);
  Ogre::Vector3 field_position(msg->position.x, msg->position.y, msg->position.z);

  float field_mag = field.length();

  _magfield_arrow->setPosition(field_position);
  _magfield_arrow->setDirection(field);
  _magfield_arrow->setScale(
      Ogre::Vector3(_scale * field_mag, _scale * field_mag, _scale * field_mag));
  _magfield_arrow->setColor(_color);

  float gradient_mag = gradient.length();
  _gradient_arrow->setPosition(field_position);
  _gradient_arrow->setDirection(gradient);
  _gradient_arrow->setScale(Ogre::Vector3(_gradient_scale * gradient_mag,
                                          _gradient_scale * gradient_mag,
                                          _gradient_scale * gradient_mag));
  _gradient_arrow->setColor(_gradient_color);
}

void DipoleGradientVisual::setFramePosition(const Ogre::Vector3& position) {
  _frame_node->setPosition(position);
}

void DipoleGradientVisual::setFrameOrientation(const Ogre::Quaternion& orientation) {
  _frame_node->setOrientation(orientation);
}

void DipoleGradientVisual::setColor(float r, float g, float b, float a) {
  _color = Ogre::ColourValue(r, g, b, a);
  _magfield_arrow->setColor(r, g, b, a);
}

void DipoleGradientVisual::setGradientColor(float r, float g, float b, float a) {
  _gradient_color = Ogre::ColourValue(r, g, b, a);
  _gradient_arrow->setColor(r, g, b, a);
}

void DipoleGradientVisual::setPosition(const Ogre::Vector3& position) { _position = position; }

void DipoleGradientVisual::setScale(float s) {
  _scale = s;

  if (_field_msg != NULL) {
    Ogre::Vector3 field(_field_msg->field.x, _field_msg->field.y, _field_msg->field.z);
    float field_mag = field.length();
    _magfield_arrow->setScale(
        Ogre::Vector3(_scale * field_mag, _scale * field_mag, _scale * field_mag));
  }
}

void DipoleGradientVisual::setGradientScale(float s) {
  _gradient_scale = s;

  if (_field_msg != NULL) {
    Ogre::Vector3 gradient(_field_msg->gradient.x, _field_msg->gradient.y, _field_msg->gradient.z);
    float gradient_mag = gradient.length();
    _gradient_arrow->setScale(Ogre::Vector3(_gradient_scale * gradient_mag,
                                            _gradient_scale * gradient_mag,
                                            _gradient_scale * gradient_mag));
  }
}
}  // namespace mag_control
