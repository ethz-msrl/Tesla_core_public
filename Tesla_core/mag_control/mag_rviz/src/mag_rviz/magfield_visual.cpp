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

#include "mag_rviz/magfield_visual.h"

#include <ros/console.h>

namespace mag_control {

MagFieldVisual::MagFieldVisual(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node) {
  _scene_manager = scene_manager;
  _frame_node = parent_node->createChildSceneNode();

  _magfield_arrow.reset(new rviz::Arrow(scene_manager, _frame_node, 0.8f, 0.1f, 0.2f, 0.2f));

  _magfield_arrow->setPosition(Ogre::Vector3::ZERO);
  _magfield_arrow->setColor(Ogre::ColourValue::White);
  _magfield_arrow->setScale(Ogre::Vector3::ZERO);

  _scale = 1.f;
  _display_at_field_position = false;
}

MagFieldVisual::~MagFieldVisual() { _scene_manager->destroySceneNode(_frame_node); }

void MagFieldVisual::setMessage(const mag_msgs::FieldStampedConstPtr& msg) {
  _field_msg = msg;

  Ogre::Vector3 field(msg->field.vector.x, msg->field.vector.y, msg->field.vector.z);
  Ogre::Vector3 field_position(msg->field.position.x, msg->field.position.y, msg->field.position.z);

  float field_mag = field.length();

  _magfield_arrow->setPosition(field_position);
  _magfield_arrow->setDirection(field);
  _magfield_arrow->setScale(
      Ogre::Vector3(_scale * field_mag, _scale * field_mag, _scale * field_mag));
  _magfield_arrow->setColor(_color);
}

void MagFieldVisual::setFramePosition(const Ogre::Vector3& position) {
  _frame_node->setPosition(position);
}

void MagFieldVisual::setFrameOrientation(const Ogre::Quaternion& orientation) {
  _frame_node->setOrientation(orientation);
}

void MagFieldVisual::setColor(float r, float g, float b, float a) {
  _color = Ogre::ColourValue(r, g, b, a);
  _magfield_arrow->setColor(r, g, b, a);
}

void MagFieldVisual::setPosition(const Ogre::Vector3& position) { _position = position; }

void MagFieldVisual::setScale(float s) {
  _scale = s;

  if (_field_msg != NULL) {
    Ogre::Vector3 field(_field_msg->field.vector.x, _field_msg->field.vector.y,
                        _field_msg->field.vector.z);
    float field_mag = field.length();
    _magfield_arrow->setScale(
        Ogre::Vector3(_scale * field_mag, _scale * field_mag, _scale * field_mag));
  }
}

}  // namespace mag_control
