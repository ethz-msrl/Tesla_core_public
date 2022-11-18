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

#include <OGRE/OgreVector3.h>
#include <mag_msgs/FieldArrayStamped.h>

#include <vector>

namespace rviz {
class Arrow;
class Shape;
}  // namespace rviz

namespace mag_control {
class MagFieldArrayVisual {
 public:
  MagFieldArrayVisual(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node);

  virtual ~MagFieldArrayVisual();

  void setMessage(const mag_msgs::FieldArrayStampedConstPtr& msg);

  void setFramePosition(const Ogre::Vector3& position);

  void setFrameOrientation(const Ogre::Quaternion& orientation);

  void setColor(float r, float g, float b, float a);

  void setScale(float s);

 private:
  std::vector<rviz::Arrow*> _magfield_arrows;
  mag_msgs::FieldArrayStampedConstPtr _fields_msg;

  Ogre::SceneNode* _frame_node;
  Ogre::SceneManager* _scene_manager;

  float _scale;
  Ogre::Vector3 _position;
  Ogre::ColourValue _color;
};
}  // namespace mag_control
