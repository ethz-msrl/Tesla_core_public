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
#include "mag_rviz/currents_visual.h"

#include <rviz/ogre_helpers/movable_text.h>

#include <ros/console.h>

namespace mag_control {

CurrentsVisual::CurrentsVisual(Ogre::SceneManager *scene_manager, Ogre::SceneNode *parent_node)
    : num_decimals_(2), text_size_(0.3) {
  _scene_manager = scene_manager;
  _frame_node = parent_node->createChildSceneNode();

  _p_movable_text = new rviz::MovableText("Currents", "Arial", text_size_);
  _p_movable_text->setColor(Ogre::ColourValue::White);
  _p_movable_text->setVisible(false);
  _p_movable_text->setTextAlignment(rviz::MovableText::H_CENTER, rviz::MovableText::V_CENTER);

  Ogre::Vector3 text_position(5., 5., 5.);
  _p_movable_text->setGlobalTranslation(text_position);
  parent_node->attachObject(_p_movable_text);
}

CurrentsVisual::~CurrentsVisual() { _scene_manager->destroySceneNode(_frame_node); }

void CurrentsVisual::setMessage(const mag_msgs::CurrentsStampedConstPtr &msg) {
  _p_movable_text->setVisible(true);

  std::ostringstream ss;

  //    for(int i = 0; i < msg->num; i++) {
  //        ss << std::setw(6) << std::left << i;
  //    }

  //    ss << std::endl;

  for (int i = 0; i < msg->currents.size(); i++) {
    ss << std::setw(6) << std::left << std::fixed << std::setprecision(num_decimals_)
       << msg->currents[i];
  }

  ss << std::endl;

  ROS_DEBUG("Displaying currents \n%s", ss.str().c_str());

  _p_movable_text->setCaption(ss.str());
}

void CurrentsVisual::setColor(const Ogre::ColourValue &color) { _p_movable_text->setColor(color); }

void CurrentsVisual::setPosition(const Ogre::Vector3 &position) {
  _p_movable_text->setGlobalTranslation(position);
}

void CurrentsVisual::setTextSize(float s) { _p_movable_text->setCharacterHeight(s); }

}  // namespace mag_control
