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
#include <rviz/properties/bool_property.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/vector_property.h>
#include <rviz/visualization_manager.h>

#include "mag_rviz/magfield_display.h"
#include "mag_rviz/magfield_visual.h"

namespace mag_control {

MagFieldDisplay::MagFieldDisplay() {
  _p_color_property =
      new rviz::ColorProperty("Color", QColor(255, 0, 0), "Color to draw the magnetic field arrow",
                              this, SLOT(updateProperties()));
  _p_alpha_property = new rviz::FloatProperty(
      "Alpha", 1.0, "Transparency of the magnetic field arrow", this, SLOT(updateProperties()));

  _p_scale_property = new rviz::FloatProperty(
      "Scale", 1.f, "Apply scaling parameter to the field arrow", this, SLOT(updateProperties()));

  _p_scale_property->setMin(0.f);
  _p_scale_property->setMax(1e6);
}

MagFieldDisplay::~MagFieldDisplay() {}

void MagFieldDisplay::onInitialize() {
  MFDClass::onInitialize();
  _p_visual.reset(new MagFieldVisual(context_->getSceneManager(), scene_node_));
  updateProperties();
}

void MagFieldDisplay::reset() { MFDClass::reset(); }

void MagFieldDisplay::updateProperties() {
  float alpha = _p_alpha_property->getFloat();
  Ogre::ColourValue color = _p_color_property->getOgreColor();

  _p_visual->setColor(color.r, color.g, color.b, alpha);
  _p_visual->setScale(_p_scale_property->getFloat());
}

void MagFieldDisplay::processMessage(const mag_msgs::FieldStampedConstPtr &msg) {
  Ogre::Quaternion orientation;
  Ogre::Vector3 position;
  if (!context_->getFrameManager()->getTransform(msg->header.frame_id, msg->header.stamp, position,
                                                 orientation)) {
    ROS_DEBUG("Error transforming from frame '%s' to frame '%s'", msg->header.frame_id.c_str(),
              qPrintable(fixed_frame_));
    return;
  }

  _p_visual->setMessage(msg);
  _p_visual->setFramePosition(position);
  _p_visual->setFrameOrientation(orientation);
}

}  // namespace mag_control

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mag_control::MagFieldDisplay, rviz::Display)
