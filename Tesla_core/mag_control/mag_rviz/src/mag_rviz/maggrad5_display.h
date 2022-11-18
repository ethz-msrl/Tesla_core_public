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
#include <mag_msgs/Gradient3Stamped.h>
#include <mag_msgs/Gradient5Stamped.h>
#include <rviz/message_filter_display.h>
#endif

namespace Ogre {
class SceneNode;
}

namespace rviz {
class ColorProperty;
class FloatProperty;
class IntProperty;
class VectorProperty;
class BoolProperty;
}  // namespace rviz

namespace mag_control {

class MagGrad3Visual;

class MagGrad5Display : public rviz::MessageFilterDisplay<mag_msgs::Gradient5Stamped> {
  Q_OBJECT

 public:
  MagGrad5Display();
  virtual ~MagGrad5Display();

 protected:
  virtual void onInitialize();

  virtual void reset();

 private slots:
  void updateProperties();

 private:
  void processMessage(const mag_msgs::Gradient5Stamped::ConstPtr& msg);

  rviz::ColorProperty* _p_color_property;
  rviz::FloatProperty* _p_alpha_property;
  rviz::VectorProperty* _p_position_property;
  rviz::VectorProperty* _p_align_vector_property;
  rviz::FloatProperty* _p_scale_property;
  rviz::BoolProperty* _p_use_gradient_position_property;

  boost::shared_ptr<MagGrad3Visual> _p_visual;

  boost::shared_ptr<mag_msgs::Gradient3Stamped> _p_grad3_msg;
};
}  // namespace mag_control
