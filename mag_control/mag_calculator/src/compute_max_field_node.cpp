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

#include <string>

#include <ros/package.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <tf/transform_listener.h>
#include "mag_calculator/compute_max_field_node.h"
#include "mag_manip/backward_model_factory.h"
#include "mag_manip/emns.h"
#include "mag_manip/emns_parameters_yaml.h"
#include "mag_manip/types.h"
#include "mag_msgs/FieldStamped.h"

using namespace mag_calculator;
using namespace mag_manip;
using namespace std;

ComputeMaxFieldNode::ComputeMaxFieldNode(ros::NodeHandle& nh, EMNSParameters::Ptr p_emns_params,
                                         BackwardModel::Ptr p_model)
    : nh_(nh), p_emns_params_(p_emns_params), p_model_(p_model) {
  sub_mag_field_ = nh.subscribe("/backward_model/field", 1, &ComputeMaxFieldNode::fieldSub, this);

  pub_max_field_mag_ = nh.advertise<std_msgs::Float32>("/max_field", 1);
}

void ComputeMaxFieldNode::fieldSub(const mag_msgs::FieldStampedConstPtr msg) {
  ROS_DEBUG_STREAM("Got a field message " << msg->field);
  // Transform to mns frame
  geometry_msgs::PointStamped position_out;
  geometry_msgs::Vector3Stamped vec_out;
  try {
    listener_.waitForTransform("mns", msg->header.frame_id, msg->header.stamp, ros::Duration(3.0));
    geometry_msgs::PointStamped stamped_in;
    stamped_in.header = msg->header;
    stamped_in.point = msg->field.position;
    listener_.transformPoint("mns", stamped_in, position_out);

    geometry_msgs::Vector3Stamped vec_in;
    vec_in.header = msg->header;
    vec_in.vector = msg->field.vector;
    listener_.transformVector("mns", vec_in, vec_out);
    ROS_DEBUG_STREAM("Done calculating transform" << endl
                                                  << "mag_field (before) " << vec_in << endl
                                                  << "mag_field (after) " << vec_out);
  } catch (tf::TransformException& ex) {
    ROS_ERROR("%s", ex.what());
  }

  PositionVec new_position(position_out.point.x, position_out.point.y, position_out.point.z);
  FieldVec mag_field(vec_out.vector.x, vec_out.vector.y, vec_out.vector.z);

  double max_field_mag = getMaxFieldMagnitudeAlignedWithTargetField(
      *p_model_, new_position, mag_field, p_emns_params_->getMaxCurrents(),
      p_emns_params_->getCoilResistances(), p_emns_params_->getMaxPower());

  ROS_DEBUG_STREAM("Publishing max_field_mag " << max_field_mag);

  std_msgs::Float32 msg_out;
  msg_out.data = max_field_mag;
  pub_max_field_mag_.publish(msg_out);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "compute_max_field");

  ROS_INFO("Started compute_max_field node");
  auto nh = ros::NodeHandle();

  // This demonstrates how you can create the eMNS settings if they are defined in a YAML file
  // The resulting parameters interface can then be passed on
  string emns_params_path = ros::package::getPath("mag_manip") + "/params/Navion_2.yaml";
  auto p_emns_params = EMNSParametersYAML::ptrFromFile(emns_params_path);

  // Here we create the backward model as usual
  string model_path = ros::package::getPath("mpem") + "/cal/Navion_2_Calibration_24-02-2020.yaml";
  BackwardModelFactory f;
  auto p_model = f.create("mpem_L2", model_path);

  // The node takes the params and injects the EMNSParameters dependency
  ComputeMaxFieldNode node(nh, p_emns_params, p_model);

  ros::spin();

  return 0;
}
