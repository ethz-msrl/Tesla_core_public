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

#include "mag_calculator/backward_model.h"

#include <tf_conversions/tf_eigen.h>

#include <Eigen/Geometry>
#include <Eigen/SVD>

#include "mag_manip/backward_model_factory.h"
#include "mag_manip/helpers.h"
#include "mag_msgs/CurrentsStamped.h"

using namespace mag_manip;

mag_calculator::BackwardModel::BackwardModel(ros::NodeHandle& nh, ros::NodeHandle& nh_private)
    : nh_(nh), nh_private_(nh_private), listener_() {
  // read from parameter server
  nh_private.param<std::string>("calibration_path", cal_path_, "");
  nh_private.param<std::string>("calibration_type", cal_type_, "mpem_L2");
  nh_private.param<double>("max_field_intensity", max_field_intensity_T_, 0.04);
  nh_private.param<double>("max_gradient_intensity", max_gradient_intensity_T_m_, 0.5);
  nh_private.param<bool>("is_field_inverted", is_field_inverted_, false);
  nh_private.param<bool>("send_ecb_block_msg", send_ecb_block_msg_, false);
  nh_private.param<bool>("ecb_direct", ecb_direct_, false);

  // services
  srv_load_cal_file_ = nh_private.advertiseService("load_calibration_file",
                                                   &BackwardModel::loadCalibrationSrv, this);

  // subscribers
  sub_mag_field_ = nh_private.subscribe("field", 1, &BackwardModel::fieldSub, this);
  sub_dipole_gradient_ =
      nh_private.subscribe("dipole_gradient", 1, &BackwardModel::dipoleGradientSub, this);
  sub_field_gradient5_ =
      nh_private.subscribe("field_gradient5", 1, &BackwardModel::fieldGradient5Sub, this);

  // publishers
  if (send_ecb_block_msg_) {
    // We publish the ECB currents on a global topic so that it can be read by the ECB controller
    // node
    pub_currents_block_ =
        nh.advertise<ecb_msgs::CurrentsBlockStamped>("/ECB/desired_currents_block", 1);
  }
  pub_currents_ = nh_private.advertise<mag_msgs::CurrentsStamped>("currents", 1);

  BackwardModelFactory f;

  try {
    p_backward_model_ = f.create(cal_type_, cal_path_);

    if (!p_backward_model_->isValid()) {
      ROS_FATAL_STREAM("Model defined by " << cal_path_ << " is not valid.");
    }

    num_coils_ = p_backward_model_->getNumCoils();
  } catch (std::exception& e) {
    ROS_FATAL_STREAM("Unable to load model: " << e.what());
    return;
  }
}

bool mag_calculator::BackwardModel::loadCalibrationSrv(
    mag_calculator::load_calibration_file::Request& req,
    mag_calculator::load_calibration_file::Response& resp) {
  cal_path_ = req.filepath;

  p_backward_model_->setCalibrationFile(cal_path_);
  bool is_valid = p_backward_model_->isValid();

  if (!is_valid) {
    ROS_ERROR_STREAM("Model defined by " << cal_path_ << " is not valid.");
  }

  num_coils_ = p_backward_model_->getNumCoils();

  return is_valid;
}

void mag_calculator::BackwardModel::sendCurrents(const CurrentsVec& currents) {
  mag_msgs::CurrentsStamped desired_currents_msg;
  desired_currents_msg.header.frame_id = "NA";
  desired_currents_msg.header.stamp = ros::Time::now();
  std::vector<double> currents_v(currents.data(),
                                 currents.data() + currents.rows() * currents.cols());
  desired_currents_msg.currents = currents_v;
  pub_currents_.publish(desired_currents_msg);
  ROS_DEBUG("Bckwards model: send desired currents");

  /* Because the ECB takes block of 40 current values, we will send
   * a block containing 40 copies of our 8x1 current vector */
  ecb_msgs::CurrentsBlockStamped currents_block_msg;
  currents_block_msg.header.frame_id = "ecb";
  currents_block_msg.header.stamp = ros::Time::now();

  for (int j = 0; j < 40; j++) {
    for (int c = 0; c < currents.rows(); c++) {
      currents_block_msg.block.currents_mA[c + j * currents.rows()] = 1000 * currents(c);
    }
  }

  currents_block_msg.block.direct = ecb_direct_;

  if (send_ecb_block_msg_) pub_currents_block_.publish(currents_block_msg);
}

void mag_calculator::BackwardModel::fieldSub(const mag_msgs::FieldStamped::ConstPtr& msg) {
  ROS_DEBUG_STREAM("fieldSub");

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
  } catch (tf::TransformException& ex) {
    ROS_ERROR("%s", ex.what());
    return;
  }

  Eigen::Vector3d new_position(position_out.point.x, position_out.point.y, position_out.point.z);
  Eigen::Vector3d mag_field(vec_out.vector.x, vec_out.vector.y, vec_out.vector.z);

  if (is_field_inverted_) mag_field *= -1;

  if (mag_field.norm() > max_field_intensity_T_) {
    ROS_WARN(
        "Desired magnetic field intensity %2.f mT exceeds the maximum allowed intensity %2.f mT. "
        "Scaling the field down.",
        1000 * mag_field.norm(), 1000 * max_field_intensity_T_);

    double scale_f = max_field_intensity_T_ / mag_field.norm();
    mag_field *= scale_f;
  }

  CurrentsVec currents;
  try {
    currents = p_backward_model_->computeCurrentsFromField(new_position, mag_field);
  } catch (std::exception& e) {
    ROS_ERROR_STREAM("Unable to compute currents because: " << e.what());
    return;
  }

  sendCurrents(currents);
}

void mag_calculator::BackwardModel::fieldGradient5Sub(
    const mag_msgs::FieldGradient5Stamped::ConstPtr& msg) {
  ROS_DEBUG_STREAM("fieldGradient5Sub");

  geometry_msgs::PointStamped position_out;
  geometry_msgs::Vector3Stamped field_out;
  Gradient5Vec gradient5_rotated;

  try {
    listener_.waitForTransform("mns", msg->header.frame_id, msg->header.stamp, ros::Duration(3.0));
    geometry_msgs::PointStamped stamped_in;
    stamped_in.header = msg->header;
    stamped_in.point = msg->position;
    listener_.transformPoint("mns", stamped_in, position_out);

    tf::StampedTransform T_mns_f;
    listener_.lookupTransform("mns", msg->header.frame_id, ros::Time(0), T_mns_f);
    Eigen::VectorXd gradient5(5);
    gradient5 << msg->vector.xx, msg->vector.xy, msg->vector.xz, msg->vector.yy, msg->vector.yz;
    GradientMat grad_mat = gradient5VecToGradientMat(gradient5);
    Eigen::Affine3d T_mns_f_e;
    tf::transformTFToEigen(T_mns_f, T_mns_f_e);
    Eigen::Matrix3d R_mns_f_e = T_mns_f_e.linear();
    GradientMat grad_mat_rotated = R_mns_f_e * grad_mat;
    gradient5_rotated = gradientMatToGradient5Vec(grad_mat_rotated);

    geometry_msgs::Vector3Stamped field_in;
    field_in.header = msg->header;
    field_in.vector = msg->field;
    listener_.transformVector("mns", field_in, field_out);
  } catch (tf::TransformException& ex) {
    ROS_ERROR("%s", ex.what());
    return;
  }

  Eigen::Vector3d new_position(position_out.point.x, position_out.point.y, position_out.point.z);
  Eigen::Vector3d mag_field(field_out.vector.x, field_out.vector.y, field_out.vector.z);

  if (mag_field.norm() > max_field_intensity_T_) {
    ROS_WARN(
        "Desired magnetic field intensity %2.f mT exceeds the maximum allowed intensity %2.f mT. "
        "Scaling the field down.",
        1000 * mag_field.norm(), 1000 * max_field_intensity_T_);

    double scale_f = mag_field.norm() / max_field_intensity_T_;
    mag_field *= scale_f;
  }

  if (gradient5_rotated.norm() > max_gradient_intensity_T_m_) {
    ROS_WARN(
        "Desired magnetic gradient intensity %2.f mT/m exceeds the maximum allowed intensity %2.f "
        "mT/m. Scaling the gradient down.",
        1000 * gradient5_rotated.norm(), 1000 * max_gradient_intensity_T_m_);
    double scale_f = gradient5_rotated.norm() / max_gradient_intensity_T_m_;
    gradient5_rotated *= scale_f;
  }

  CurrentsVec currents;
  try {
    currents = p_backward_model_->computeCurrentsFromFieldGradient5(new_position, mag_field,
                                                                    gradient5_rotated);
  } catch (std::exception& e) {
    ROS_ERROR_STREAM("Unable to compute currents because: " << e.what());
    return;
  }
  sendCurrents(currents);
}

void mag_calculator::BackwardModel::dipoleGradientSub(
    const mag_msgs::DipoleGradientStamped::ConstPtr& msg) {
  ROS_DEBUG_STREAM("dipoleGradientSub");

  // Transform to mns frame
  geometry_msgs::PointStamped position_out;
  geometry_msgs::Vector3Stamped vec_out;
  geometry_msgs::Vector3Stamped gradient_out;
  geometry_msgs::Vector3Stamped dipole_out;
  try {
    listener_.waitForTransform("mns", msg->header.frame_id, msg->header.stamp, ros::Duration(3.0));
    geometry_msgs::PointStamped stamped_in;
    stamped_in.header = msg->header;
    stamped_in.point = msg->position;
    listener_.transformPoint("mns", stamped_in, position_out);

    geometry_msgs::Vector3Stamped vec_in;
    vec_in.header = msg->header;
    vec_in.vector = msg->field;
    listener_.transformVector("mns", vec_in, vec_out);

    geometry_msgs::Vector3Stamped gradient_in;
    gradient_in.header = msg->header;
    gradient_in.vector = msg->gradient;
    listener_.transformVector("mns", gradient_in, gradient_out);

    geometry_msgs::Vector3Stamped dipole_in;
    dipole_in.header = msg->header;
    dipole_in.vector = msg->dipole;
    listener_.transformVector("mns", dipole_in, dipole_out);
  } catch (tf::TransformException& ex) {
    ROS_ERROR("%s", ex.what());
    return;
  }

  Eigen::Vector3d new_position(position_out.point.x, position_out.point.y, position_out.point.z);
  Eigen::Vector3d mag_field(vec_out.vector.x, vec_out.vector.y, vec_out.vector.z);
  Eigen::Vector3d gradient(gradient_out.vector.x, gradient_out.vector.y, gradient_out.vector.z);
  Eigen::Vector3d dipole(dipole_out.vector.x, dipole_out.vector.y, dipole_out.vector.z);

  if (is_field_inverted_) {
    ROS_WARN("Inverted magnetic field is not supported for dipole gradient");
    mag_field *= -1;
  }

  if (mag_field.norm() > max_field_intensity_T_) {
    ROS_WARN(
        "Desired magnetic field intensity %2.f mT exceeds the maximum allowed intensity %2.f mT. "
        "Scaling the field down.",
        1000 * mag_field.norm(), 1000 * max_field_intensity_T_);

    double scale_f = max_field_intensity_T_ / mag_field.norm();
    mag_field *= scale_f;
  }

  if (mag_field.norm() < 1e-12 && gradient.norm() < 1e-12) {
    ROS_WARN("Both desired magnetic field and gradients are near zero. Outputting zero currents");
    sendCurrents(CurrentsVec::Zero(num_coils_));
  } else if (mag_field.norm() < 1e-12 && dipole.norm() < 1e-12) {
    ROS_WARN("Both desired magnetic field and dipole are near zero. Outputting zero currents");
    sendCurrents(CurrentsVec::Zero(num_coils_));
  } else {
    dipole /= dipole.norm();
    CurrentsVec currents;
    try {
      currents = p_backward_model_->computeCurrentsFromFieldDipoleGradient3(new_position, mag_field,
                                                                            dipole, gradient);
    } catch (std::exception& e) {
      ROS_ERROR_STREAM("Unable to compute currents because: " << e.what());
      return;
    }
    sendCurrents(currents);
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "backward_model");

  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");

  ROS_INFO("Starting backward model");

  mag_calculator::BackwardModel node(nh, nh_private);

  ros::spin();

  ROS_INFO("Terminating backward model");
  return 0;
}
