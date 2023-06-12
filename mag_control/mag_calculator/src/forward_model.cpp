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

#include <mag_manip/forward_model_factory.h>
#include <mag_manip/helpers.h>

#include "mag_calculator/forward_model.h"

#include "mag_msgs/CurrentsStamped.h"
#include "mag_msgs/FieldArrayStamped.h"

#include <dynamic_reconfigure/server.h>
#include <mag_calculator/ForwardModelConfig.h>

void mag_calculator::ForwardModel::dynCallback(mag_calculator::ForwardModelConfig& config,
                                               uint32_t level) {
  ROS_DEBUG("Reconfigure Request");

  poses_.header.frame_id = frame_id_;
  poses_.header.stamp = ros::Time::now();
  poses_.poses.clear();

  for (int iX = 0; iX < config.nx; iX++) {
    for (int iY = 0; iY < config.ny; iY++) {
      for (int iZ = 0; iZ < config.nz; iZ++) {
        geometry_msgs::Point point;
        double cX = config.spacing * (config.nx - 1) / 2.;
        double cY = config.spacing * (config.ny - 1) / 2.;
        double cZ = config.spacing * (config.nz - 1) / 2.;
        point.x = iX * config.spacing - cX;
        point.y = iY * config.spacing - cY;
        point.z = iZ * config.spacing - cZ;
        geometry_msgs::Pose pose;
        pose.position = point;
        pose.orientation = geometry_msgs::Quaternion();

        poses_.poses.push_back(pose);
      }
    }
  }

  publishMagneticFields();
}

mag_calculator::ForwardModel::ForwardModel(ros::NodeHandle& nh, ros::NodeHandle& nh_private)
    : nh_(nh), nh_private_(nh_private), mns_frame_id_("mns") {
  // Default pose at which to calculate field
  geometry_msgs::Pose pose;
  pose.position = geometry_msgs::Point();
  pose.orientation = geometry_msgs::Quaternion();
  poses_.poses.push_back(pose);
  poses_.header.frame_id = frame_id_;
  poses_.header.stamp = ros::Time::now();

  nh_private_.param<std::string>("frame_id", frame_id_, "mns");
  nh_private_.param<std::string>("calibration_type", cal_type_, "mpem");
  nh_private.param<std::string>("calibration_path", cal_path_, "");
  nh_private.param<bool>("is_field_inverted", is_field_inverted_, false);
  nh_private.param<bool>("publish_field_aligned_grad_array", publish_field_aligned_grad_array_,
                         true);
  nh_private.param<bool>("display_warning_zero_field", display_warning_zero_field_, true);

  mag_manip::ForwardModelFactory fact;

  try {
    p_forward_model_ = fact.create(cal_type_, cal_path_);

    if (!p_forward_model_->isValid()) {
      ROS_FATAL_STREAM("Model defined by " << cal_path_ << " is not valid.");
    }
  } catch (std::exception& e) {
    ROS_FATAL_STREAM("Unable to load model: " << e.what());
  }

  currents_msg_.header.frame_id = "NA";
  currents_msg_.header.stamp = ros::Time::now();
  num_coils_ = p_forward_model_->getNumCoils();
  std::vector<double> currents_msg_v(num_coils_, 0.);
  currents_msg_.currents = currents_msg_v;

  // services
  srv_load_cal_file_ =
      nh_private.advertiseService("load_calibration_file", &ForwardModel::loadCalibrationSrv, this);
  srv_get_field_ =
      nh_private.advertiseService("get_field", &ForwardModel::getMagneticFieldSrv, this);

  // subscribers
  sub_pose_ = nh_private.subscribe("poses", 1, &ForwardModel::poseArraySub, this);
  sub_currents_ = nh_private.subscribe("currents", 1, &ForwardModel::currentsSub, this);

  // publishers
  pub_field_array_ = nh_private.advertise<mag_msgs::FieldArrayStamped>("field_array", 1);

  // this is technically not a fieldarray but a Gradient3 array
  if (publish_field_aligned_grad_array_)
    pub_dipole_grad_array_ =
        nh_private.advertise<mag_msgs::FieldArrayStamped>("field_aligned_gradient_array", 1);

  auto fun = boost::bind(&ForwardModel::dynCallback, this, _1, _2);
  dyn_server_.setCallback(fun);
}

void mag_calculator::ForwardModel::currentsSub(const mag_msgs::CurrentsStamped& msg) {
  ROS_DEBUG("currentsSub");
  currents_msg_ = msg;
  publishMagneticFields();
}

void mag_calculator::ForwardModel::poseArraySub(const geometry_msgs::PoseArray& msg) {
  ROS_DEBUG("poseArraySub");
  poses_ = msg;
  frame_id_ = poses_.header.frame_id;
  publishMagneticFields();
}

void mag_calculator::ForwardModel::publishMagneticFields() {
  ROS_DEBUG_STREAM("Publish magnetic field");

  // transform from frame to mns
  tf::StampedTransform trans_mns_f;

  if (frame_id_ == mns_frame_id_) {
    trans_mns_f.setIdentity();
  } else {
    try {
      tf_listener_.waitForTransform(mns_frame_id_, frame_id_, ros::Time(0), ros::Duration(3.0));
      tf_listener_.lookupTransform(mns_frame_id_, frame_id_, ros::Time(0), trans_mns_f);
    } catch (tf::TransformException& e) {
      ROS_ERROR_STREAM("Unable to lookup transform between mns and " << frame_id_ << " because of "
                                                                     << e.what());
      return;
    }
  }

  mag_msgs::FieldArrayStamped mfA;

  mfA.header.frame_id = mns_frame_id_;

  mag_msgs::FieldArrayStamped grad_array;
  if (publish_field_aligned_grad_array_) {
    grad_array.header.frame_id = mns_frame_id_;
    grad_array.header.stamp = poses_.header.stamp;
  }

  // Eigen::Vector8d current_vector;
  if (currents_msg_.currents.size() != num_coils_) {
    ROS_ERROR(
        "Currents vector size does not match number of coils in "
        "calibration");
    return;
  }

  Eigen::VectorXd current_vector(num_coils_);
  for (unsigned int i = 0; i < num_coils_; i++) {
    current_vector(i) = currents_msg_.currents[i];
  }

  for (const auto pose : poses_.poses) {
    // Need to transform pose into the right frame
    tf::Pose pose_tf;
    tf::poseMsgToTF(pose, pose_tf);
    tf::Transform trans_p_mns = trans_mns_f * pose_tf;
    tf::Vector3 t_p_mns = trans_p_mns.getOrigin();

    Eigen::Vector3d position(t_p_mns.x(), t_p_mns.y(), t_p_mns.z());
    Eigen::Vector3d mf_e = p_forward_model_->computeFieldFromCurrents(position, current_vector);

    if (is_field_inverted_) mf_e *= -1;

    mag_msgs::Field mf;
    mf.vector.x = mf_e(0);
    mf.vector.y = mf_e(1);
    mf.vector.z = mf_e(2);
    mf.position.x = t_p_mns.x();
    mf.position.y = t_p_mns.y();
    mf.position.z = t_p_mns.z();
    mfA.fields.push_back(mf);

    if (publish_field_aligned_grad_array_) {
      mag_msgs::Field grad3;

      if (mf_e.norm() < 1e-6) {
        ROS_WARN_STREAM_COND(display_warning_zero_field_,
                             "The field is near zero at one or more positions");
        grad3.vector.x = 0;
        grad3.vector.y = 0;
        grad3.vector.z = 0;
      } else {
        mag_manip::Gradient5Vec gradient =
            p_forward_model_->computeGradient5FromCurrents(position, current_vector);
        mag_manip::Gradient3Vec grad3_e = mag_manip::alignedGradientFromGrad5(mf_e, gradient);
        grad3.vector.x = grad3_e(0);
        grad3.vector.y = grad3_e(1);
        grad3.vector.z = grad3_e(2);
      }

      grad3.position.x = t_p_mns.x();
      grad3.position.y = t_p_mns.y();
      grad3.position.z = t_p_mns.z();
      grad_array.fields.push_back(grad3);
    }
  }
  mfA.header.stamp = ros::Time::now();
  pub_field_array_.publish(mfA);

  if (publish_field_aligned_grad_array_) {
    pub_dipole_grad_array_.publish(grad_array);
  }
}

bool mag_calculator::ForwardModel::loadCalibrationSrv(
    mag_calculator::load_calibration_file::Request& req,
    mag_calculator::load_calibration_file::Response& resp) {
  cal_path_ = req.filepath;

  try {
    p_forward_model_->setCalibrationFile(cal_path_);
  } catch (std::exception& e) {
    ROS_ERROR("Failed to load calibration");
    return false;
  }

  return true;
}

bool mag_calculator::ForwardModel::getMagneticFieldSrv(
    mag_calculator::get_magnetic_field::Request& req,
    mag_calculator::get_magnetic_field::Response& resp) {
  ROS_DEBUG_STREAM("Get magnetic field service");
  // Eigen::Vector8d current_vector;
  Eigen::VectorXd current_vector(num_coils_);
  for (unsigned int i = 0; i < num_coils_; i++) {
    current_vector(i) = req.currents[i];
  }

  Eigen::Vector3d position(req.position[0], req.position[1], req.position[2]);
  try {
    Eigen::Vector3d mf = p_forward_model_->computeFieldFromCurrents(position, current_vector);

    if (is_field_inverted_) mf *= -1;

    resp.mag_field[0] = mf(0);
    resp.mag_field[1] = mf(1);
    resp.mag_field[2] = mf(2);
  } catch (...) {
    ROS_ERROR("Failed to calculate field");
    return false;
  }

  return true;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "forward_model");

  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");

  ROS_INFO("Starting forward model");

  mag_calculator::ForwardModel node(nh, nh_private);

  ros::spin();

  ROS_INFO("Termining forward model");
  return 0;
}
