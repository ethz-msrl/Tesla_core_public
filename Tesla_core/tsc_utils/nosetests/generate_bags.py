# coding: utf-8
# Tesla - A ROS-based framework for performing magnetic manipulation

# Software License Agreement (BSD License)

# 2022 ETH Zurich, ​MAVT, Multi Scale Robotics Lab (MSRL) , Prof Bradley J. Nelson
# All rights reserved.

# Redistribution and use of this software in source and binary forms,
# with or without modification, are permitted provided that the following conditions are met:

# * Redistributions of source code must retain the above
#   copyright notice, this list of conditions and the
#   following disclaimer.

# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.

# * All advertising materials mentioning features or use of this software
#   must display the following acknowledgement:
#   “This product includes software developed by the Multi-Scale Robotics Lab,
#   ETH Zurich, Switzerland and its contributors.”

# * Neither the name of MSRL nor the names of its
#   contributors may be used to endorse or promote products
#   derived from this software without specific prior
#   written permission of MSRL.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
# THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
# IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
# PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
# OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
# STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
# OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import numpy as np
import rosbag
import rospy
from conversions import np_to_gradient5, np_to_point_msg, np_to_vector3_msg
from ecb_msgs.msg import CurrentsBlockStamped
from geometry_msgs.msg import (PointStamped, PoseStamped, QuaternionStamped,
                               TransformStamped, Vector3Stamped, WrenchStamped)
from mag_msgs.msg import (CurrentsStamped, Field, FieldArrayStamped,
                          FieldGradient3Stamped, FieldGradient5Stamped,
                          FieldStamped, Gradient3Stamped, Gradient5Stamped)
from mag_rod_msgs.msg import LengthStamped
from sensor_msgs.msg import CompressedImage, Image, JointState, Joy
from std_msgs.msg import Float32, Float32MultiArray

import cv2
from apriltag_ros.msg import AprilTagDetection, AprilTagDetectionArray

if __name__ == "__main__":

    bag = rosbag.Bag("test.bag", mode="w")

    for i in range(100):

        # we just test a single scalar std_msg type instead of all of them
        # since it would require creating a generic factory function to generate
        # all the different message types
        float_msg = Float32()
        float_msg.data = float(i)
        bag.write("/float32", float_msg)

        float_ma_msg = Float32MultiArray()
        float_ma_msg.data = [i] * 8
        bag.write("/float32multiarray", float_ma_msg)

        point_msg = PointStamped()
        point_msg.header.stamp = rospy.Time.from_sec(i)
        point_msg.point.x = float(i)
        bag.write("/point", point_msg)

        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.from_sec(i)
        pose_msg.pose.position.x = float(i)
        pose_msg.pose.orientation.x = float(i)
        bag.write("/pose", pose_msg)

        quat_msg = QuaternionStamped()
        quat_msg.header.stamp = rospy.Time.from_sec(i)
        quat_msg.quaternion.x = float(i)
        bag.write("/quaternion", quat_msg)

        tran_msg = TransformStamped()
        tran_msg.header.stamp = rospy.Time.from_sec(i)
        tran_msg.transform.translation.x = float(i)
        tran_msg.transform.rotation.x = float(i)
        bag.write("/transform", tran_msg)

        vec_msg = Vector3Stamped()
        vec_msg.header.stamp = rospy.Time.from_sec(i)
        vec_msg.vector.x = float(i)
        bag.write("/vector3", vec_msg)

        wren_msg = WrenchStamped()
        wren_msg.header.stamp = rospy.Time.from_sec(i)
        wren_msg.wrench.force.x = float(i)
        wren_msg.wrench.torque.x = float(i)
        bag.write("/wrench", wren_msg)

        joy_msg = Joy()
        joy_msg.header.stamp = rospy.Time.from_sec(i)
        joy_msg.axes = [float(i)]
        joy_msg.buttons = [i]
        bag.write("/joy", joy_msg)

        joint_state_msg = JointState()
        joint_state_msg.header.stamp = rospy.Time.from_sec(i)
        joint_state_msg.name = ["joint"]
        joint_state_msg.position = [float(i)]
        joint_state_msg.velocity = [float(i)]
        joint_state_msg.effort = [float(i)]
        bag.write("/joint_state", joint_state_msg)

        field_msg = FieldStamped()
        field_msg.header.stamp = rospy.Time.from_sec(i)
        field_msg.field.vector = np_to_vector3_msg([i, 0, 0])

        bag.write("/field", field_msg)

        currents_msg = CurrentsStamped()
        currents_msg.header.stamp = rospy.Time.from_sec(i)
        currents_msg.currents = [i] * 8
        bag.write("/currents", currents_msg)

        field_array_msg = FieldArrayStamped()
        field_array_msg.header.stamp = rospy.Time.from_sec(i)
        for j in range(10):
            field_msg = Field()
            field_msg.vector = np_to_point_msg([i, 0, 0])
            field_array_msg.fields.append(field_msg)
        bag.write("/field_array", field_array_msg)

        field_grad3_msg = FieldGradient3Stamped()
        field_grad3_msg.header.stamp = rospy.Time.from_sec(i)
        field_grad3_msg.field = np_to_vector3_msg([i, 0, 0])
        field_grad3_msg.vector = np_to_vector3_msg([i, 0, 0])
        bag.write("/field_gradient3", field_grad3_msg)

        field_grad5_msg = FieldGradient5Stamped()
        field_grad5_msg.header.stamp = rospy.Time.from_sec(i)
        field_grad5_msg.field = np_to_vector3_msg([i, 0, 0])
        field_grad5_msg.vector = np_to_gradient5([i, 0, 0, 0, 0])
        bag.write("/field_gradient5", field_grad5_msg)

        grad3_msg = Gradient3Stamped()
        grad3_msg.header.stamp = rospy.Time.from_sec(i)
        grad3_msg.gradient = np_to_vector3_msg([i, 0, 0])
        bag.write("/gradient3", grad3_msg)

        grad5_msg = Gradient5Stamped()
        grad5_msg.header.stamp = rospy.Time.from_sec(i)
        grad5_msg.gradient.vector = np_to_gradient5([i, 0, 0, 0, 0])
        bag.write("/gradient5", grad5_msg)

        length_msg = LengthStamped()
        length_msg.header.stamp = rospy.Time.from_sec(i)
        length_msg.length = i
        bag.write("/length", length_msg)

        ecb_curr_msg = CurrentsBlockStamped()
        ecb_curr_msg.header.stamp = rospy.Time.from_sec(i)
        ecb_curr_msg.block.currents_mA = [i] * 320
        bag.write("/currents_block", ecb_curr_msg)

        april_msg = AprilTagDetectionArray()
        april_msg.header.stamp = rospy.Time.from_sec(i)
        detection = AprilTagDetection()
        detection.id = [i]
        detection.size = [i]
        detection.pose.pose.pose.position = np_to_point_msg([i, 0, 0])
        april_msg.detections.append(detection)
        bag.write("/april_tag", april_msg)

    for i in range(10):
        img_msg = Image()
        img_msg.header.stamp = rospy.Time.from_sec(i)
        img_msg.width = 320
        img_msg.height = 240
        img_msg.encoding = "mono8"
        img_msg.data = [0] * (320 * 240)
        bag.write("/image", img_msg)

        cv_img = np.zeros((240, 320, 3), np.uint8)
        comp_msg = CompressedImage()
        comp_msg.header.stamp = rospy.Time.from_sec(i)
        comp_msg.format = "jpeg"
        comp_msg.data = np.array(cv2.imencode(".jpg", cv_img)[1]).tostring()
        bag.write("/image_compressed", comp_msg)

    bag.close()
