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

import os

import numpy as np
import rosbag
import rospkg
from numpy.testing import assert_allclose

import cv2
from tsc_utils.rosbag_extract import (get_available_topics, topic_to_df,
                                      topic_write_video)
r = rospkg.RosPack()
bag_path = os.path.join(r.get_path('tsc_utils'), 'nosetests', 'test.bag')


def test_topic_to_df_float32():
    bag = rosbag.Bag(bag_path)
    df = topic_to_df(bag, "/float32", "std_msgs/Float32")
    assert len(df) > 0
    assert_allclose(df.values[:, 0], np.arange(100))


def test_topic_to_df_float32multiarray():
    bag = rosbag.Bag(bag_path)
    df = topic_to_df(bag, "/float32multiarray", "std_msgs/Float32MultiArray")
    assert len(df) > 0
    assert_allclose(df.values[:, 0], np.arange(100))
    assert_allclose(df.values[:, 1], np.arange(100))


def test_topic_to_df_point_stamped():
    bag = rosbag.Bag(bag_path)
    df = topic_to_df(bag, "/point", "geometry_msgs/PointStamped")
    assert len(df) > 0
    assert_allclose(df["px"].values, df.index.values / 1e9)
    assert_allclose(df["py"].values, np.zeros((len(df),)))
    assert_allclose(df["pz"].values, np.zeros((len(df),)))


def test_topic_to_df_pose_stamped():
    bag = rosbag.Bag(bag_path)
    df = topic_to_df(bag, "/pose", "geometry_msgs/PoseStamped")
    assert len(df) > 0
    assert_allclose(df["px"].values, df.index.values / 1e9)
    assert_allclose(df["py"].values, np.zeros((len(df),)))
    assert_allclose(df["pz"].values, np.zeros((len(df),)))
    assert_allclose(df["qx"].values, df.index.values / 1e9)
    assert_allclose(df["qy"].values, np.zeros((len(df),)))
    assert_allclose(df["qz"].values, np.zeros((len(df),)))
    assert_allclose(df["qw"].values, np.zeros((len(df),)))


def test_topic_to_df_quaternion_stamped():
    bag = rosbag.Bag(bag_path)
    df = topic_to_df(bag, "/quaternion", "geometry_msgs/QuaternionStamped")
    assert len(df) > 0
    assert_allclose(df["qx"].values, df.index.values / 1e9)
    assert_allclose(df["qy"].values, np.zeros((len(df),)))
    assert_allclose(df["qz"].values, np.zeros((len(df),)))
    assert_allclose(df["qw"].values, np.zeros((len(df),)))


def test_topic_to_df_tran_stamped():
    bag = rosbag.Bag(bag_path)
    df = topic_to_df(bag, "/transform", "geometry_msgs/TransformStamped")
    assert len(df) > 0
    assert_allclose(df["tx"].values, df.index.values / 1e9)
    assert_allclose(df["ty"].values, np.zeros((len(df),)))
    assert_allclose(df["tz"].values, np.zeros((len(df),)))
    assert_allclose(df["qx"].values, df.index.values / 1e9)
    assert_allclose(df["qy"].values, np.zeros((len(df),)))
    assert_allclose(df["qz"].values, np.zeros((len(df),)))
    assert_allclose(df["qw"].values, np.zeros((len(df),)))


def test_topic_to_df_vector3_stamped():
    bag = rosbag.Bag(bag_path)
    df = topic_to_df(bag, "/vector3", "geometry_msgs/Vector3Stamped")
    assert len(df) > 0
    assert_allclose(df["vx"].values, df.index.values / 1e9)
    assert_allclose(df["vy"].values, np.zeros((len(df),)))
    assert_allclose(df["vz"].values, np.zeros((len(df),)))


def test_topic_to_df_wren_stamped():
    bag = rosbag.Bag(bag_path)
    df = topic_to_df(bag, "/wrench", "geometry_msgs/WrenchStamped")
    assert len(df) > 0
    assert_allclose(df["fx"].values, df.index.values / 1e9)
    assert_allclose(df["fy"].values, np.zeros((len(df),)))
    assert_allclose(df["fz"].values, np.zeros((len(df),)))
    assert_allclose(df["tx"].values, df.index.values / 1e9)
    assert_allclose(df["ty"].values, np.zeros((len(df),)))
    assert_allclose(df["tz"].values, np.zeros((len(df),)))


def test_topic_to_df_joy():
    bag = rosbag.Bag(bag_path)
    df = topic_to_df(bag, "/joy", "sensor_msgs/Joy")
    assert len(df) > 0
    assert_allclose(df["ax0"].values, df.index.values / 1e9)
    assert_allclose(df["button0"].values, df.index.values / 1e9)


def test_topic_to_df_joint_state():
    bag = rosbag.Bag(bag_path)
    df = topic_to_df(bag, "/joint_state", "sensor_msgs/JointState")
    assert len(df) > 0
    assert_allclose(df["joint_position"].values, df.index.values / 1e9)
    assert_allclose(df["joint_velocity"].values, df.index.values / 1e9)
    assert_allclose(df["joint_effort"].values, df.index.values / 1e9)


def test_topic_to_df_field_stamped():
    bag = rosbag.Bag(bag_path)
    df = topic_to_df(bag, "/field", "mag_msgs/FieldStamped")
    assert len(df) > 0
    assert_allclose(df["bx"].values, df.index.values / 1e9)
    assert_allclose(df["by"].values, np.zeros((len(df),)))
    assert_allclose(df["bz"].values, np.zeros((len(df),)))


def test_topic_to_df_currents_stamped():
    bag = rosbag.Bag(bag_path)
    df = topic_to_df(bag, "/currents", "mag_msgs/CurrentsStamped")
    assert len(df) > 0
    assert_allclose(df["I0"].values, df.index.values / 1e9)
    assert_allclose(df["I1"].values, df.index.values / 1e9)
    assert_allclose(df["I2"].values, df.index.values / 1e9)
    assert_allclose(df["I3"].values, df.index.values / 1e9)
    assert_allclose(df["I4"].values, df.index.values / 1e9)
    assert_allclose(df["I5"].values, df.index.values / 1e9)
    assert_allclose(df["I6"].values, df.index.values / 1e9)
    assert_allclose(df["I7"].values, df.index.values / 1e9)


def test_topic_to_df_field_array_stamped():
    bag = rosbag.Bag(bag_path)
    df = topic_to_df(bag, "/field_array", "mag_msgs/FieldArrayStamped")
    assert len(df) == 100
    assert_allclose(df["bx0"].values, df.index.values / 1e9)
    assert_allclose(df["by0"].values, np.zeros((len(df),)))
    assert_allclose(df["bz0"].values, np.zeros((len(df),)))
    assert_allclose(df["bx1"].values, df.index.values / 1e9)
    assert_allclose(df["by1"].values, np.zeros((len(df),)))
    assert_allclose(df["bz1"].values, np.zeros((len(df),)))
    assert_allclose(df["bx2"].values, df.index.values / 1e9)
    assert_allclose(df["by2"].values, np.zeros((len(df),)))
    assert_allclose(df["bz2"].values, np.zeros((len(df),)))


def test_topic_to_df_currents_block_stamped():
    bag = rosbag.Bag(bag_path)
    df = topic_to_df(bag, "/currents_block", "ecb_msgs/CurrentsBlockStamped")
    assert_allclose(df.iloc[:, 0].values, df.index.values / 1e9)
    assert_allclose(df.iloc[:, 1].values, df.index.values / 1e9)
    assert_allclose(df.iloc[:, 319].values, df.index.values / 1e9)
    assert len(df) == 100


def test_topic_to_df_length_stamped():
    bag = rosbag.Bag(bag_path)
    df = topic_to_df(bag, "/length", "mag_rod_msgs/LengthStamped")
    assert len(df) == 100
    assert_allclose(df["length"].values, df.index.values / 1e9)


def test_topic_to_df_grad3_stamped():
    bag = rosbag.Bag(bag_path)
    df = topic_to_df(bag, "/gradient3", "mag_msgs/Gradient3Stamped")
    assert len(df) == 100
    assert_allclose(df["gx"].values, df.index.values / 1e9)


def test_topic_to_df_grad5_stamped():
    bag = rosbag.Bag(bag_path)
    df = topic_to_df(bag, "/gradient5", "mag_msgs/Gradient5Stamped")
    assert len(df) == 100
    assert_allclose(df["bxx"].values, df.index.values / 1e9)
    assert_allclose(df["bxy"].values, np.zeros((len(df),)))


def test_topic_to_df_field_gradient3_stamped():
    bag = rosbag.Bag(bag_path)
    df = topic_to_df(bag, "/field_gradient3", "mag_msgs/FieldGradient3Stamped")
    assert len(df) == 100
    assert_allclose(df["bx"].values, df.index.values / 1e9)
    assert_allclose(df["gx"].values, df.index.values / 1e9)
    assert_allclose(df["gy"].values, np.zeros((len(df),)))


def test_topic_to_df_field_gradient5_stamped():
    bag = rosbag.Bag(bag_path)
    df = topic_to_df(bag, "/field_gradient5", "mag_msgs/FieldGradient5Stamped")
    assert len(df) == 100
    assert_allclose(df["bx"].values, df.index.values / 1e9)
    assert_allclose(df["bxx"].values, df.index.values / 1e9)
    assert_allclose(df["bxy"].values, np.zeros((len(df),)))


def test_topic_to_df_april_tag_detection_array():
    bag = rosbag.Bag(bag_path)
    df = topic_to_df(bag, "/april_tag", "apriltag_ros/AprilTagDetectionArray")
    assert_allclose(df["ident"].values, df.index.values / 1e9)
    assert_allclose(df["size"].values, df.index.values / 1e9)
    assert_allclose(df["pos_x"].values, df.index.values / 1e9)
    assert len(df) == 100


def test_get_available_topics():
    bag = rosbag.Bag(bag_path)
    print(get_available_topics(bag))


def test_topic_write_video():
    bag = rosbag.Bag(bag_path)
    topic_write_video(bag, "/image", "sensor_msgs/Image", ".")
    cap = cv2.VideoCapture("_image.mp4")

    while cap.grab():
        ret, img = cap.retrieve()
        assert ret
        assert_allclose(img, np.zeros((240, 320, 3), np.uint8))
    os.remove("_image.mp4")
