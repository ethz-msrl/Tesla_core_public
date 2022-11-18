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
import shutil
import subprocess
from collections import namedtuple

import cv_bridge
import numpy as np
from tsc_utils.conversions import (
    gradient5_to_np,
    point_msg_to_np,
    quaternion_msg_to_np,
    vector3_msg_to_np,
)

import cv2
import pandas as pd

scalar_msg_types = (
    "std_msgs/Float32",
    "std_msgs/Float64",
    "std_msgs/Int8",
    "std_msgs/Int16",
    "std_msgs/Int32",
    "std_msgs/Int64",
    "std_msgs/UInt16",
    "std_msgs/UInt32",
    "std_msgs/UInt64",
)

multiarray_types = tuple(msg_type + "MultiArray" for msg_type in scalar_msg_types)

available_message_types = [
    "geometry_msgs/PointStamped",
    "geometry_msgs/PoseStamped",
    "geometry_msgs/QuaternionStamped",
    "geometry_msgs/TransformStamped",
    "geometry_msgs/Vector3Stamped",
    "geometry_msgs/WrenchStamped",
    "sensor_msgs/Joy",
    "sensor_msgs/JointState",
    "sensor_msgs/Image",
    "sensor_msgs/CompressedImage",
    "mag_msgs/CurrentsStamped",
    "mag_msgs/FieldStamped",
    "mag_msgs/FieldArrayStamped",
    "mag_msgs/FieldGradient3Stamped",
    "mag_msgs/FieldGradient5Stamped",
    "mag_msgs/Gradient3Stamped",
    "mag_msgs/Gradient5Stamped" "mag_rod_msgs/LengthStamped",
    "ecb_msgs/CurrentsBlockStamped" "apriltag_ros/AprilTagDetectionArray",
]
available_message_types += scalar_msg_types
available_message_types += multiarray_types


def topic_to_df(bag, topic, msg_type):
    """
    Extracts a known topic in a rosbag to a pandas DataFrame

    Args:
        bag (rosbag.Bag): the bag file
        topic (str): the name of the topic
        msg_type (str): the full message type. See available_message_types
    Returns:
        pandas.DataFrame containing the exported data
    """
    holder = namedtuple("Timeseries", ["data", "stamps", "columns"])
    holder.stamps = []
    holder.data = []

    if msg_type in scalar_msg_types:
        for topic, msg, stamp in bag.read_messages(topics=(topic)):
            holder.stamps.append(stamp.to_nsec())
            holder.data.append(msg.data)
        holder.columns = None

    if msg_type in multiarray_types:
        for topic, msg, stamp in bag.read_messages(topics=(topic)):
            holder.stamps.append(stamp.to_nsec())
            holder.data.append(msg.data)
        holder.columns = None

    if msg_type == "geometry_msgs/PointStamped":
        for topic, msg, stamp in bag.read_messages(topics=(topic)):
            # use stamp from message instead of from bag
            holder.stamps.append(msg.header.stamp.to_nsec())
            point = point_msg_to_np(msg.point)
            holder.data.append(point)
        holder.columns = ("px", "py", "pz")

    if msg_type == "geometry_msgs/PoseStamped":
        for topic, msg, stamp in bag.read_messages(topics=(topic)):
            # use stamp from message instead of from bag
            holder.stamps.append(msg.header.stamp.to_nsec())
            position = point_msg_to_np(msg.pose.position)
            orientation = quaternion_msg_to_np(msg.pose.orientation)
            holder.data.append(np.concatenate((position, orientation)))
        holder.columns = ("px", "py", "pz", "qx", "qy", "qz", "qw")

    if msg_type == "geometry_msgs/QuaternionStamped":
        for topic, msg, stamp in bag.read_messages(topics=(topic)):
            # use stamp from message instead of from bag
            holder.stamps.append(msg.header.stamp.to_nsec())
            orientation = quaternion_msg_to_np(msg.quaternion)
            holder.data.append(orientation)
        holder.columns = ("qx", "qy", "qz", "qw")

    if msg_type == "geometry_msgs/TransformStamped":
        for topic, msg, stamp in bag.read_messages(topics=(topic)):
            # use stamp from message instead of from bag
            holder.stamps.append(msg.header.stamp.to_nsec())
            position = point_msg_to_np(msg.transform.translation)
            orientation = quaternion_msg_to_np(msg.transform.rotation)
            holder.data.append(np.concatenate((position, orientation)))
        holder.columns = ("tx", "ty", "tz", "qx", "qy", "qz", "qw")

    if msg_type == "geometry_msgs/Vector3Stamped":
        for topic, msg, stamp in bag.read_messages(topics=(topic)):
            # use stamp from message instead of from bag
            holder.stamps.append(msg.header.stamp.to_nsec())
            holder.data.append(vector3_msg_to_np(msg.vector))
        holder.columns = ("vx", "vy", "vz")

    if msg_type == "geometry_msgs/WrenchStamped":
        for topic, msg, stamp in bag.read_messages(topics=(topic)):
            # use stamp from message instead of from bag
            holder.stamps.append(msg.header.stamp.to_nsec())
            force = vector3_msg_to_np(msg.wrench.force)
            torque = vector3_msg_to_np(msg.wrench.torque)
            holder.data.append(np.concatenate((force, torque)))
        holder.columns = ("fx", "fy", "fz", "tx", "ty", "tz")

    if msg_type == "sensor_msgs/Joy":
        for topic, msg, stamp in bag.read_messages(topics=(topic)):
            # use stamp from message instead of from bag
            holder.stamps.append(msg.header.stamp.to_nsec())
            holder.data.append(np.concatenate((msg.axes, msg.buttons)))
        axes_labels = ["ax{:d}".format(i) for i in range(len(msg.axes))]
        buttons_labels = ["button{:d}".format(i) for i in range(len(msg.buttons))]
        holder.columns = []
        holder.columns += axes_labels
        holder.columns += buttons_labels

    if msg_type == "sensor_msgs/JointState":
        for topic, msg, stamp in bag.read_messages(topics=(topic)):
            # use stamp from message instead of from bag
            holder.stamps.append(msg.header.stamp.to_nsec())
            holder.data.append(np.concatenate((msg.position, msg.velocity, msg.effort)))
        position_labels = ["{:s}_position".format(name) for name in msg.name]
        velocity_labels = ["{:s}_velocity".format(name) for name in msg.name]
        effort_labels = ["{:s}_effort".format(name) for name in msg.name]
        holder.columns = []
        holder.columns += position_labels
        holder.columns += velocity_labels
        holder.columns += effort_labels

    if msg_type == "mag_msgs/FieldStamped":
        for topic, msg, stamp in bag.read_messages(topics=(topic)):
            # use stamp from message instead of from bag
            holder.stamps.append(msg.header.stamp.to_nsec())
            position = point_msg_to_np(msg.field.position)
            field = point_msg_to_np(msg.field.vector)
            holder.data.append(np.concatenate((position, field)))
        holder.columns = ("px", "py", "pz", "bx", "by", "bz")

    elif msg_type == "mag_msgs/CurrentsStamped":
        for topic, msg, stamp in bag.read_messages(topics=(topic)):
            # use stamp from message instead of from bag
            holder.stamps.append(msg.header.stamp.to_nsec())
            holder.data.append(msg.currents)
        holder.columns = ["I{:d}".format(i) for i in range(len(msg.currents))]

    elif msg_type == "mag_msgs/FieldGradient3Stamped":
        for topic, msg, stamp in bag.read_messages(topics=(topic)):
            # use stamp from message instead of from bag
            holder.stamps.append(msg.header.stamp.to_nsec())
            position = point_msg_to_np(msg.position)
            field = point_msg_to_np(msg.field)
            vector = point_msg_to_np(msg.vector)
            holder.data.append(np.concatenate((position, field, vector)))
        holder.columns = ("px", "py", "pz", "bx", "by", "bz", "gx", "gy", "gz")

    elif msg_type == "mag_msgs/FieldGradient5Stamped":
        for topic, msg, stamp in bag.read_messages(topics=(topic)):
            # use stamp from message instead of from bag
            holder.stamps.append(msg.header.stamp.to_nsec())
            position = point_msg_to_np(msg.position)
            field = point_msg_to_np(msg.field)
            grad = gradient5_to_np(msg.vector)
            holder.data.append(np.concatenate((position, field, grad)))
        holder.columns = (
            "px",
            "py",
            "pz",
            "bx",
            "by",
            "bz",
            "bxx",
            "bxy",
            "bxz",
            "byy",
            "byz",
        )

    elif msg_type == "mag_msgs/Gradient3Stamped":
        for topic, msg, stamp in bag.read_messages(topics=(topic)):
            # use stamp from message instead of from bag
            holder.stamps.append(msg.header.stamp.to_nsec())
            position = point_msg_to_np(msg.position)
            grad = point_msg_to_np(msg.gradient)
            holder.data.append(np.concatenate((position, grad)))
        holder.columns = ("px", "py", "pz", "gx", "gy", "gz")

    elif msg_type == "mag_msgs/Gradient5Stamped":
        for topic, msg, stamp in bag.read_messages(topics=(topic)):
            # use stamp from message instead of from bag
            holder.stamps.append(msg.header.stamp.to_nsec())
            position = point_msg_to_np(msg.gradient.position)
            grad = gradient5_to_np(msg.gradient.vector)
            holder.data.append(np.concatenate((position, grad)))
        holder.columns = ("px", "py", "pz", "bxx", "bxy", "bxz", "byy", "byz")

    elif msg_type == "mag_rod_msgs/LengthStamped":
        for topic, msg, stamp in bag.read_messages(topics=(topic)):
            # use stamp from message instead of from bag
            holder.stamps.append(msg.header.stamp.to_nsec())
            holder.data.append([msg.length])
        holder.columns = ("length",)

    elif msg_type == "mag_msgs/FieldArrayStamped":
        # note that this will not output the field positions
        for topic, msg, stamp in bag.read_messages(topics=(topic)):
            # use stamp from message instead of from bag
            holder.stamps.append(msg.header.stamp.to_nsec())
            fields = [point_msg_to_np(field.vector) for field in msg.fields]
            fields = np.array(fields).ravel()
            holder.data.append(fields)
        holder.columns = np.array(
            [
                ["bx{:d}".format(i), "by{:d}".format(i), "bz{:d}".format(i)]
                for i in range(len(msg.fields))
            ]
        ).ravel()

    elif msg_type == "apriltag_ros/AprilTagDetectionArray":
        for topic, msg, stamp in bag.read_messages(topics=(topic)):
            holder.stamps.append(msg.header.stamp.to_nsec())
            ident = msg.detections[0].id[0]
            size = msg.detections[0].size[0]

            pos_x = msg.detections[0].pose.pose.pose.position.x
            pos_y = msg.detections[0].pose.pose.pose.position.y
            pos_z = msg.detections[0].pose.pose.pose.position.z

            quad_x = msg.detections[0].pose.pose.pose.orientation.x
            quad_y = msg.detections[0].pose.pose.pose.orientation.y
            quad_z = msg.detections[0].pose.pose.pose.orientation.z
            quad_w = msg.detections[0].pose.pose.pose.orientation.w

            holder.data.append(
                [ident, size, pos_x, pos_y, pos_z, quad_x, quad_y, quad_z, quad_w]
            )
        holder.columns = (
            "ident",
            "size",
            "pos_x",
            "pos_y",
            "pos_z",
            "quad_x",
            "quad_y",
            "quad_z",
            "quad_w",
        )

    elif msg_type == "ecb_msgs/CurrentsBlockStamped":
        for topic, msg, stamp in bag.read_messages(topics=(topic)):
            holder.stamps.append(msg.header.stamp.to_nsec())
            holder.data.append(msg.block.currents_mA)
        holder.columns = None

    df = pd.DataFrame(index=holder.stamps, columns=holder.columns, data=holder.data)
    df.index.name = "timestamp (ns)"

    return df


def topic_write_video(bag, topic, msg_type, top_dir, bitrate="5M"):
    """
    Writes the topic to a video file
    """
    bridge = cv_bridge.CvBridge()
    msg = bag.get_type_and_topic_info().topics[topic]
    freq = msg.frequency
    img_dir = os.path.join(top_dir, topic.split("/")[-1])

    if not os.path.exists(img_dir):
        os.mkdir(img_dir)

    fc = 0
    for topic, msg, _ in bag.read_messages(topics=topic):

        if msg_type == "sensor_msgs/Image":
            img = bridge.imgmsg_to_cv2(msg)
        elif msg_type == "sensor_msgs/CompressedImage":
            img = bridge.compressed_imgmsg_to_cv2(msg)
        else:
            raise ValueError("Invalid msg_type: {}".format(msg_type))

        img_file = os.path.join(img_dir, "{:08d}".format(fc) + ".jpg")
        cv2.imwrite(img_file, img)
        fc += 1

    video = topic.replace("/", "_") + ".mp4"
    img_format_str = os.path.join(img_dir, "%08d.jpg")
    ret = subprocess.call(
        [
            "ffmpeg",
            "-f",
            "image2",
            "-i",
            img_format_str,
            "-r",
            "%d" % freq,
            "-b",
            bitrate,
            video,
        ]
    )

    if ret == 0:
        print("video encoded for topic {}".format(topic))
    else:
        print("video encoding failed for topic {}".format(topic))

    shutil.rmtree(img_dir)


def get_available_topics(bag):
    """
    Returns the topics in a bag that can be exported

    Args:
        bag (rosbag.Bag): the bag file
    Returns:
        dict with keys as topic names and values as message type of available topics
    """
    info = bag.get_type_and_topic_info()[1]

    return {
        topic: value.msg_type
        for topic, value in info.items()
        if value.msg_type in available_message_types
    }
