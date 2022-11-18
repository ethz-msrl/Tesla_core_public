#! /usr/bin/env python
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

import rospy
from tsc_utils.rviz_utils import rviz_mesh_marker
from visualization_msgs.msg import Marker

if __name__ == "__main__":

    rospy.init_node("publish_rviz_marker")

    frame_id = rospy.get_param("~frame_id", "world")
    mesh_file_path = rospy.get_param(
        "~mesh_file_path"
    )  # mesh_file_path example: 'package://package_name/mesh/mesh_file.obj'
    marker_id = rospy.get_param("~marker_id", 0)
    position = rospy.get_param("~position", [0.0, 0.0, 0])
    orientation = rospy.get_param("~orientation", [0.0, 0.0, 0.0, 1.0])
    scale_factor = rospy.get_param("~scale_factor", [1.0, 1.0, 1.0])
    color_rgb_alpha = rospy.get_param("~color_rgb_alpha", [0.5, 1.0, 1.0, 0.7])

    marker_pub = rospy.Publisher("~mesh_markers", Marker, queue_size=1)
    mesh_marker = Marker()
    mesh_marker = rviz_mesh_marker(
        None,
        frame_id,
        mesh_file_path,
        marker_id,
        position,
        orientation,
        scale_factor,
        color_rgb_alpha,
    )

    r = rospy.Rate(30)

    while not rospy.is_shutdown():
        t = rospy.get_rostime().to_sec()
        mesh_marker.header.stamp = rospy.Time.from_sec(t)
        marker_pub.publish(mesh_marker)
        r.sleep()

    rospy.spin()
