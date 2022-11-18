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
from visualization_msgs.msg import Marker

"""Some help functions for ROS RVIZ"""


def rviz_mesh_marker(
    t=None,
    frame_id="world",
    mesh_file_path="",
    marker_id=0,
    position=[0.0, 0.0, 0],
    orientation=[0.0, 0.0, 0.0, 1.0],
    scale_factor=[1.0, 1.0, 1.0],
    color_rgb_alpha=[0.5, 1.0, 1.0, 0.7],
):

    """Generates RVIZ markers from mesh files (.stl, .obj, .dae, ...)

    Returns:
        ros marker object
    """

    if t is None:
        t = rospy.get_rostime().to_sec()
    # mesh_file_path example: 'package://package_name/mesh/mesh_file.obj'
    if mesh_file_path == "":
        rospy.logwarn("No mesh file path given.")

    mesh_marker = Marker()
    mesh_marker.header.frame_id = frame_id
    mesh_marker.header.stamp = rospy.Time.from_sec(t)
    mesh_marker.action = mesh_marker.ADD
    mesh_marker.type = mesh_marker.MESH_RESOURCE
    mesh_marker.mesh_resource = mesh_file_path
    mesh_marker.id = marker_id
    mesh_marker.pose.position.x = position[0]
    mesh_marker.pose.position.y = position[1]
    mesh_marker.pose.position.z = position[2]
    mesh_marker.pose.orientation.x = orientation[0]
    mesh_marker.pose.orientation.y = orientation[1]
    mesh_marker.pose.orientation.z = orientation[2]
    mesh_marker.pose.orientation.w = orientation[3]

    mesh_marker.scale.x = scale_factor[0]
    mesh_marker.scale.y = scale_factor[1]
    mesh_marker.scale.z = scale_factor[2]

    mesh_marker.color.a = color_rgb_alpha[3]
    mesh_marker.color.r = color_rgb_alpha[0]
    mesh_marker.color.g = color_rgb_alpha[1]
    mesh_marker.color.b = color_rgb_alpha[2]

    return mesh_marker
