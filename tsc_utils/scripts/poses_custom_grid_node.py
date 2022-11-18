#! /usr/bin/env python

from __future__ import division

from builtins import range

import rospy
from geometry_msgs.msg import Point, Pose, PoseArray

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

"""
This ROS node publishes the poses on a regular grid defined by the number of points
nx, ny and nz along x, y and z and the spacing in m with offsets offx/y/z
"""

if __name__ == "__main__":

    rospy.init_node("poses_list_node", log_level=rospy.DEBUG)

    poses_list_frame = rospy.get_param("~frame_id", "mns")
    nx = rospy.get_param("~nx", 10)
    ny = rospy.get_param("~ny", 10)
    nz = rospy.get_param("~nz", 10)
    offx = rospy.get_param("~offx", 0.0)
    offy = rospy.get_param("~offy", 0.0)
    offz = rospy.get_param("~offz", 0.0)
    spacing = rospy.get_param("~spacing", 0.02)

    poses_pub = rospy.Publisher("~poses", PoseArray, queue_size=1)

    poses_msg = PoseArray()
    poses_msg.header.frame_id = poses_list_frame
    poses_msg.header.stamp = rospy.Time.now()

    rate = rospy.Rate(1)  # 1hz

    for iX in range(nx):
        for iY in range(ny):
            for iZ in range(nz):
                cX = spacing * (nx - 1) / 2.0
                cY = spacing * (ny - 1) / 2.0
                cZ = spacing * (nz - 1) / 2.0
                p = Pose()
                p.position = Point(
                    float(iX * spacing - cX + offx),
                    float(iY * spacing - cY + offy),
                    float(iZ * spacing - cZ + offz),
                )
                poses_msg.poses.append(p)

    while not rospy.is_shutdown():

        poses_pub.publish(poses_msg)
        rate.sleep()
