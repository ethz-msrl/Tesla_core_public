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
import random
import numpy as np
import std_msgs.msg
from mag_msgs.msg import FieldArrayStamped, Field


def create_message():
    msg = FieldArrayStamped()

    header = std_msgs.msg.Header()
    header.stamp = rospy.Time.now()
    header.frame_id = "mns"
    msg.header = header

    point_array = generate_position_array()
    field_vector = generate_field_vector()

    for i in range(len(point_array)):
        field = Field()
        field.vector.x = field_vector[0]
        field.vector.y = field_vector[1]
        field.vector.z = field_vector[2]
        field.position.x = point_array[i, 0]
        field.position.y = point_array[i, 1]
        field.position.z = point_array[i, 2]
        msg.fields.append(field)

    return msg


def generate_position_array():
    nx = 11
    ny = 7
    nz = 3
    dist = 0.02

    point = np.array([0 - int(nx/2) * dist, 0 - int(ny/2)*dist, 0 - int(nz/2)*dist])
    point_array = np.zeros([nx*ny*nz, 3], dtype=float)

    i = 0
    for z in range(nz):
        for y in range(ny):
            for x in range(nx):
                new_point = np.array([point[0] + x*dist, point[1] + y*dist, point[2] + z*dist])
                point_array[i] = new_point
                i += 1
    return point_array


def generate_field_vector():
    field_vector = 2 * np.random.rand(3) - 1  # random sample from -1 to 1
    field_vector_normalized = field_vector / np.linalg.norm(field_vector)
    field_vector_tesla = field_vector_normalized * random.random() * 0.03

    return field_vector_tesla


if __name__ == '__main__':
    try:

        field_pub = rospy.Publisher('navion/actual_field', FieldArrayStamped, queue_size=1)
        rospy.init_node('test_field_array_pub_node', anonymous=True)
        rate = rospy.Rate(1)

        while not rospy.is_shutdown():
            msg = create_message()
            field_pub.publish(msg)
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
