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

import csv

import rospy
from mag_msgs.msg import FieldStamped

"""
This ROS node takes an input space-delimeted CSV file with the following format:
    fieldX fieldY fieldZ posX posY posZ duration

where:
    field are the magnetic fields in T to be applied
    and pos is the position at which the field should be applied in the magnetic workspae
    Once an entry has been applied for the required duration, the next row will be applied

"""

if __name__ == "__main__":

    rospy.init_node("cmag_fields_list", log_level=rospy.DEBUG)

    fields_list_filename = rospy.get_param("~fields_list_filename", "fields_list.csv")
    des_field_pub = rospy.Publisher("/backward_cmag/field", FieldStamped, queue_size=1)

    # need to sleep a bit before publishing the next message
    time_to_wait = rospy.get_param("~time_to_wait", 1.0)

    with open(fields_list_filename, "r") as f:
        while time_to_wait > 0:
            rospy.loginfo("Starting in %d s", time_to_wait)
            rospy.sleep(1)
            time_to_wait = time_to_wait - 1

        reader = csv.reader(f, delimiter=" ")
        for row in reader:
            if rospy.is_shutdown():
                break

            if len(row) != 7:
                rospy.logerr("Invalid row in %s", fields_list_filename)
                continue

            fields_str = row[0:3]
            field = [float(f) for f in fields_str]

            pos_str = row[3:6]
            pos = [float(p) for p in pos_str]

            duration = float(row[6])

            desired_field_msg = FieldStamped()
            desired_field_msg.header.frame_id = "mns"
            desired_field_msg.header.stamp = rospy.Time.now()
            desired_field_msg.field.vector.x = field[0]
            desired_field_msg.field.vector.y = field[1]
            desired_field_msg.field.vector.z = field[2]
            desired_field_msg.field.position.x = pos[0]
            desired_field_msg.field.position.y = pos[1]
            desired_field_msg.field.position.z = pos[2]

            rospy.loginfo(
                "publishing field: %2.1f %2.1f %2.1f (mT)",
                1000 * field[0],
                1000 * field[1],
                1000 * field[2],
            )

            des_field_pub.publish(desired_field_msg)

            rospy.sleep(duration)
