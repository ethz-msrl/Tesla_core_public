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
from mag_msgs.msg import CurrentsStamped

"""
This ROS node takes an input space-delimeted CSV file with the following format:
    current0 ... currentN duration

where:
    current0 ... currentN are the currents in Amps to be applied on the coils
    duration is a time in seconds indicating how long the current should be applied
    Once a current has been applied for the required duration, the next row will be applied

The currents are published on topic desired_currents. If you want this to get mapped to particular system,
you can either remap the topic to the appropriate input, or you can run this in a namespace.

For example, if the cardiomag_controller listens on /cmag/desired_currents,
then run this node in a namespace called /cmag

The currents can be generated using the MSS calibration in MATLAB or using jupyter
to output the CSV file.
The appropriate current message is then set
"""

if __name__ == "__main__":

    rospy.init_node("currents_list_node", log_level=rospy.DEBUG)

    currents_list_filename = rospy.get_param("~input_filename", "currents_list.csv")
    num_coils = rospy.get_param("~num_coils", 8)
    currents_topic = rospy.get_param("~currents_topic", "desired_currents")

    des_currents_pub = rospy.Publisher(
        currents_topic, CurrentsStamped, queue_size=1
    )

    # need to sleep a bit before publishing the next message
    time_to_wait = rospy.get_param("~time_to_wait", 1.0)

    with open(currents_list_filename, "r") as f:
        while time_to_wait > 0:
            rospy.loginfo("Starting in %d s", time_to_wait)
            rospy.sleep(1)
            time_to_wait = time_to_wait - 1

        reader = csv.reader(f, delimiter=" ")
        for row in reader:
            if rospy.is_shutdown():
                break

            if len(row) != num_coils + 1:
                rospy.logerr("Invalid row in %s", currents_list_filename)
                continue

            currents_str = row[0:num_coils]
            currents = [float(c) for c in currents_str]

            duration = float(row[num_coils])

            desired_currents_msg = CurrentsStamped()
            desired_currents_msg.header.frame_id = "map"
            desired_currents_msg.header.stamp = rospy.Time.now()
            desired_currents_msg.currents = currents

            rospy.loginfo("publishing currents: %s", str.join(" ", currents_str))

            des_currents_pub.publish(desired_currents_msg)

            rospy.sleep(duration)
