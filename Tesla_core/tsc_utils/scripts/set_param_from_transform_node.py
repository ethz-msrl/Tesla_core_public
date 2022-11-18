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
import tf

if __name__ == "__main__":

    rospy.init_node("set_param_from_transform")

    parent_frame = rospy.get_param("~parent_frame", "/parent_frame")
    child_frame = rospy.get_param("~child_frame", "/child_frame")

    rospy.set_param("~setup_registration/parent_frame", parent_frame)
    rospy.set_param("~setup_registration/child_frame", child_frame)
    rospy.loginfo("Check transform between %s and %s", parent_frame, child_frame)

    listener = tf.TransformListener()

    r = rospy.Rate(1)

    while not rospy.is_shutdown():
        try:
            (trans, rot) = listener.lookupTransform(
                parent_frame, child_frame, rospy.Time(0)
            )
        except (
            tf.LookupException,
            tf.ConnectivityException,
            tf.ExtrapolationException,
        ):
            rospy.logwarn(
                "Transform between %s and %s does not exist", parent_frame, child_frame
            )
            r.sleep()
            continue

        rospy.set_param("~setup_registration/translation", trans)
        rospy.set_param("~setup_registration/quaternion", rot)
        rospy.loginfo(
            "Transform parameters: \n parent_frame: %s \n child_frame: %s \n translation: %s \n rotation: %s",
            parent_frame,
            child_frame,
            trans,
            rot,
        )

        r.sleep()

    rospy.spin()
