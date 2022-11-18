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

import os.path
from builtins import object

import numpy as np
import rospkg
import rospy
import tf
from geometry_msgs.msg import PointStamped, Vector3Stamped
from mag_manip.mag_manip import BackwardModelMPEML2
from mag_msgs.msg import FieldStamped
from std_msgs.msg import Float32

# This node publishes the maximum magnitude of the field achievable in the
# direction of the field for given maximum currents in the coils and a linear
# model of the system
# subscriber: FieldStamped /backward_model/field : field
# publisher: Float32 /max_field  : max magnitude of the field achievable
# along the /backward_model/field
# parameters:
#     calibration_path : path to the calibration file of the system
#     max_currents : maximum current in the coils in A
#


class ComputeMaxField(object):
    def __init__(self):

        rospy.loginfo("Starting Max Field node")

        self.listener = tf.TransformListener()

        # Subscriber
        self.target_mf_sub = rospy.Subscriber(
            "/backward_model/field", FieldStamped, self.target_field_cb
        )

        # Publisher
        self.max_field_pub_ = rospy.Publisher("max_field", Float32, queue_size=1)

        rp = rospkg.RosPack()

        # Load params

        # Load system calibration
        cal_path = ""
        if not rospy.has_param("~calibration_path"):
            cal_path = os.path.join(
                rp.get_path("mpem"), "cal", "Navion_2_Calibration_24-02-2020.yaml"
            )
            rospy.logwarn("Calibration path not specified, defaulting to %s", cal_path)
        else:
            cal_path = rospy.get_param("~calibration_path")
        self.BM = BackwardModelMPEML2()
        self.BM.setCalibrationFile(cal_path)

        self.imax = rospy.get_param("~max_current", 35)
        rospy.logdebug("Max current : %s", self.imax)

        self.max_mf = rospy.get_param("max_mf", 0.02)

    def getMaxField(self, field, position):

        # Compute the transformation matrix
        bg_jac_inv = self.BM.getFieldActuationMatrixInverse(position)

        # Compute current
        currents = bg_jac_inv.dot(field)
        # Actual max target current
        maxCurrent = max(abs(currents))

        # Ratio of actual max current to admissible max current (must stay below 1)
        scaleI = maxCurrent / self.imax
        actual_field_norm = np.sqrt(np.sum(field ** 2, axis=0))

        # Compute max field in this direction
        if actual_field_norm > 0:
            maxField = actual_field_norm / scaleI
        else:
            maxField = self.max_mf

        return maxField

    def target_field_cb(self, msg):

        B = Vector3Stamped()
        B.header = msg.header
        B.vector = msg.field.vector

        P = PointStamped()
        P.header = msg.header
        P.point = msg.field.position

        # Transformation to mns frame using transformation listener
        try:
            Btransform = self.listener.transformVector3("mns", B)
        except (
            tf.LookupException,
            tf.ConnectivityException,
            tf.ExtrapolationException,
        ):
            rospy.logwarn(
                "Transformation between mns and frame of the field not \
                available: use frame of the message as mns frame !"
            )
            Btransform = B
            pass

        try:
            Ptransform = self.listener.transformPoint("mns", P)
        except (
            tf.LookupException,
            tf.ConnectivityException,
            tf.ExtrapolationException,
        ):
            rospy.logwarn(
                "Transformation between mns and frame of the field not \
                available: use frame of the message as mns frame !"
            )
            Ptransform = P
            pass

        # Extract desired field in mns frame
        Bx = Btransform.vector.x
        By = Btransform.vector.y
        Bz = Btransform.vector.z
        Px = Ptransform.point.x
        Py = Ptransform.point.y
        Pz = Ptransform.point.z
        field = np.array([Bx, By, Bz])
        position = np.array([Px, Py, Pz])

        # Compute max field
        MaxField = self.getMaxField(field, position)

        msg_max_field = Float32()
        msg_max_field.data = MaxField

        self.max_field_pub_.publish(msg_max_field)


if __name__ == "__main__":
    rospy.init_node("Compute_Max_Field", log_level=rospy.DEBUG)

    try:
        node = ComputeMaxField()
    except rospy.ROSInterruptException:
        pass

    r = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown():
        r.sleep()

    rospy.spin()
