from __future__ import print_function

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

import timeit

import numpy as np
import tensorflow as tf
from tensorflow.compat.v1.saved_model import loader


def normalize_currents(currents):
    return (2 * (currents + 35) / 70) - 1


class Benchmark(object):
    def __init__(self, model_dir):
        self.graph = tf.Graph()
        self.sess = tf.Session(graph=self.graph)
        loader.load(self.sess, ["serve"], model_dir)
        self.input_t = self.graph.get_tensor_by_name("Placeholder:0")
        self.output = self.graph.get_tensor_by_name("G_2/13_conv/BiasAdd:0")
        self.field_scale = 0.36339

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.sess.close()

    def compute_field(self):
        currents = np.zeros((1, 8), np.float)
        field_out = self.sess.run(
            "G_2/13_conv/BiasAdd:0", {"Placeholder:0": normalize_currents(currents)}
        )
        field_out *= self.field_scale
        field_out = np.squeeze(field_out)
        field_out = np.transpose(field_out, (2, 1, 0, 3))


if __name__ == "__main__":
    b = Benchmark("../models/cmag_cnn_v1")
    times = timeit.Timer(b.compute_field).repeat(3, 100)

    # Divide by the number of repeats
    time_taken = min(times) / 100
    print("time_taken for a single field computation: {} ms".format(time_taken * 1000))
