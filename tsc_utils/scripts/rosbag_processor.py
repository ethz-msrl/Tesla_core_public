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

from __future__ import print_function

import argparse
import glob
import os
import sys

import rosbag
from tsc_utils.rosbag_extract import get_available_topics, topic_to_df, topic_write_video

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="extracts various types of messages from a rosbag"
    )
    parser.add_argument(
        "path", help="path to a rosbag or a folder containg several rosbags"
    )
    parser.add_argument("out", help="folder to save output data")
    parser.add_argument("--ignore", nargs="+", help="list of topics to ignore")
    parser.add_argument(
        "--types",
        nargs="+",
        help="list of message types to process [ex: mag_msgs/FieldStamped]",
    )
    parser.add_argument(
        "--show_topics",
        action="store_true",
        help="list the topics that can be extracted from the bag",
    )

    args = parser.parse_args()

    if os.path.isfile(args.path):
        bag_paths = [args.path]
    elif os.path.isdir(args.path):
        bag_paths = glob.glob(os.path.join(args.path, "*.bag"))
    else:
        raise ValueError("Unable to load bag file or folder: {}".format(args.path))

    if args.show_topics:
        for bag_path in bag_paths:
            print("bag: {} topics\n".format(bag_path))
            bag = rosbag.Bag(bag_path)
            available_topics = get_available_topics(bag)
            for topic in available_topics:
                print(topic)
        sys.exit()

    if args.ignore:
        print("will try to ignore the following topics\n")
        for t in args.ignore:
            print(t)

    if args.types:
        print("will try to only export the following message types\n")
        for t in args.types:
            print(t)

    if not os.path.isdir(args.out):
        raise ValueError("{:s} is not a valid directory".format(args.out))

    for bag_path in bag_paths:
        print("\nprocessing bag: {:s}".format(bag_path))
        bag = rosbag.Bag(bag_path)
        available_topics = get_available_topics(bag)

        if args.types:
            available_topics = {
                k: v for k, v in available_topics.items() if v in args.types
            }

        img_topics = [
            k
            for k, v in available_topics.items()
            if v in ("sensor_msgs/Image", "sensor_msgs/CompressedImage")
        ]
        if args.ignore:
            topics = set(available_topics.keys()) - set(args.ignore)
        else:
            topics = available_topics.keys()

        numerical_topics = set(topics) - set(img_topics)

        if len(bag_paths) > 1:
            out_folder = os.path.join(
                args.out, os.path.splitext(os.path.basename(bag_path))[0]
            )
            os.mkdir(out_folder)
        else:
            out_folder = args.out

        for topic in numerical_topics:
            df = topic_to_df(bag, topic, available_topics[topic])
            csv_filename = topic.replace("/", "_") + ".csv"
            csv_path = os.path.join(out_folder, csv_filename)
            df.to_csv(csv_path)

        for topic in img_topics:
            topic_write_video(bag, topic, available_topics[topic], out_folder)
