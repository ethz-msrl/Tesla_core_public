#! /bin/bash

# shellcheck disable=SC1090
source /opt/ros/"$ROS_DISTRO"/setup.bash
rosdep init
rosdep update
