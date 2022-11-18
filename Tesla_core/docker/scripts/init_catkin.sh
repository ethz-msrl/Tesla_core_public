#! /bin/sh

# caution, this needs to be run from the top level of the catkin workspace
catkin init --workspace .
catkin config --extend /opt/ros/"$ROS_DISTRO"
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
