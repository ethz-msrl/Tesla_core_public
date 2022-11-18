#! /usr/bin/env bash
set -e

underscore_to_hyphen ()
{
    echo $1 | tr '_' '-'
    return 0
}

get_package_version ()
{
    location=$(catkin locate $1)
    package_fp="$location/package.xml"
    package_version=$(xmllint --xpath '//version/text()' $package_fp)
    echo "$package_version"-0
    return 0
}

ARCH=$(dpkg --print-architecture)
#ROS_DISTRO=melodic
DISTRO=$(lsb_release -c | cut -f 2)
BUILD_DIR=/packages
WS_DIR=/tc_ws

if [ ! -d "$BUILD_DIR/$DISTRO" ]; then
    mkdir -p "$BUILD_DIR/$DISTRO"
fi

cd $WS_DIR
source "$WS_DIR/devel/setup.bash"

if [ -z "${ROS_DISTRO}" ]; then
   echo "ROS_DISTRO environment variable is not set. Make sure ros is installed."
   exit -1
fi

echo "ROS version installed: $ROS_DISTRO"

# packages are sorted in topological order
packages_l=($(catkin list -u))

for package in "${packages_l[@]}"; do
    echo "Installing $package"
    PKG_NAME=$(underscore_to_hyphen $package)
    PKG_VERSION=$(get_package_version $package)
    DEB_FN=ros-${ROS_DISTRO}-${PKG_NAME}_${PKG_VERSION}${DISTRO}_$ARCH.deb
    cd "$(catkin locate "$package")"
    bloom-generate rosdebian
    fakeroot debian/rules binary
    dpkg -i ../$DEB_FN
    mv ../$DEB_FN "$BUILD_DIR/$DISTRO"
    rosdep update
done

# ARCH=amd64
# ROS_DISTRO=melodic
# DISTRO=bionic
# 
# PKG_NAME=catkin-simple
# PKG_VERSION=0.1.1-0
# DEB_FN=ros-${ROS_DISTRO}-${PKG_NAME}_${PKG_VERSION}${DISTRO}_$ARCH.deb
# cd /tc_ws/src/external/catkin_simple
# bloom-generate rosdebian
# fakeroot debian/rules binary
# dpkg -i ../$DEB_FN
# mv ../$DEB_FN /tc_ws/packages/$DISTRO
# rosdep update
# 
# PKG_NAME=mag-msgs
# PKG_VERSION=3.0.0-0
# DEB_FN=ros-${ROS_DISTRO}-${PKG_NAME}_${PKG_VERSION}${DISTRO}_$ARCH.deb
# cd /tc_ws/src/Tesla_core/tsc_msgs/mag_msgs
# bloom-generate rosdebian
# fakeroot debian/rules binary
# dpkg -i ../$DEB_FN
# mv ../$DEB_FN /tc_ws/packages/$DISTRO
# rosdep update
# 
# PKG_NAME=ecb-msgs
# PKG_VERSION=3.0.0-0
# DEB_FN=ros-${ROS_DISTRO}-${PKG_NAME}_${PKG_VERSION}${DISTRO}_$ARCH.deb
# cd /tc_ws/src/Tesla_core/tsc_msgs/ecb_msgs
# bloom-generate rosdebian
# fakeroot debian/rules binary
# dpkg -i ../$DEB_FN
# mv ../$DEB_FN /tc_ws/packages/$DISTRO
# rosdep update
# 
# PKG_NAME=mag-tensorflow
# PKG_VERSION=3.0.0-0
# DEB_FN=ros-${ROS_DISTRO}-${PKG_NAME}_${PKG_VERSION}${DISTRO}_$ARCH.deb
# cd /tc_ws/src/Tesla_core/mag_control/mag_tensorflow
# bloom-generate rosdebian
# fakeroot debian/rules binary
# dpkg -i ../$DEB_FN
# mv ../$DEB_FN /tc_ws/packages/$DISTRO
# rosdep update
# 
# PKG_NAME=tsc-utils
# PKG_VERSION=3.0.0-0
# DEB_FN=ros-${ROS_DISTRO}-${PKG_NAME}_${PKG_VERSION}${DISTRO}_$ARCH.deb
# cd /tc_ws/src/Tesla_core/tsc_utils
# bloom-generate rosdebian
# fakeroot debian/rules binary
# dpkg -i ../$DEB_FN
# mv ../$DEB_FN /tc_ws/packages/$DISTRO
# rosdep update
# 
# PKG_NAME=mpem
# PKG_VERSION=3.0.0-0
# DEB_FN=ros-${ROS_DISTRO}-${PKG_NAME}_${PKG_VERSION}${DISTRO}_$ARCH.deb
# cd /tc_ws/src/Tesla_core/mag_control/mpem
# bloom-generate rosdebian
# fakeroot debian/rules binary
# dpkg -i ../$DEB_FN
# mv ../$DEB_FN /tc_ws/packages/$DISTRO
# rosdep update
# 
# PKG_NAME=mag-manip
# PKG_VERSION=3.0.0-0
# DEB_FN=ros-${ROS_DISTRO}-${PKG_NAME}_${PKG_VERSION}${DISTRO}_$ARCH.deb
# cd /tc_ws/src/Tesla_core/mag_control/mag_manip
# bloom-generate rosdebian
# fakeroot debian/rules binary
# dpkg -i ../$DEB_FN
# mv ../$DEB_FN /tc_ws/packages/$DISTRO
# rosdep update
# 
# PKG_NAME=mag-calculator
# PKG_VERSION=3.0.0-0
# DEB_FN=ros-${ROS_DISTRO}-${PKG_NAME}_${PKG_VERSION}${DISTRO}_$ARCH.deb
# cd /tc_ws/src/Tesla_core/mag_control/mag_calculator
# bloom-generate rosdebian
# fakeroot debian/rules binary
# dpkg -i ../$DEB_FN
# mv ../$DEB_FN /tc_ws/packages/$DISTRO
# rosdep update
