#! /bin/bash

catkin build mag_manip --no-deps --cmake-args -DENABLE_COVERAGE_TESTING=ON -DCMAKE_BUILD_TYPE=Debug -DCMAKE_CXX_FLAGS="-fno-inline -fno-inline-small-functions -fno-default-inline"

catkin build mag_manip -v --no-deps --catkin-make-args mag_manip_coverage_report

