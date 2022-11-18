cmake_minimum_required(VERSION 2.8.3)
project(mag_calculator)

find_package(catkin_simple REQUIRED)
catkin_simple()

add_definitions(-std=c++11 -Wall -Wextra)

cs_add_executable(backward_model src/backward_model.cpp)
cs_add_executable(forward_model src/forward_model.cpp)
cs_add_executable(compute_max_field src/compute_max_field_node.cpp)

catkin_install_python(PROGRAMS 
    scripts/ComputeMaxField.py
    scripts/currents_list_node.py
    DESTINATION
    ${CATKIN_PACKAGE_BIN_DESTINATION})

cs_install()  
cs_export()

# Add custom (non compiling) targets so launch scripts and python files show up in QT Creator's project view.
file(GLOB_RECURSE EXTRA_FILES */*)
add_custom_target(${PROJECT_NAME}_OTHER_FILES ALL WORKING_DIRECTORY ${PROJECT_SOURCE_DIR} SOURCES ${EXTRA_FILES})
