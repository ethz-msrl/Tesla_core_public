cmake_minimum_required(VERSION 2.8.3)
project(tsc_utils)

find_package(catkin_simple REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

catkin_simple()

add_definitions(-std=c++11 -Wall -Wextra)

catkin_python_setup()

if (CATKIN_ENABLE_TESTING)
    catkin_add_nosetests(nosetests)
endif()

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake;${CMAKE_MODULE_PATH}")

file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/cmake/find_numpy_include_dir.py
     DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_ETC_DESTINATION})

cs_install()

cs_export(
    CFG_EXTRAS "create_swig_python_module.cmake" "mac_include_numpy.cmake"
    )

catkin_add_gtest(test_eigen_matrix_compare test/test_eigen_matrix_compare.cpp)
target_link_libraries(test_eigen_matrix_compare Eigen3::Eigen)

catkin_add_gtest(test_eigen_tensor_compare test/test_eigen_tensor_compare.cpp)
target_link_libraries(test_eigen_tensor_compare Eigen3::Eigen)

file(GLOB python_scripts scripts/*.py)
catkin_install_python(PROGRAMS ${python_scripts}
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

file(GLOB_RECURSE EXTRA_FILES */*)
add_custom_target(${PROJECT_NAME}_OTHER_FILES ALL WORKING_DIRECTORY ${PROJECT_SOURCE_DIR} SOURCES ${EXTRA_FILES})
