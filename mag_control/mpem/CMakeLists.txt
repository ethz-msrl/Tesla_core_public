cmake_minimum_required(VERSION 2.8.3)
project(mpem)

find_package(catkin_simple REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

catkin_simple()

add_definitions(-std=c++11 -Wall -Wextra)

catkin_python_setup()

# On Mac OSX, Homebrew puts the Numpy C headers in the python site-packages
# Directory. We need to find the include ourselves.
if (APPLE)
    set(CMAKE_MACOSX_RPATH True)
    set(CMAKE_INSTALL_RPATH_USE_LINK_PATH ON)
    mac_include_numpy()
endif()

cs_add_library(${PROJECT_NAME} 
        src/electromagnet_calibration.cpp
        src/scalorPotential.cpp
        src/PseudoInverse.cpp
        )
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen yaml-cpp)

cs_add_executable(dipole-model-fit
    src/dipole-model-fit.cpp)
target_link_libraries(dipole-model-fit mpem Eigen3::Eigen yaml-cpp)

catkin_add_gtest(test_electromagnet_calibration test/test_electromagnet_calibration.cpp)
target_link_libraries(test_electromagnet_calibration ${catkin_LIBRARIES} ${PROJECT_NAME})


add_subdirectory(python)

if (CATKIN_ENABLE_TESTING)
    # we need to add a dependency on the SWIG shared library that is created _mpem
    # or the tests can be run at a time where the python module is not ready yet
    catkin_add_nosetests(nosetests/test_mpem.py
        DEPENDENCIES 
        _mpem)
endif()

cs_install()

install(DIRECTORY ${mpem_SOURCE_DIR}/cal/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/cal
        )

cs_export()


# Add custom (non compiling) targets so launch scripts and python files show up in QT Creator's project view.
file(GLOB_RECURSE EXTRA_FILES */*)
add_custom_target(${PROJECT_NAME}_OTHER_FILES ALL WORKING_DIRECTORY ${PROJECT_SOURCE_DIR} SOURCES ${EXTRA_FILES})
