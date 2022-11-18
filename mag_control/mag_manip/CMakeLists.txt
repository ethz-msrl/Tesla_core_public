cmake_minimum_required(VERSION 3.0)
project(mag_manip)


find_package(catkin_simple REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(Ceres REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(${CERES_INCLUDE_DIRS} ${YAML_CPP_INCLUDE_DIR})

# turning off ALL_DEPS_REQUIRED because mag_tensorflow can be turned off
catkin_simple()

add_definitions(-std=c++11)

option(MAG_MANIP_BUILD_PYTHON "option to compile the Python bindings using SWIG" ON)

option(DISABLE_TENSORFLOW "Turns the parts that need tensorflow on or off" OFF)
if (DISABLE_TENSORFLOW)
    add_definitions(-DNO_TENSORFLOW)
endif()

catkin_python_setup()

if (APPLE)
    mac_include_numpy()
endif()

set(SRC_LIST 
    src/emns_parameters_yaml.cpp
    src/emns_parameters_rosparam.cpp
    src/emns_parameters_conversions.cpp
    src/forward_model.cpp
    src/forward_model_factory.cpp
    src/forward_model_linear.cpp
    src/forward_model_linear_factory.cpp
    src/forward_model_saturation.cpp
    src/forward_model_mpem.cpp
    src/forward_model_linear_vfield.cpp
    src/forward_model_linear_rbf.cpp
    src/forward_model_linear_thinplatespline.cpp
    src/forward_model_linear_saturation.cpp
    src/forward_model_linear_currents_jacobian_functor.cpp
    src/forward_model_linear_saturation_currents_jacobian_functor.cpp
    src/currents_jacobian_functor_factory.cpp
    src/backward_model.cpp
    src/backward_model_factory.cpp
    src/backward_model_linear_L2.cpp
    src/backward_model_mpem_L2.cpp
    src/backward_model_linear_vfield_L2.cpp
    src/backward_model_linear_rbf_L2.cpp
    src/backward_model_linear_thinplatespline_L2.cpp
    src/backward_model_nls.cpp
    src/backward_model_nls_functors.cpp
    src/backward_model_saturation.cpp
    src/backward_model_linear_L2_saturation.cpp
    src/saturation_function_factory.cpp
    src/saturation.cpp
    src/vfield.cpp
    src/interpolate_regular.cpp
    src/interpolate_trilinear.cpp
    src/interpolate_tricubic.cpp
    src/interpolate_tricubic_scalar_field.cpp
    src/rbf_3d_field_interpolator.cpp)

if(NOT DISABLE_TENSORFLOW) 
    message(STATUS "Adding TensorFlow models")
    list(APPEND SRC_LIST
    src/forward_model_tensorflow.cpp)
endif()

cs_add_library(${PROJECT_NAME} 
        #  src/helpers.cpp
        ${SRC_LIST}
        )

      target_link_libraries(${PROJECT_NAME} yaml-cpp Eigen3::Eigen ceres)

add_subdirectory(test)
if (MAG_MANIP_BUILD_PYTHON)
    add_subdirectory(python)
endif()

add_subdirectory(bench)

if (CATKIN_ENABLE_TESTING AND MAG_MANIP_BUILD_PYTHON)
    # we need to add a dependency on the SWIG shared library that is created _mag_manip
    # or the tests can be run at a time where the python module is not ready yet
    catkin_add_nosetests(nosetests/test_mag_manip.py
        DEPENDENCIES 
        _mag_manip)
endif()

install(DIRECTORY ${mag_manip_SOURCE_DIR}/params/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/params
        )

cs_install()
cs_export(
        INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR}
)
