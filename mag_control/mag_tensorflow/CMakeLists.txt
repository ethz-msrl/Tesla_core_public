cmake_minimum_required(VERSION 3.5)
project(mag_tensorflow)

SET(CMAKE_CXX_STANDARD 11)

find_package(catkin_simple REQUIRED)
catkin_simple()

option(TENSORFLOW_USE_GPU "Use the Tensorflow API compiled for GPUs. Linux Only." OFF)
set(TENSORFLOW_VERSION 1.15.0)

# If the DISABLE_TENSORFLOW switch is set to on, this package does nothing so that it 
# doesn't get linked in any dependent packages
if(NOT DISABLE_TENSORFLOW)
    if (NOT ${CMAKE_SYSTEM_NAME} MATCHES Linux)
        message(FATAL_ERROR "Only Linux is supported at the moment")
    endif()

# The configure step fails at catkin_package() if this directory
# doesn't exist yet. Create it at configure time!
    file(MAKE_DIRECTORY ${CATKIN_DEVEL_PREFIX}/include)

# this is the directory will ExternalProject_ADD will untar the tensorflow headers
# these will then get moved into the catkin devel space
    set(tensorflow_TEMP_INCLUDE_DIR ${CMAKE_CURRENT_BINARY_DIR}/tensorflow-prefix/src/tensorflow/include/tensorflow)
    set(tensorflow_TEMP_LIB_DIR ${CMAKE_CURRENT_BINARY_DIR}/tensorflow-prefix/src/tensorflow/lib)

    if(TENSORFLOW_USE_GPU) 
        set(TF_URL
            https://storage.googleapis.com/tensorflow/libtensorflow/libtensorflow-gpu-linux-x86_64-${TENSORFLOW_VERSION}.tar.gz)
        set(TF_URL_MD5 164d98d460f8028f5e33f3651d027c77)
    else()
        set(TF_URL
            https://storage.googleapis.com/tensorflow/libtensorflow/libtensorflow-cpu-linux-x86_64-${TENSORFLOW_VERSION}.tar.gz)
        set(TF_URL_MD5 f6f9e49ccbf194c309b89b0a6396f678)
    endif()

    include(ExternalProject)
    ExternalProject_Add(tensorflow
        URL ${TF_URL}
        URL_MD5 ${TF_URL_MD5}
        CONFIGURE_COMMAND ""
        BUILD_COMMAND ""
        INSTALL_COMMAND cp -nr ${tensorflow_TEMP_INCLUDE_DIR} 
        ${CATKIN_DEVEL_PREFIX}/include
        && cp -a ${tensorflow_TEMP_LIB_DIR}/. ${CATKIN_DEVEL_PREFIX}/lib
        )

    set(tensorflow_INCLUDE_DIR ${CATKIN_DEVEL_PREFIX}/include)
    set(tensorflow_LIBRARY ${CATKIN_DEVEL_PREFIX}/lib/libtensorflow.so)

    include_directories(${tensorflow_INCLUDE_DIR})

    cs_add_library(mag_tensorflow src/model.cpp)
    target_link_libraries(mag_tensorflow ${tensorflow_LIBRARY})
    add_dependencies(mag_tensorflow tensorflow)

    cs_add_executable(hello-world src/hello-world.c)
    target_link_libraries(hello-world ${tensorflow_LIBRARY})
    add_dependencies(hello-world tensorflow)

    catkin_add_gtest(test_model test/test_model.cpp)
    target_link_libraries(test_model mag_tensorflow ${catkin_LIBRARIES} ${tensorflow_LIBRARY})
    add_dependencies(test_model tensorflow)

    install(DIRECTORY ${CATKIN_DEVEL_PREFIX}/include/tensorflow
        DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        )

    install(DIRECTORY ${CATKIN_DEVEL_PREFIX}/lib/
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        FILES_MATCHING PATTERN "libtensorflow*"
        )

    cs_install()
    cs_export(INCLUDE_DIRS ${${PROJECT_NAME}_LOCAL_INCLUDE_DIR} ${CS_PROJECT_INCLUDE_DIRS} ${tensorflow_INCLUDE_DIR}
        LIBRARIES tensorflow)

    set(default_model_file cmag_cnn_v1.tar CACHE STRING "The default model tar containing the SavedModel")
    add_custom_command( TARGET mag_tensorflow PRE_BUILD
            COMMAND tar xvf ${default_model_file}
            WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/models
            DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/models/${default_model_file}
            COMMENT "Untarring ${default_model_file}"
            VERBATIM)
       
# we need to put these in the install space or they are not found with rospack
# when using the install
    install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/models
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
else()
    message(WARNING "mag_tensorflow has been disabled by the flag DISABLE_TENSORFLOW")
endif(NOT DISABLE_TENSORFLOW)

