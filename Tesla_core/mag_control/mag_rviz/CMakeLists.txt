cmake_minimum_required(VERSION 3.0.2)
project(mag_rviz)

find_package(catkin REQUIRED COMPONENTS rviz mag_msgs tf)
catkin_package()

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src)
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

## This plugin includes Qt widgets, so we must include Qt.
## We'll use the version that rviz used so they are compatible.
if(rviz_QT_VERSION VERSION_LESS "5")
    message(STATUS "Using Qt4 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
    find_package(Qt4 ${rviz_QT_VERSION} EXACT REQUIRED QtCore QtGui)
    ## pull in all required include dirs, define QT_LIBRARIES, etc.
    include(${QT_USE_FILE})
else()
    message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
    find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
    ## make target_link_libraries(${QT_LIBRARIES}) pull in all required dependencies
    set(QT_LIBRARIES Qt5::Widgets)
endif()

include_directories(${QT_INCLUDE_DIR})

set(CMAKE_AUTOMOC ON)
#add_definitions(-DQT_NO_KEYWORDS)

set(SRC_FILES			src/mag_rviz/magfield_visual.cpp
                        src/mag_rviz/magfield_display.cpp
                        src/mag_rviz/magfieldarray_visual.cpp
                        src/mag_rviz/magfieldarray_display.cpp
                        src/mag_rviz/maggrad3_display.cpp
                        src/mag_rviz/maggrad5_display.cpp
                        src/mag_rviz/maggrad3_visual.cpp
                        src/mag_rviz/currents_visual.cpp
                        src/mag_rviz/currents_panel.cpp
                        src/mag_rviz/send_currents_panel.cpp
                        src/mag_rviz/magfield_panel.cpp
                        src/mag_rviz/magfield_rot_panel.cpp
                        src/mag_rviz/dipolegradient_panel.cpp
                        src/mag_rviz/dipolegradient_display.cpp
                        src/mag_rviz/dipolegradient_visual.cpp
                        src/mag_rviz/tracking_update.cpp
                        src/mag_rviz/view_publisher.cpp
                        src/mag_rviz/ps4_visual.cpp)                                

add_library(${PROJECT_NAME} ${SRC_FILES})
target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} ${catkin_LIBRARIES} ${rviz_DEFAULT_PLUGIN_LIBRARIES})

#Add custom (non compiling) targets so launch scripts and python files show up in QT Creator's project view.
file(GLOB_RECURSE EXTRA_FILES */*)
add_custom_target(${PROJECT_NAME}_OTHER_FILES ALL WORKING_DIRECTORY ${PROJECT_SOURCE_DIR} SOURCES ${EXTRA_FILES})

install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
  plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
