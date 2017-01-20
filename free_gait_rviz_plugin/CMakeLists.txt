cmake_minimum_required(VERSION 2.8.3)
project(free_gait_rviz_plugin)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  rviz
  free_gait_core
  free_gait_ros
  free_gait_msgs
)

catkin_package(
  INCLUDE_DIRS
    include
    src
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    rviz
    free_gait_core
    free_gait_ros
    free_gait_msgs
  DEPENDS
)

include_directories(
  include
  src
  ${catkin_INCLUDE_DIRS}
)

link_directories(${catkin_LIBRARY_DIRS})

## This setting causes Qt's "MOC" generation to happen automatically.
set(CMAKE_AUTOMOC ON)

set(INCLUDE_FILES
  include/free_gait_rviz_plugin/FreeGaitPreviewDisplay.hpp
)

## This plugin includes Qt widgets, so we must include Qt.
## We'll use the version that rviz used so they are compatible.
if(rviz_QT_VERSION VERSION_LESS "5")
  message(STATUS "Using Qt4 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt4 ${rviz_QT_VERSION} EXACT REQUIRED QtCore QtGui)
  ## pull in all required include dirs, define QT_LIBRARIES, etc.
  include(${QT_USE_FILE})
    qt4_wrap_cpp(MOC_FILES
    ${INCLUDE_FILES}
  )
else()
  message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
  ## make target_link_libraries(${QT_LIBRARIES}) pull in all required dependencies
  set(QT_LIBRARIES Qt5::Widgets)
    QT5_WRAP_CPP(MOC_FILES
    ${INCLUDE_FILES}
  )
endif()

## Avoid Qt signals and slots defining "emit", "slots", etc.
add_definitions(-DQT_NO_KEYWORDS)

## The list of source files.
## The generated MOC files are included automatically as headers.
set(SOURCE_FILES
  src/FreeGaitPreviewDisplay.cpp
  src/FreeGaitPreviewPlayback.cpp
  src/FreeGaitPreviewVisual.cpp
)

## An rviz plugin is just a shared library, so here we declare the
## library to be called ${PROJECT_NAME} and specify the list of
## source files we collected above in ${SOURCE_FILES}.

add_library(rviz_free_gait_additions
  src/rviz/properties/button_property.cpp
  src/rviz/properties/button_toggle_property.cpp
  src/rviz/properties/float_slider_property.cpp
  src/rviz/properties/line_edit_with_slider.cpp
)

add_library(${PROJECT_NAME}
  ${SOURCE_FILES}
  ${MOC_FILES}
)

## Link the executable with whatever Qt libraries have been defined by
## the find_package(Qt4 ...) line above, or by the
## set(QT_LIBRARIES Qt5::Widgets), and with whatever libraries
## catkin has included.

target_link_libraries(rviz_free_gait_additions
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  ${QT_LIBRARIES}
  ${OGRE_OV_LIBRARIES_ABS}
  ${OPENGL_LIBRARIES}
  ${rviz_ADDITIONAL_LIBRARIES}
  ${X11_X11_LIB}
  assimp
)

target_link_libraries(${PROJECT_NAME}
  ${QT_LIBRARIES}
  ${catkin_LIBRARIES}
  rviz_free_gait_additions
)

## Install rules
install(
  DIRECTORY include/${PROJECT_NAME}
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(
TARGETS ${PROJECT_NAME} rviz_free_gait_additions
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  FILES plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  DIRECTORY icons
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/icons
)
